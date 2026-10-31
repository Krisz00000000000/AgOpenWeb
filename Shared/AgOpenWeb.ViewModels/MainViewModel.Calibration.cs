// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using AgOpenWeb.Models.State;
using AgOpenWeb.Services.Calibration;

namespace AgOpenWeb.ViewModels;

/// <summary>
/// Automatic calibration menu (the web UI's Calibration panel).
///
/// The algorithms only ever produce a <b>proposal</b>: recording and fitting happen here,
/// and the operator applies a result by writing the two vehicle fields through the normal
/// <c>config.set</c> path — so the existing save/persist route runs and nothing is written
/// behind the operator's back. Algorithm 1 (antenna lateral offset + height) is live;
/// 2–6 are placeholders in the menu until their own steps land.
/// </summary>
public partial class MainViewModel
{
    /// <summary>Antenna offset + height — algorithm 1 of the calibration plan.</summary>
    private const int AntennaCalibrationAlgorithm = 1;

    /// <summary>
    /// Re-fit every this many new samples (and on start/stop/reset). Fitting per cycle
    /// would allocate a handful of arrays 10×/s for a number that moves by millimetres.
    /// </summary>
    private const int FitEverySamples = 25;

    private readonly AntennaCalibrationCollector _antennaCalibration = new();
    private int _calibrationAlgorithm;

    /// <summary>True while an automatic calibration is recording.</summary>
    public bool IsCalibrationRecording => _antennaCalibration.IsCollecting;

    /// <summary>
    /// Start recording an algorithm. An unknown algorithm id is ignored (the menu shows
    /// 2–6 as placeholders, so a stale client cannot start something unimplemented).
    /// </summary>
    public void StartAutomaticCalibration(int algorithm)
    {
        if (algorithm != AntennaCalibrationAlgorithm) return;

        _calibrationAlgorithm = algorithm;
        _antennaCalibration.Reset();
        _antennaCalibration.Start();

        // Publish straight away so the panel shows what is still missing rather than zeros.
        PublishCalibration(_antennaCalibration.Fit());
    }

    /// <summary>Stop recording. The samples and the last fit are kept.</summary>
    public void StopAutomaticCalibration()
    {
        _antennaCalibration.Stop();
        PublishCalibration(_antennaCalibration.Fit());
    }

    /// <summary>Drop the recording and clear the panel.</summary>
    public void ResetAutomaticCalibration()
    {
        _antennaCalibration.Reset();
        _calibrationAlgorithm = 0;
        PublishCalibration(null);
    }

    /// <summary>
    /// One GPS cycle's worth of sampling, on the UI thread from
    /// <see cref="ApplyGpsCycleResult"/>. Returns immediately unless a recording is running.
    /// </summary>
    private void ObserveCalibration(GpsCycleResult result)
    {
        if (!_antennaCalibration.IsCollecting) return;

        var track = State.Field.ActiveTrack;
        if (track is null || track.Points.Count < 2)
        {
            State.Calibration.LastRejectReason = "no active track";
            return;
        }

        // Line-relative deviation of the RAW antenna (positive = right of A→B), so the
        // two directions differ only by the direction sign — see AntennaCalibrationSample.
        double deviation = CalibrationGeometry.SignedDistanceToTrack(
            track, result.AntennaEasting, result.AntennaNorthing);

        bool kept = _antennaCalibration.TryAdd(
            result.IsHeadingSameWay ? 1.0 : -1.0,
            result.RollDegrees,
            deviation,
            result.FixQuality,
            result.Speed,
            out string? rejection);

        State.Calibration.LastRejectReason = rejection ?? "";

        if (kept && _antennaCalibration.Count % FitEverySamples == 0)
            PublishCalibration(_antennaCalibration.Fit());
        else
            PublishCalibrationCounters();
    }

    private void PublishCalibration(AntennaCalibrationReport? report)
    {
        var cal = State.Calibration;
        PublishCalibrationCounters();

        if (report is null)
        {
            cal.IsDeterminate = false;
            cal.IsAccepted = false;
            cal.LateralOffsetMeters = 0.0;
            cal.HeightMeters = 0.0;
            cal.LateralOffsetStdDevMeters = double.NaN;
            cal.HeightStdDevMeters = double.NaN;
            cal.Message = "";
            cal.LastRejectReason = "";
            return;
        }

        cal.IsDeterminate = report.IsDeterminate;
        cal.IsAccepted = report.IsAccepted;
        cal.Message = report.Message;

        if (report.Fit is { } fit)
        {
            cal.LateralOffsetMeters = fit.LateralOffsetMeters;
            cal.HeightMeters = fit.HeightAboveRollAxisMeters;
            cal.LateralOffsetStdDevMeters = fit.LateralOffsetStdDevMeters;
            cal.HeightStdDevMeters = fit.HeightStdDevMeters;
        }
        else
        {
            cal.LateralOffsetMeters = 0.0;
            cal.HeightMeters = 0.0;
            cal.LateralOffsetStdDevMeters = double.NaN;
            cal.HeightStdDevMeters = double.NaN;
        }
    }

    private void PublishCalibrationCounters()
    {
        var cal = State.Calibration;
        cal.Algorithm = _calibrationAlgorithm;
        cal.IsRecording = _antennaCalibration.IsCollecting;
        cal.ForwardSamples = _antennaCalibration.ForwardCount;
        cal.ReverseSamples = _antennaCalibration.ReverseCount;
        cal.RollRangeDegrees = _antennaCalibration.RollRangeDegrees;

        // Which guided step the operator is on. Field/line come from the app's own state, so
        // the wizard follows whatever the operator (or the wizard itself) just created.
        var stage = CalibrationWizard.Evaluate(
            State.Field.ActiveField is not null,
            State.Field.ActiveTrack is not null,
            cal.ForwardSamples,
            cal.ReverseSamples,
            cal.RollRangeDegrees);
        cal.Step = CalibrationWizard.StepNumber(stage);
        cal.StepKey = CalibrationWizard.Key(stage);
    }
}