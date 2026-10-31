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

    /// <summary>Driving distance after which the wizard builds the AB line from the driven pass.</summary>
    private const double AutoLineDistanceMetres = 15.0;

    // Where wizard step 2 (no guidance line yet) started. Local plane for the distance,
    // lat/lon for the line itself (it is built through the driven points, like a typed AB).
    private double? _wizardLineStartE;
    private double? _wizardLineStartN;
    private double? _wizardLineStartLat;
    private double? _wizardLineStartLon;

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
        _wizardLineStartE = null; // step 2 will pick its start point on the next cycle

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
        _wizardLineStartE = null;
        PublishCalibration(null);
    }

    /// <summary>
    /// One GPS cycle's worth of sampling, on the UI thread from
    /// <see cref="ApplyGpsCycleResult"/>. Returns immediately unless a recording is running.
    /// </summary>
    private void ObserveCalibration(GpsCycleResult result)
    {
        if (!_antennaCalibration.IsCollecting) return;

        var line = State.Guidance.DisplayLine is { Count: >= 2 } followed
            ? followed
            : State.Field.ActiveTrack?.Points;
        if (line is null || line.Count < 2)
        {
            State.Calibration.LastRejectReason = "no guidance line yet";
            return;
        }

        // Line-relative deviation of the RAW antenna (positive = right of A→B), so the
        // two directions differ only by the direction sign — see AntennaCalibrationSample.
        // Measured against the line the vehicle is STEERED to (the followed, offset line for
        // the current pass). The base track can be a whole pass away — a driven A+ line sits
        // at the implement's edge, which reads as ~half a tool width (1.79 m) of "antenna
        // error" while the vehicle is perfectly on its line.
        double deviation = CalibrationGeometry.SignedDistanceToLine(
            line, result.AntennaEasting, result.AntennaNorthing);

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

    /// <summary>
    /// Wizard step 2: while there is no guidance line, the operator just drives forward and
    /// the line is built from the segment they drove — A where this step started, B
    /// <see cref="AutoLineDistanceMetres"/> later. Nothing to open, nothing to place by hand;
    /// <c>calibration.wizard.track</c> stays available as a manual override.
    /// </summary>
    private void UpdateCalibrationLine(GpsCycleResult result)
    {
        if (!_antennaCalibration.IsCollecting)
        {
            _wizardLineStartE = null;
            return;
        }

        // A line already exists (or the operator made one): nothing to build.
        if (State.Field.ActiveTrack is not null)
        {
            _wizardLineStartE = null;
            State.Calibration.LineDistanceMeters = 0;
            return;
        }

        if (_wizardLineStartE is null || _wizardLineStartLat is null || _wizardLineStartLon is null)
        {
            _wizardLineStartE = result.Easting;
            _wizardLineStartN = result.Northing;
            _wizardLineStartLat = result.Latitude;
            _wizardLineStartLon = result.Longitude;
            State.Calibration.LineDistanceMeters = 0;
            return;
        }

        double de = result.Easting - _wizardLineStartE.Value;
        double dn = result.Northing - _wizardLineStartN!.Value;
        double driven = Math.Sqrt((de * de) + (dn * dn));
        State.Calibration.LineDistanceMeters = driven;

        if (driven < AutoLineDistanceMetres) return;

        double latA = _wizardLineStartLat.Value;
        double lonA = _wizardLineStartLon.Value;
        _wizardLineStartE = null;
        _wizardLineStartN = null;
        _wizardLineStartLat = null;
        _wizardLineStartLon = null;
        State.Calibration.LineDistanceMeters = 0;

        // The ordinary typed AB: heading runs from A to B, so the line follows what was driven.
        CreateLatLonABLine(latA, lonA, result.Latitude, result.Longitude);
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