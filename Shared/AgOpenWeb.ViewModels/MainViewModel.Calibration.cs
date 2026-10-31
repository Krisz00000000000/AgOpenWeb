// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using AgOpenWeb.Models.State;
using AgOpenWeb.Models.Timing;
using AgOpenWeb.Services.Calibration;

namespace AgOpenWeb.ViewModels;

/// <summary>
/// Automatic calibration menu (the web UI's Calibration panel).
///
/// The algorithms only ever produce a <b>proposal</b>: recording and fitting happen here,
/// and the operator applies a result by writing the settings through the normal
/// <c>config.set</c> path — so the existing save/persist route runs and nothing is written
/// behind the operator's back.
///
/// Wired: 1 (antenna lateral offset + height, guided wizard), 2 (WAS zero + gain),
/// 4 (cross-track integrator, off by default) and 5 (pass-drift watcher, observer only).
/// 3 (steering response) stays greyed in the menu: it is the only algorithm that moves the
/// steering, and the plan requires its gated test path to be reviewed by a human first.
/// 6 (sensor fusion) is not started, as the plan asks.
/// </summary>
public partial class MainViewModel
{
    /// <summary>Antenna offset + height — algorithm 1 of the calibration plan.</summary>
    private const int AntennaCalibrationAlgorithm = 1;

    /// <summary>WAS zero + gain — algorithm 2.</summary>
    private const int WasCalibrationAlgorithm = 2;

    /// <summary>Cross-track integrator — algorithm 4.</summary>
    private const int XteIntegratorAlgorithm = 4;

    /// <summary>Pass-drift watcher — algorithm 5 (observer mode: it corrects nothing).</summary>
    private const int PassDriftAlgorithm = 5;

    /// <summary>
    /// Re-fit every this many new samples (and on start/stop/reset). Fitting per cycle
    /// would allocate a handful of arrays 10×/s for a number that moves by millimetres.
    /// </summary>
    private const int FitEverySamples = 25;

    /// <summary>Driving distance after which the wizard builds the AB line from the driven pass.</summary>
    private const double AutoLineDistanceMetres = 15.0;

    /// <summary>
    /// Probe the neighbour seam every this many cycles while the drift watcher is armed
    /// (≈2 Hz at a 10 Hz fix rate): one pass needs a handful of probes, not one per fix, and
    /// each probe walks the coverage map outwards from the swath edge.
    /// </summary>
    private const int DriftProbeEveryCycles = 5;

    private readonly AntennaCalibrationCollector _antennaCalibration = new();
    private readonly WasCalibrationCollector _wasCalibration = new();
    private readonly PassDriftWatcher _passDrift = new();
    private int _calibrationAlgorithm;

    // Algorithm 4 — the integrator's options are immutable, so switching it on builds a new
    // instance (Enabled = true) and switching it off detaches and drops it: the correction is
    // deliberately not persisted (the plan), and a disabled integrator can never leak a
    // leftover correction into the steering.
    private XteIntegrator? _xteIntegrator;
    private long _lastXteUpdateTicks;

    // Algorithm 5 — the pass being accumulated. A pass ends when the guidance pass offset
    // changes (a U-turn, or an on-screen pass change); its mean seam deviation is what the
    // watcher is fed.
    private bool _driftArmed;
    private int? _driftCurrentPass;
    private double _driftPassSum;
    private int _driftPassSamples;
    private double _driftPassDirectionSign = 1.0;
    private int _driftProbeCounter;

    // Where wizard step 2 (no guidance line yet) started. Local plane for the distance,
    // lat/lon for the line itself (it is built through the driven points, like a typed AB).
    private double? _wizardLineStartE;
    private double? _wizardLineStartN;
    private double? _wizardLineStartLat;
    private double? _wizardLineStartLon;

    /// <summary>True while an automatic calibration is recording (algorithm 1 or 2), or while
    /// the live ones (4, 5) are switched on for this session.</summary>
    public bool IsCalibrationRecording => _calibrationAlgorithm != 0;

    /// <summary>
    /// Start an algorithm. An unknown id — and 3, whose gated test path is waiting for its
    /// safety review — is ignored, so a stale client cannot start something unimplemented or
    /// something that moves the steering. Starting one stops whatever ran before.
    /// </summary>
    /// <param name="algorithm">1 antenna, 2 WAS, 4 integrator, 5 pass drift.</param>
    public void StartAutomaticCalibration(int algorithm)
    {
        if (algorithm is not (AntennaCalibrationAlgorithm or WasCalibrationAlgorithm
            or XteIntegratorAlgorithm or PassDriftAlgorithm))
        {
            return;
        }

        StopCurrentCalibration();
        _calibrationAlgorithm = algorithm;

        switch (algorithm)
        {
            case AntennaCalibrationAlgorithm:
                _antennaCalibration.Reset();
                _antennaCalibration.Start();
                _wizardLineStartE = null; // step 2 will pick its start point on the next cycle
                PublishCalibration(_antennaCalibration.Fit());
                break;

            case WasCalibrationAlgorithm:
                _wasCalibration.Reset();
                _wasCalibration.Start();
                PublishWasCalibration();
                break;

            case XteIntegratorAlgorithm:
                _xteIntegrator = new XteIntegrator(new XteIntegratorOptions { Enabled = true });
                _lastXteUpdateTicks = Clock.Current.GetTimestamp();
                _trackGuidanceService.SetXteIntegrator(_xteIntegrator);
                PublishXteIntegrator(null);
                break;

            case PassDriftAlgorithm:
                _passDrift.Reset();
                ResetDriftPass();
                _driftArmed = true;
                PublishDrift("waiting for a pass");
                break;
        }
    }

    /// <summary>Stop the running algorithm. Samples, fits and the drift history are kept.</summary>
    public void StopAutomaticCalibration()
    {
        int stopped = _calibrationAlgorithm;

        // A pass in progress ends here: its probes are still a valid measurement.
        if (stopped == PassDriftAlgorithm) CommitDriftPass();

        StopCurrentCalibration(); // detaches the integrator before the panel is told it is off

        switch (stopped)
        {
            case AntennaCalibrationAlgorithm:
                PublishCalibration(_antennaCalibration.Fit());
                break;
            case WasCalibrationAlgorithm:
                PublishWasCalibration();
                break;
            case XteIntegratorAlgorithm:
                PublishXteIntegrator(null);
                break;
            case PassDriftAlgorithm:
                PublishDrift(State.Calibration.DriftPasses > 0 ? "" : "stopped");
                break;
        }
    }

    /// <summary>Drop the recording and clear the panel.</summary>
    public void ResetAutomaticCalibration()
    {
        StopCurrentCalibration();

        _wizardLineStartE = null;
        _antennaCalibration.Reset();
        _wasCalibration.Reset();
        _passDrift.Reset();
        ResetDriftPass();
        _driftArmed = false;

        PublishCalibration(null);
        PublishWasCalibration();
        PublishXteIntegrator(null);
        PublishDrift("");
    }

    /// <summary>Leave whatever is running in a defined state (start, stop and reset all use it).</summary>
    private void StopCurrentCalibration()
    {
        switch (_calibrationAlgorithm)
        {
            case AntennaCalibrationAlgorithm:
                _antennaCalibration.Stop();
                break;
            case WasCalibrationAlgorithm:
                _wasCalibration.Stop();
                break;
            case XteIntegratorAlgorithm:
                DetachXteIntegrator();
                break;
            case PassDriftAlgorithm:
                _driftArmed = false;
                break;
        }

        _calibrationAlgorithm = 0;
    }

    /// <summary>Detach the integrator; the steering laws are back on the unmodified error the
    /// moment this returns, and the correction cannot come back (it is not persisted).</summary>
    private void DetachXteIntegrator()
    {
        _trackGuidanceService.SetXteIntegrator(null);
        _xteIntegrator = null;
    }

    /// <summary>
    /// One GPS cycle's worth of sampling, on the UI thread from
    /// <see cref="ApplyGpsCycleResult"/>. Dispatches to whichever algorithm is switched on and
    /// returns immediately when none is.
    /// </summary>
    private void ObserveCalibration(GpsCycleResult result)
    {
        switch (_calibrationAlgorithm)
        {
            case AntennaCalibrationAlgorithm:
                ObserveAntennaCalibration(result);
                break;
            case WasCalibrationAlgorithm:
                ObserveWasCalibration(result);
                break;
            case XteIntegratorAlgorithm:
                ObserveXteIntegrator(result);
                break;
            case PassDriftAlgorithm:
                ObservePassDrift(result);
                break;
        }
    }

    // ── Algorithm 1: antenna lateral offset + height ────────────────────

    private void ObserveAntennaCalibration(GpsCycleResult result)
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
        if (_calibrationAlgorithm != AntennaCalibrationAlgorithm)
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
        cal.IsRecording = _calibrationAlgorithm is AntennaCalibrationAlgorithm or WasCalibrationAlgorithm;
        cal.ForwardSamples = _antennaCalibration.ForwardCount;
        cal.ReverseSamples = _antennaCalibration.ReverseCount;
        cal.RollRangeDegrees = _antennaCalibration.RollRangeDegrees;

        // Which guided step the operator is on. Field/line come from the app's own state, so
        // the wizard follows whatever the operator (or the wizard itself) just created. Only
        // algorithm 1 is a guided wizard — 2, 4 and 5 drive their own panels.
        if (_calibrationAlgorithm != AntennaCalibrationAlgorithm)
        {
            cal.Step = 1;
            cal.StepKey = "field";
            cal.LineDistanceMeters = 0;
            return;
        }

        var stage = CalibrationWizard.Evaluate(
            State.Field.ActiveField is not null,
            State.Field.ActiveTrack is not null,
            cal.ForwardSamples,
            cal.ReverseSamples,
            cal.RollRangeDegrees);
        cal.Step = CalibrationWizard.StepNumber(stage);
        cal.StepKey = CalibrationWizard.Key(stage);
    }

    // ── Algorithm 2: WAS zero + gain ────────────────────────────────────

    /// <summary>
    /// One observation: the steer module's measured angle (PGN 253) against the yaw rate the
    /// vehicle actually turned with, at the speed and roll the fit's filters demand. The
    /// plan's weave: flat, empty ground, 1.5–3 m/s, ±10–15°, autosteer on.
    /// </summary>
    private void ObserveWasCalibration(GpsCycleResult result)
    {
        var cal = State.Calibration;
        if (!_wasCalibration.IsCollecting) return;

        if (!result.GpsValid)
        {
            cal.WasRejectReason = "no GPS fix";
            PublishWasCalibrationCounters();
            return;
        }

        // GpsCycleResult.Speed is m/s, the same unit WasCalibrationCollector filters on.
        double speedMps = result.Speed;
        double measured = _autoSteerService.LastSteerData.ActualSteerAngle;

        // The collector's convention is "yaw rate positive = turning left" (the plan's table);
        // AgOpenWeb's derived rate is positive when the compass heading grows, i.e. turning
        // right — so it is negated here. The fit measures the sign and the panel says so when
        // the fitted gain comes out negative (inverted WAS or an inverted gyro sign).
        double yawRate = -result.YawRateRadPerSecond;

        bool kept = _wasCalibration.TryAdd(
            measured, yawRate, speedMps, result.RollDegrees, out string? rejection);

        cal.WasRejectReason = rejection ?? "";

        if (kept && _wasCalibration.Count % FitEverySamples == 0) PublishWasCalibration();
        else PublishWasCalibrationCounters();
    }

    /// <summary>Fit the WAS recording and publish the proposal the panel's Apply would write.</summary>
    private void PublishWasCalibration()
    {
        var cal = State.Calibration;
        PublishWasCalibrationCounters();

        double wheelbase = Vehicle.Wheelbase;
        if (!(wheelbase > 0.0))
        {
            // δ_true = atan(L·ω/v): without the wheelbase the two unknowns cannot be separated,
            // so refuse rather than publish a fit that means nothing.
            ClearWasFit("set the vehicle wheelbase first — the fit needs it");
            ClearWasProposal("the proposal needs a wheelbase");
            return;
        }

        var report = _wasCalibration.Fit(wheelbase);
        cal.WasDeterminate = report.IsDeterminate;
        cal.WasAccepted = report.IsAccepted;
        cal.WasMessage = report.Message;

        if (report.Fit is not { } fit)
        {
            ClearWasFit(cal.WasMessage);
            ClearWasProposal(cal.WasMessage);
            return;
        }

        cal.WasGainObservable = fit.GainObservable;
        cal.WasZeroDegrees = fit.MeasuredSteerAngleOffsetDegrees;
        cal.WasZeroStdDevDegrees = fit.ZeroStdDevDegrees;
        cal.WasGain = fit.Gain;
        cal.WasGainStdDev = fit.GainStdDev;

        if (WasZeroGainApplication.TryCompute(
            ConfigStore.AutoSteer.WasOffset, ConfigStore.AutoSteer.CountsPerDegree, fit,
            out var settings))
        {
            cal.WasProposedOffset = settings.WasOffset;
            cal.WasProposedCountsPerDegree = settings.CountsPerDegree;
            cal.WasProposedOffsetDelta = settings.WasOffsetDeltaCounts;
            cal.WasGainApplied = settings.GainApplied;
            // The gates decide whether Apply lights up; the numbers are still shown so the
            // operator can see what the run is heading towards.
            cal.WasApplyValid = report.IsAccepted;
            cal.WasApplyMessage = settings.Message;
        }
        else
        {
            ClearWasProposal(settings.Message);
        }
    }

    /// <summary>Counters only — the state between fits, so the panel shows progress.</summary>
    private void PublishWasCalibrationCounters()
    {
        var cal = State.Calibration;
        cal.Algorithm = _calibrationAlgorithm;
        cal.IsRecording = _wasCalibration.IsCollecting;
        cal.WasSamples = _wasCalibration.Count;
        cal.WasSteerRangeDegrees = _wasCalibration.SteerRangeDegrees;
    }

    private void ClearWasFit(string message)
    {
        var cal = State.Calibration;
        cal.WasDeterminate = false;
        cal.WasAccepted = false;
        cal.WasGainObservable = false;
        cal.WasZeroDegrees = 0.0;
        cal.WasZeroStdDevDegrees = double.NaN;
        cal.WasGain = 0.0;
        cal.WasGainStdDev = double.NaN;
        cal.WasMessage = message;
    }

    /// <summary>A proposal that cannot be applied says why, and carries the current values so
    /// the panel's readback stays honest.</summary>
    private void ClearWasProposal(string message)
    {
        var cal = State.Calibration;
        cal.WasProposedOffset = ConfigStore.AutoSteer.WasOffset;
        cal.WasProposedCountsPerDegree = ConfigStore.AutoSteer.CountsPerDegree;
        cal.WasProposedOffsetDelta = 0;
        cal.WasGainApplied = false;
        cal.WasApplyValid = false;
        cal.WasApplyMessage = message;
    }

    // ── Algorithm 4: cross-track integrator (live, off by default) ───────

    /// <summary>
    /// One integrating step per GPS fix, from the error the steering law was actually given and
    /// the frame the vehicle moved in. The step lives here, not in the guidance service, so the
    /// 100 Hz control path only ever <i>reads</i> the correction — the same correction therefore
    /// cannot advance several times per fix.
    /// </summary>
    private void ObserveXteIntegrator(GpsCycleResult result)
    {
        var cal = State.Calibration;
        var integrator = _xteIntegrator;
        cal.XteEnabled = integrator is not null;
        if (integrator is null)
        {
            cal.XteCorrectionMeters = 0.0;
            cal.XteSaturated = false;
            cal.XteMessage = "off";
            return;
        }

        long now = Clock.Current.GetTimestamp();
        double dt = Clock.Current.ElapsedSeconds(_lastXteUpdateTicks, now);
        _lastXteUpdateTicks = now;

        var guidance = result.Guidance;
        // Line-relative cross-track error: positive = right of A→B, the frame the integrator's
        // correction lives in. The published CrossTrackError is flipped for the reverse
        // direction (vehicle-relative), so it is un-flipped here with the direction sign.
        double lineRelativeError = (guidance?.CrossTrackError ?? 0.0)
            * (result.IsHeadingSameWay ? 1.0 : -1.0);
        double steer = guidance?.SteerAngle ?? 0.0;
        double maxSteer = Vehicle.MaxSteerAngle;
        bool saturated = maxSteer > 0.0 && Math.Abs(steer) >= maxSteer - 1e-9;
        bool hasGuidance = guidance?.HasGuidance == true;

        // The plan's reset points: a new line, the U-turn's transitions, and every engage or
        // disengage. A U-turn end and a disengage both arrive as flags on the cycle. The reset
        // cycle publishes the zeroed correction and does not integrate again on the same fix.
        if (result.AutoSteerDisengagedThisCycle || result.YouTurn?.JustCompleted == true)
        {
            integrator.Reset();
            _lastXteUpdateTicks = now;
            PublishXteIntegrator(new XteIntegratorState(0.0, false, null, hasGuidance));
            return;
        }

        integrator.Update(new XteIntegratorInput(
            lineRelativeError,
            dt,
            result.Speed,
            result.FixQuality == 4,
            result.IsAutoSteerEngaged && hasGuidance,
            saturated));

        PublishXteIntegrator(new XteIntegratorState(
            integrator.Correction, saturated, result, hasGuidance));
    }

    /// <summary>What the panel shows while the integrator is on (or just after it went off).</summary>
    private readonly record struct XteIntegratorState(
        double CorrectionMeters, bool Saturated, GpsCycleResult? Result, bool HasGuidance);

    private void PublishXteIntegrator(XteIntegratorState? state)
    {
        var cal = State.Calibration;
        cal.Algorithm = _calibrationAlgorithm;
        cal.IsRecording = false; // the integrator is live, not a recording
        if (_xteIntegrator is not { } integrator)
        {
            cal.XteEnabled = false;
            cal.XteCorrectionMeters = 0.0;
            cal.XteSaturated = false;
            cal.XteMessage = "off — switch it on to learn a steady correction (autosteer steers on the corrected error)";
            return;
        }

        cal.XteEnabled = true;

        // Switched on (or stopped) between cycles: no fix to report yet.
        if (state is not { } s)
        {
            cal.XteCorrectionMeters = integrator.Correction;
            cal.XteSaturated = false;
            cal.XteMessage = "starting from 0 cm — autosteer steers on the corrected error";
            return;
        }

        cal.XteCorrectionMeters = s.CorrectionMeters;
        cal.XteSaturated = s.Saturated;

        var result = s.Result;
        cal.XteMessage = (result, s.HasGuidance) switch
        {
            (null, _) => "starting from 0 cm — autosteer steers on the corrected error",
            (_, false) => "no guidance line yet",
            (_, _) when result!.FixQuality != 4 => "waiting for RTK Fixed — the error is not trusted without it",
            (_, _) when !result!.IsAutoSteerEngaged => "waiting for autosteer to be engaged",
            (_, _) when result!.Speed < integrator.Options.MinSpeedMps => "waiting for the vehicle to move",
            (_, _) when Math.Abs(integrator.Correction) >= integrator.Options.MaxCorrectionMeters - 1e-9
                => "correction at its limit",
            (_, _) when s.Saturated => "holding — the steering is saturated",
            _ => FormattableString.Invariant(
                $"correcting the cross-track error with {cal.XteCorrectionMeters * 100.0:F1} cm"),
        };
    }

    // ── Algorithm 5: pass-drift watcher (observer — corrects nothing) ────

    /// <summary>
    /// Records one deviation per pass: the seam between this swath and the neighbouring,
    /// already worked pass, measured against the live coverage map
    /// (<see cref="SwathSeamProbe"/>). The probes are averaged over the pass and handed to the
    /// watcher when the pass ends, so the watcher only ever sees per-pass values — it is not on
    /// the hot path.
    /// </summary>
    private void ObservePassDrift(GpsCycleResult result)
    {
        if (!_driftArmed) return;

        var cal = State.Calibration;
        var guidance = result.Guidance;
        bool hasGuidance = guidance?.HasGuidance == true;

        // A pass ends when the guidance pass offset changes (a U-turn, or an on-screen pass
        // change). The mean of the probes collected on it is this pass's deviation.
        if (hasGuidance && _driftCurrentPass != guidance!.HowManyPathsAway)
        {
            CommitDriftPass();
            _driftCurrentPass = guidance.HowManyPathsAway;
        }

        if (!hasGuidance)
        {
            PublishDrift("no guidance line — drive a worked pass");
            return;
        }

        if (!result.IsAutoSteerEngaged)
        {
            PublishDrift("autosteer off — the seam is only trusted while following the line");
            return;
        }

        if (!result.IsToolPositionReady || !(result.ToolWidth > 0.0))
        {
            PublishDrift("no tool width — set the implement up first");
            return;
        }

        if (++_driftProbeCounter < DriftProbeEveryCycles)
        {
            PublishDrift("measuring the seam");
            return;
        }

        _driftProbeCounter = 0;

        // Vehicle-relative deviation (positive = a gap on the probe side), which is the frame
        // the watcher's "same sign across directions" hint is written for: a direction-dependent
        // error flips sign with the driving direction, side-slip and wind do not.
        double? seam = SwathSeamProbe.Measure(
            _coverageMapService.IsPointCovered,
            result.ToolEasting,
            result.ToolNorthing,
            result.ToolHeadingRadians,
            result.ToolWidth);

        if (seam is { } deviation)
        {
            _driftPassSum += deviation;
            _driftPassSamples++;
            _driftPassDirectionSign = result.IsHeadingSameWay ? 1.0 : -1.0;
            PublishDrift(FormattableString.Invariant(
                $"pass {guidance.HowManyPathsAway + 1}: seam {deviation * 100.0:F1} cm ({_driftPassSamples} probes)"));
        }
        else
        {
            PublishDrift("no neighbouring pass beside this one yet — work alongside an already worked pass");
        }
    }

    /// <summary>Hand the finished pass to the watcher and publish the indicator.</summary>
    private void CommitDriftPass()
    {
        if (_driftPassSamples == 0)
        {
            ResetDriftPass();
            return;
        }

        double mean = _driftPassSum / _driftPassSamples;
        double directionSign = _driftPassDirectionSign;
        ResetDriftPass();

        var update = _passDrift.Add(mean, directionSign);
        var cal = State.Calibration;
        cal.DriftStatus = (int)update.Status;
        cal.DriftSmoothedMeters = update.SmoothedMeters;
        cal.DriftPasses = _passDrift.PassCount;
        cal.DriftSuspect = update.SameSignAcrossDirections;

        cal.DriftMessage = update.SameSignAcrossDirections
            ? FormattableString.Invariant(
                $"passes are drifting {update.SmoothedMeters * 100.0:F1} cm and the deviation keeps its sign in both directions — suspected side-slip or wind, not a calibration error")
            : FormattableString.Invariant(
                $"pass {_passDrift.PassCount}: seam {mean * 100.0:F1} cm, smoothed {update.SmoothedMeters * 100.0:F1} cm");
    }

    private void ResetDriftPass()
    {
        _driftCurrentPass = null;
        _driftPassSum = 0.0;
        _driftPassSamples = 0;
        _driftProbeCounter = 0;
    }

    /// <summary>Publish the indicator (and, before the first pass, the reason there is none).</summary>
    private void PublishDrift(string message)
    {
        var cal = State.Calibration;
        cal.Algorithm = _calibrationAlgorithm;
        cal.IsRecording = false; // the watcher is an observer, not a recording
        cal.DriftArmed = _driftArmed;
        cal.DriftStatus = (int)_passDrift.Status;
        cal.DriftSmoothedMeters = _passDrift.SmoothedMeters;
        cal.DriftPasses = _passDrift.PassCount;
        if (message.Length > 0) cal.DriftMessage = message;
    }
}