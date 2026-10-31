// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

namespace AgOpenWeb.Models.State;

/// <summary>
/// Live mirror of the automatic calibration panel (the web UI's Calibration menu).
/// Written by <c>MainViewModel</c> on the UI thread from each GPS cycle and, on demand,
/// from the recorded samples; read by <c>SceneProjector</c> for the Status frame.
/// The computation itself lives in <c>AgOpenWeb.Services.Calibration</c> — this is only
/// the carrier so the panel and the host agree on what is being recorded.
/// </summary>
public class CalibrationState
{
    /// <summary>
    /// Which algorithm is recording, using the automatic calibration plan's numbering:
    /// 1 antenna offset + height, 2 WAS zero + gain, 3 steering response, 4 cross-track
    /// integrator, 5 pass-drift watcher, 6 sensor fusion. 0 = none selected.
    /// </summary>
    public int Algorithm { get; set; }

    /// <summary>True between Start and Stop.</summary>
    public bool IsRecording { get; set; }

    /// <summary>1-based wizard step the operator is on (1 field, 2 line, 3 A→B, 4 B→A,
    /// 5 roll spread, 6 result). Computed by CalibrationWizard from the live state.</summary>
    public int Step { get; set; }

    /// <summary>Stable key of that step ("field", "track", "forward", "reverse", "roll",
    /// "result") — the client owns the wording and its translation.</summary>
    public string StepKey { get; set; } = "field";

    /// <summary>Metres driven since wizard step 2 started — the line is built from the first
    /// <c>AutoLineDistanceMetres</c> of it, so the operator only has to drive straight.</summary>
    public double LineDistanceMeters { get; set; }

    /// <summary>Usable samples recorded driving the track's recorded A→B order.</summary>
    public int ForwardSamples { get; set; }

    /// <summary>Usable samples recorded driving B→A.</summary>
    public int ReverseSamples { get; set; }

    /// <summary>Peak-to-peak roll covered by the samples, degrees.</summary>
    public double RollRangeDegrees { get; set; }

    /// <summary>The unknowns could be separated (the fit's own pivot gate passed).</summary>
    public bool IsDeterminate { get; set; }

    /// <summary>Determinate and every sample/roll gate met — safe to apply.</summary>
    public bool IsAccepted { get; set; }

    /// <summary>Fitted antenna lateral offset, metres, positive = right of the centreline
    /// (what <c>Vehicle.AntennaOffset</c> expects).</summary>
    public double LateralOffsetMeters { get; set; }

    /// <summary>Fitted antenna height, metres (what <c>Vehicle.AntennaHeight</c> expects).</summary>
    public double HeightMeters { get; set; }

    /// <summary>Standard error of the offset (metres; NaN when it cannot be estimated).</summary>
    public double LateralOffsetStdDevMeters { get; set; }

    /// <summary>Standard error of the height (metres).</summary>
    public double HeightStdDevMeters { get; set; }

    /// <summary>One-line result or the reason the fit cannot be accepted yet.</summary>
    public string Message { get; set; } = "";

    /// <summary>Why the most recent sample was dropped ("" when it was kept) — so the
    /// panel can say "no RTK", "too slow", "too far off the line" instead of nothing
    /// happening.</summary>
    public string LastRejectReason { get; set; } = "";

    // ── Algorithm 2: WAS zero and gain ──────────────────────────────────

    /// <summary>Usable samples recorded by the WAS calibration.</summary>
    public int WasSamples { get; set; }

    /// <summary>Steer-angle range the samples covered, degrees (the gain gate).</summary>
    public double WasSteerRangeDegrees { get; set; }

    /// <summary>The fit produced usable unknowns.</summary>
    public bool WasDeterminate { get; set; }

    /// <summary>Every gate passed, so the zero may be applied.</summary>
    public bool WasAccepted { get; set; }

    /// <summary>The steering moved far enough for the gain to be observable.</summary>
    public bool WasGainObservable { get; set; }

    /// <summary>The sensor reading that means "wheels straight", degrees.</summary>
    public double WasZeroDegrees { get; set; }

    /// <summary>Standard error of <see cref="WasZeroDegrees"/> (NaN when unknown).</summary>
    public double WasZeroStdDevDegrees { get; set; }

    /// <summary>Fitted gain (inverse sensor scale, ≈ 1 when right).</summary>
    public double WasGain { get; set; }

    /// <summary>Standard error of <see cref="WasGain"/> (NaN when not observable).</summary>
    public double WasGainStdDev { get; set; }

    /// <summary>One-line WAS result or the reason it cannot be accepted yet.</summary>
    public string WasMessage { get; set; } = "";

    /// <summary>Why the most recent WAS sample was dropped ("" when kept).</summary>
    public string WasRejectReason { get; set; } = "";

    /// <summary>The <c>AutoSteer.WasOffset</c> (raw counts) the WAS proposal would write —
    /// only meaningful when <see cref="WasApplyValid"/>.</summary>
    public int WasProposedOffset { get; set; }

    /// <summary>The <c>AutoSteer.CountsPerDegree</c> the WAS proposal would write (the fitted
    /// gain folded in when it was observable, otherwise the current value).</summary>
    public double WasProposedCountsPerDegree { get; set; }

    /// <summary>How far the WAS offset would move, counts (signed) — the panel's readback.</summary>
    public int WasProposedOffsetDelta { get; set; }

    /// <summary>The proposal may actually be applied (fits and passes
    /// <see cref="AgOpenWeb.Services.Calibration.WasZeroGainApplication"/>'s guards).</summary>
    public bool WasApplyValid { get; set; }

    /// <summary>One line describing what Apply would write, or why it may not.</summary>
    public string WasApplyMessage { get; set; } = "";

    /// <summary>The fitted gain went into <see cref="WasProposedCountsPerDegree"/> rather than
    /// being left alone.</summary>
    public bool WasGainApplied { get; set; }

    // ── Algorithm 3: steering response (the only one that moves the steering) ──

    /// <summary>0 idle, 1 running, 2 finished with a result.</summary>
    public int SteerTestStage { get; set; }

    /// <summary>How far the running test has got, 0–1.</summary>
    public double SteerTestProgress { get; set; }

    /// <summary>The response fit produced usable unknowns.</summary>
    public bool SteerDeterminate { get; set; }

    /// <summary>The first-order model describes the response and no gate failed.</summary>
    public bool SteerAccepted { get; set; }

    /// <summary>Dead time θ, seconds.</summary>
    public double SteerDeadTimeSeconds { get; set; }

    /// <summary>Time constant τ, seconds.</summary>
    public double SteerTimeConstantSeconds { get; set; }

    /// <summary>Steady-state gain K.</summary>
    public double SteerGain { get; set; }

    /// <summary>The servo looks rate limited — repeat with a smaller step.</summary>
    public bool SteerRateLimited { get; set; }

    /// <summary>Steering-box play, degrees (NaN until measured).</summary>
    public double SteerBacklashDegrees { get; set; }

    /// <summary>One-line steering result or the reason there is none yet.</summary>
    public string SteerMessage { get; set; } = "";

    // ── Algorithm 4: cross-track integrator ─────────────────────────────

    /// <summary>The operator switched the integrator on for this session.</summary>
    public bool XteEnabled { get; set; }

    /// <summary>The learned correction in metres (the panel shows it in cm).</summary>
    public double XteCorrectionMeters { get; set; }

    /// <summary>The steering was saturated on the last cycle (integration held).</summary>
    public bool XteSaturated { get; set; }

    /// <summary>One line: what the integrator is doing, or why it is holding this cycle.</summary>
    public string XteMessage { get; set; } = "";

    // ── Algorithm 5: pass-drift watcher ─────────────────────────────────

    /// <summary>The operator armed the watcher for this session (the watcher corrects
    /// nothing; this only decides whether passes are recorded).</summary>
    public bool DriftArmed { get; set; }

    /// <summary>The traffic light as <c>PassDriftStatus</c>: 0 idle, 1 ok, 2 watch, 3 alarm.</summary>
    public int DriftStatus { get; set; }

    /// <summary>The smoothed per-pass deviation, metres.</summary>
    public double DriftSmoothedMeters { get; set; }

    /// <summary>Passes recorded since the watcher was armed.</summary>
    public int DriftPasses { get; set; }

    /// <summary>The last pass pair had the same deviation sign in both directions —
    /// suspected side-slip rather than a calibration error.</summary>
    public bool DriftSuspect { get; set; }

    /// <summary>One-line drift status or the reason a pass was skipped.</summary>
    public string DriftMessage { get; set; } = "";
}