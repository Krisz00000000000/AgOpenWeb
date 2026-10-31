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
}