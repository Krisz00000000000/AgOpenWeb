// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// Thresholds for the WAS zero / gain calibration (algorithm 2), used twice:
/// <see cref="WasCalibrationCollector"/> drops each sample that fails them while
/// recording, and <see cref="WasZeroGainCalibrator"/> decides whether a fit may be
/// accepted. The filter values are the plan's.
/// </summary>
public sealed class WasCalibrationOptions
{
    /// <summary>Shared defaults — one instance, never mutated.</summary>
    public static WasCalibrationOptions Default { get; } = new();

    /// <summary>The yaw-rate model is only valid while actually moving.</summary>
    public double MinSpeedMps { get; init; } = 1.0;

    /// <summary>
    /// Reject samples taken on a tilt. The model is a flat-ground Ackermann relationship:
    /// on a slope (or slippery ground) the yaw rate no longer matches the geometry.
    /// </summary>
    public double MaxAbsRollDegrees { get; init; } = 3.0;

    /// <summary>Reject samples while turning fast enough for side-slip to matter.</summary>
    public double MaxAbsYawRateRadPerSecond { get; init; } = 0.5;

    /// <summary>Reject samples at or past the steering limit — the geometry is not valid there.</summary>
    public double MaxAbsSteerAngleDegrees { get; init; } = 35.0;

    /// <summary>Usable samples required before a fit may be accepted (the plan's 300).</summary>
    public int MinSamples { get; init; } = 300;

    /// <summary>
    /// Steer-angle range (degrees, peak-to-peak) the recording must cover for the
    /// <b>gain</b> to be observable. Below this the fit reports the zero alone, with the
    /// gain fixed at 1, and says so.
    /// </summary>
    public double MinSteerRangeDegrees { get; init; } = 8.0;

    /// <summary>
    /// Smallest pivot of the column-normalized normal equations. Below this the two
    /// unknowns are not distinguishable to the required precision.
    /// </summary>
    public double MinPivot { get; init; } = 0.01;

    /// <summary>Reject samples past this count so a forgotten recording cannot grow without bound.</summary>
    public int MaxSamples { get; init; } = 20000;
}
