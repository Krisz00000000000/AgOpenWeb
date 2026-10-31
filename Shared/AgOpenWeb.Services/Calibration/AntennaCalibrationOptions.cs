// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// Thresholds for the antenna lateral-offset / height calibration (algorithm 1).
/// Used twice: <see cref="AntennaCalibrationCollector"/> gates each sample with
/// them while recording, and <see cref="AntennaOffsetHeightCalibrator"/> uses
/// them to decide whether a fit may be accepted.
/// </summary>
public sealed class AntennaCalibrationOptions
{
    /// <summary>Shared defaults — one instance, never mutated.</summary>
    public static AntennaCalibrationOptions Default { get; } = new();

    /// <summary>
    /// Usable samples required in <b>each</b> driving direction (A→B and B→A).
    /// One direction alone cannot separate the antenna offset, the antenna
    /// height and the line position (see the calibrator's model notes).
    /// </summary>
    public int MinSamplesPerDirection { get; init; } = 100;

    /// <summary>
    /// Minimum spread of the recorded roll (degrees, peak-to-peak). Below this the
    /// height term <c>h·sin(roll)</c> is nearly constant and cannot be separated
    /// from the lateral offset.
    /// </summary>
    public double MinRollRangeDegrees { get; init; } = 3.0;

    /// <summary>
    /// Smallest pivot of the column-normalized normal equations. Below this the
    /// three unknowns are not distinguishable to the required precision.
    /// Matches the AgOpenGPS-derived plan's 0.01 gate.
    /// </summary>
    public double MinPivot { get; init; } = 0.01;

    /// <summary>
    /// Reject samples past this count ("buffer full") so an operator who forgets to
    /// stop a recording cannot grow the buffer without bound.
    /// </summary>
    public int MaxSamples { get; init; } = 20000;

    /// <summary>Only RTK Fixed fixes are used (GGA fix quality 4).</summary>
    public int RtkFixedFixQuality { get; init; } = 4;

    /// <summary>Slow passes only — the model has no side-slip term.</summary>
    public double MinSpeedMps { get; init; } = 0.5;

    /// <summary>
    /// Reject samples farther from the line than this. The measurement assumes the
    /// vehicle follows the line (autosteer on), so a large deviation is a different
    /// error, not the antenna geometry.
    /// </summary>
    public double MaxAbsDeviationMeters { get; init; } = 1.0;
}
