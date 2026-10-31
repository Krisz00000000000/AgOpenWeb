// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// One observation for the antenna lateral-offset / height fit (algorithm 1).
///
/// Model (line-relative, AgOpenWeb sign conventions):
/// <code>deviation = s · (a + h · sin(roll)) + c</code>
/// with <c>s</c> = <see cref="DirectionSign"/> and the unknowns
/// <c>a</c> (antenna lateral offset), <c>h</c> (antenna height above the roll axis)
/// and <c>c</c> (where the pass actually sits relative to the line — removed by
/// recording the same wheel track in both directions).
///
/// Sign conventions — the same the rest of AgOpenWeb uses, verified against
/// <c>AntennaToPivotTransform</c> and <c>VehicleConfig</c>:
/// <list type="bullet">
/// <item><see cref="DirectionSign"/>: +1 driving the track's recorded A→B order,
/// −1 driving it B→A (<c>IsHeadingSameWay</c> in the pipeline).</item>
/// <item><see cref="RollDegrees"/>: IMU roll in degrees, <b>positive = the vehicle
/// tilts to its own right</b> (right side down).</item>
/// <item><see cref="LateralDeviationMeters"/>: the <b>raw</b> (not antenna-corrected)
/// antenna position's signed perpendicular distance from the guidance line,
/// <b>positive = right of the A→B direction</b> — i.e. line-relative, not
/// vehicle-relative.</item>
/// </list>
///
/// Note on the last point: <c>TrackGuidanceOutput.CrossTrackError</c> flips sign when
/// driving against the track (<c>TrackGuidanceService</c> negates
/// <c>DistanceFromLinePivot</c> for <c>!IsHeadingSameWay</c>), so it is
/// <i>vehicle</i>-relative. Multiply it by <see cref="DirectionSign"/> before feeding
/// it here, or the <c>c</c> column would flip sign with the driving direction.
/// </summary>
/// <param name="DirectionSign">+1 = A→B, −1 = B→A.</param>
/// <param name="RollDegrees">Roll in degrees, positive = vehicle right side down.</param>
/// <param name="LateralDeviationMeters">Raw antenna deviation from the line, positive = right of A→B.</param>
public readonly record struct AntennaCalibrationSample(
    double DirectionSign,
    double RollDegrees,
    double LateralDeviationMeters);
