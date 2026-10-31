// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// One observation for the WAS (wheel angle sensor) zero / gain fit (algorithm 2).
///
/// Model. For a first-order (Ackermann) steered, non-slipping vehicle the <i>true</i>
/// steer angle follows from the wheelbase, the yaw rate and the speed:
/// <code>tan(δ_true) = L·ω / v             →  δ_true = atan(L·ω / v)</code>
/// The sensor reports <c>x</c> for that angle, and the two are assumed linearly related:
/// <code>δ_true = g·x + b</code>
/// with the unknowns <c>g</c> (the <b>gain</b>, the inverse sensor scale, ≈ 1 when the
/// sensor's scale is right) and <c>b</c> (the intercept). The sensor's zero is
/// <c>−b/g</c>: the reading that corresponds to driving straight.
///
/// Sign conventions (the module convention the plan lists, verified against AgOpenWeb's
/// own steer path):
/// <list type="bullet">
/// <item><see cref="MeasuredSteerAngleDegrees"/>: <b>positive = steered left</b>, in degrees,
/// with the sign of the steer module's feedback (PGN 253 → <c>LastSteerData.ActualSteerAngle</c>).</item>
/// <item><see cref="YawRateRadiansPerSecond"/>: the IMU gyro's yaw rate, <b>positive =
/// turning left</b> (counter-clockwise in the ENU plane, the same sense heading grows).
/// Because both quantities share that sense, a good sensor gives <c>g ≈ +1</c>; the fit does
/// not assume the sign, it measures it.</item>
/// <item><see cref="RollDegrees"/>: IMU roll in degrees. Only a filter here (the model does
/// not hold on a tilt), so its sign does not enter the fit — see
/// <see cref="AntennaCalibrationSample"/> for the convention.</item>
/// </list>
/// </summary>
/// <param name="MeasuredSteerAngleDegrees">Steer angle the module reports, positive = left.</param>
/// <param name="YawRateRadiansPerSecond">Yaw rate from the IMU gyro, positive = turning left.</param>
/// <param name="SpeedMetersPerSecond">Ground speed, m/s.</param>
/// <param name="RollDegrees">IMU roll, degrees (filter only).</param>
public readonly record struct WasCalibrationSample(
    double MeasuredSteerAngleDegrees,
    double YawRateRadiansPerSecond,
    double SpeedMetersPerSecond,
    double RollDegrees);
