// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// One steering-step sample for algorithm 3: when it was taken, what was asked of the
/// steering and what the module reported back.
///
/// Both angles are in degrees with the module's sign convention (positive = steered left,
/// see <see cref="WasCalibrationSample"/>). The pair is what the test path writes: the
/// commanded angle goes out in PGN 254 and the measured one comes back in PGN 253
/// (<c>LastSteerData.ActualSteerAngle</c>), so the time base is the PGN 253 feedback rate.
/// </summary>
/// <param name="TimeSeconds">Seconds from the start of the test (monotonic).</param>
/// <param name="CommandedSteerAngleDegrees">The angle the test asked for.</param>
/// <param name="MeasuredSteerAngleDegrees">The angle the module reported.</param>
public readonly record struct SteerResponseSample(
    double TimeSeconds,
    double CommandedSteerAngleDegrees,
    double MeasuredSteerAngleDegrees);

/// <summary>
/// One settled point for the backlash estimate. The steering is brought to
/// <see cref="CommandedSteerAngleDegrees"/> from one side and left to settle, so the play in
/// the steering box shows up as an offset that depends on which way it was approached.
/// </summary>
/// <param name="CommandedSteerAngleDegrees">The angle the steering settled at.</param>
/// <param name="MeasuredSteerAngleDegrees">The angle the module reported for it.</param>
/// <param name="ApproachSign">+1 when the command was reached from below, −1 from above.</param>
public readonly record struct SteeringBacklashPoint(
    double CommandedSteerAngleDegrees,
    double MeasuredSteerAngleDegrees,
    double ApproachSign);
