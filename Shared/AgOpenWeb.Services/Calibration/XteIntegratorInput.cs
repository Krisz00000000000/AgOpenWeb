// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// One control cycle's inputs for the cross-track integrator. The signs are whatever the
/// guidance already uses — <see cref="CrossTrackErrorMeters"/> is exactly the value handed
/// to the steering law, so this class never reinterprets it.
/// </summary>
/// <param name="CrossTrackErrorMeters">The measured cross-track error, metres.</param>
/// <param name="DeltaTimeSeconds">Time since the previous update, seconds.</param>
/// <param name="SpeedMps">Ground speed, m/s.</param>
/// <param name="HasRtkFixed">RTK Fixed — the error is not trusted without it.</param>
/// <param name="IsAutoSteerEngaged">Autosteer is engaged.</param>
/// <param name="IsSteeringSaturated">The desired steer angle hit its limit this cycle.</param>
public readonly record struct XteIntegratorInput(
    double CrossTrackErrorMeters,
    double DeltaTimeSeconds,
    double SpeedMps,
    bool HasRtkFixed,
    bool IsAutoSteerEngaged,
    bool IsSteeringSaturated);
