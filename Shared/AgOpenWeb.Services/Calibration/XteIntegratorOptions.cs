// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// Settings for the cross-track integrator (algorithm 4). The defaults are the plan's, and
/// the integrator is <b>off by default</b>: switching it on is a deliberate act, because it
/// also hides a wrong antenna offset or roll correction (see the plan's warning — algorithm
/// 1 must be measured with the integrator off).
/// </summary>
public sealed class XteIntegratorOptions
{
    /// <summary>Shared defaults — one instance, never mutated.</summary>
    public static XteIntegratorOptions Default { get; } = new();

    /// <summary>Off unless the operator turns it on. While off,
    /// <see cref="XteIntegrator.Update"/> is a no-op that returns 0.</summary>
    public bool Enabled { get; init; }

    /// <summary>Integral gain <c>k_i</c> in 1/s. <c>1/k_i</c> is the integration time
    /// constant and must be at least an order of magnitude slower than the steering's own
    /// delay, or the loop starts to oscillate.</summary>
    public double IntegralGainPerSecond { get; init; } = 0.05;

    /// <summary>Hard limit of the learned correction <c>c_max</c>, metres.</summary>
    public double MaxCorrectionMeters { get; init; } = 0.30;

    /// <summary>Optional leak <c>λ</c> in 1/s — lets the correction forget slowly. 0 = no leak.</summary>
    public double LeakPerSecond { get; init; }

    /// <summary>Do not integrate while the cross-track error is past this (metres): that is
    /// not a steady-state bias any more, it is a different problem.</summary>
    public double MaxAbsErrorMeters { get; init; } = 0.5;

    /// <summary>Do not integrate below this speed (m/s) — the response needs to be moving.</summary>
    public double MinSpeedMps { get; init; } = 0.5;

    /// <summary>
    /// True (the plan's rule): stop integrating while the steering is saturated, unless the
    /// error opposes the correction — in that case keep going, because that is what unwinds
    /// the windup. False: integrate regardless and only clamp (the naive integrator the
    /// plan's simulation compares against).
    /// </summary>
    public bool ConditionalAntiWindup { get; init; } = true;
}
