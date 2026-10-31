// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// The anti-windup cross-track integrator of algorithm 4: on a slope, in a side wind or
/// behind an implement that pulls sideways the vehicle drifts, and a purely proportional law
/// (Stanley, Pure Pursuit) keeps a steady error. This class learns a slowly-varying
/// correction <c>c</c> that is added to the error the steering law sees:
/// <code>corrected = e + c,   c ← clamp(c·(1 − λ·Δt) + k_i·e·Δt, ±c_max)</code>
///
/// Anti-windup (the plan's rule): while the steering is saturated the integration stops —
/// unless the error opposes the correction, which is exactly the case that unwinds it.
/// Beyond that it refuses to integrate when the error is too large to be a steady-state
/// bias, without RTK Fixed, with autosteer off, or when standing still.
///
/// Stateful but dependency-free: no DI, no I/O, and it never writes a setting. The
/// correction is deliberately not persisted across restarts (the plan) and
/// <see cref="Reset"/> starts a new line, a U-turn, or a fresh engage.
///
/// Not wired into <c>TrackGuidanceService</c> yet: the plan wants the sign convention
/// checked against the existing guidance code and the HotPath budget measured first.
/// </summary>
public sealed class XteIntegrator
{
    private readonly object _lock = new();
    private readonly XteIntegratorOptions _options;
    private double _correction;

    public XteIntegrator(XteIntegratorOptions? options = null)
    {
        _options = options ?? XteIntegratorOptions.Default;
    }

    /// <summary>The settings in force for this integrator.</summary>
    public XteIntegratorOptions Options => _options;

    /// <summary>The learned correction <c>c</c> in metres (0 while disabled).</summary>
    public double Correction
    {
        get { lock (_lock) return _correction; }
    }

    /// <summary>Zero the correction — call on a new line, on the U-turn state machine's
    /// transitions and on every autosteer engage/disengage.</summary>
    public void Reset()
    {
        lock (_lock) _correction = 0.0;
    }

    /// <summary>The error the steering law should be given this cycle: <c>e + c</c>.</summary>
    public double CorrectedError(double crossTrackErrorMeters)
    {
        lock (_lock) return crossTrackErrorMeters + _correction;
    }

    /// <summary>
    /// Advance one cycle and return the correction to add to the cross-track error. While
    /// disabled, or while any gate holds, the correction is left untouched — the returned
    /// value is exactly what was there before, so the disabled path changes nothing.
    /// </summary>
    public double Update(XteIntegratorInput input)
    {
        lock (_lock)
        {
            if (!_options.Enabled) return _correction;

            double error = input.CrossTrackErrorMeters;
            if (!double.IsFinite(error) || !(input.DeltaTimeSeconds > 0.0)) return _correction;
            if (!input.HasRtkFixed || !input.IsAutoSteerEngaged) return _correction;
            if (input.SpeedMps < _options.MinSpeedMps) return _correction;
            if (Math.Abs(error) > _options.MaxAbsErrorMeters) return _correction;

            // Saturated: only keep integrating when the error opposes the correction, since
            // that is the direction that reduces the windup.
            bool unwinding = (_correction * error) < 0.0;
            if (_options.ConditionalAntiWindup && input.IsSteeringSaturated && !unwinding)
            {
                return _correction;
            }

            double decay = Math.Max(0.0, 1.0 - (_options.LeakPerSecond * input.DeltaTimeSeconds));
            double step = _options.IntegralGainPerSecond * error * input.DeltaTimeSeconds;

            _correction = Math.Clamp(
                (_correction * decay) + step,
                -_options.MaxCorrectionMeters,
                _options.MaxCorrectionMeters);

            return _correction;
        }
    }
}
