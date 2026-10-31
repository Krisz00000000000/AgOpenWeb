// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;
using System.Collections.Generic;

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// The identified steering response: a first-order lag with dead time, plus the measured
/// evidence about whether that model is actually valid.
/// Produced by <see cref="SteerResponseIdentifier.FitStepResponse"/>.
/// </summary>
public readonly struct SteerStepResponseFit
{
    /// <summary>Resting angle before the step, degrees.</summary>
    public double BaselineDegrees { get; init; }

    /// <summary>Commanded step size Δ, degrees (after − before).</summary>
    public double StepAmplitudeDegrees { get; init; }

    /// <summary>When the command stepped, seconds from the test's start.</summary>
    public double StepTimeSeconds { get; init; }

    /// <summary>Dead time θ, seconds — the delay before the wheels start moving.</summary>
    public double DeadTimeSeconds { get; init; }

    /// <summary>Time constant τ, seconds.</summary>
    public double TimeConstantSeconds { get; init; }

    /// <summary>Steady-state gain K (degrees per degree of command).</summary>
    public double SteadyStateGain { get; init; }

    /// <summary>RMS of the fit residuals, degrees.</summary>
    public double FitRmsDegrees { get; init; }

    /// <summary>RMS of the pre-step samples about their mean — the sensor noise, degrees.</summary>
    public double SensorNoiseDegrees { get; init; }

    /// <summary>Measured peak steering rate, degrees per second.</summary>
    public double PeakRateDegreesPerSecond { get; init; }

    /// <summary>The fitted model's initial rate <c>K·Δ/τ</c>, degrees per second.</summary>
    public double ModelInitialRateDegreesPerSecond { get; init; }

    /// <summary><see cref="PeakRateDegreesPerSecond"/> / <see cref="ModelInitialRateDegreesPerSecond"/>
    /// — the plan's rate-limit ratio (below <c>RateLimitRatio</c> = suspect).</summary>
    public double RateRatio { get; init; }

    /// <summary>The measured shape ratio <c>(t95 − t10)/(t63 − t10)</c>; a true first-order
    /// response gives ≈ 3.25, a rate-limited ramp ≈ 1.6.</summary>
    public double ShapeRatio { get; init; }

    /// <summary>True when either check says the servo is moving at a limited rate, so the
    /// first-order parameters must not be used as they stand (repeat with a smaller step).</summary>
    public bool IsRateLimited { get; init; }

    /// <summary>Total samples used.</summary>
    public int SampleCount { get; init; }

    /// <summary>Samples before the step.</summary>
    public int PreStepSampleCount { get; init; }

    /// <summary>Samples after the step.</summary>
    public int PostStepSampleCount { get; init; }
}

/// <summary>
/// Outcome of a steering step-response fit: whether the test data is complete enough to
/// identify anything (<see cref="IsDeterminate"/>), whether the identified parameters may be
/// used (<see cref="IsAccepted"/>) and, when they exist, the values themselves.
/// </summary>
public sealed class SteerStepResponseReport
{
    /// <summary>True when a step was found and the first-order fit could be solved.</summary>
    public bool IsDeterminate { get; init; }

    /// <summary>True when the fit is determinate, every gate passed and the first-order model
    /// actually describes the response (not rate limited, residuals at the noise level).</summary>
    public bool IsAccepted { get; init; }

    /// <summary>The identified parameters; <c>null</c> when <see cref="IsDeterminate"/> is false.</summary>
    public SteerStepResponseFit? Fit { get; init; }

    /// <summary>Human-readable reasons the fit cannot be accepted yet (empty when it can).</summary>
    public IReadOnlyList<string> Warnings { get; init; } = Array.Empty<string>();

    /// <summary>Information that does not block acceptance.</summary>
    public IReadOnlyList<string> Notes { get; init; } = Array.Empty<string>();

    /// <summary>One-line summary for the panel (invariant number formatting).</summary>
    public string Message { get; init; } = string.Empty;
}
