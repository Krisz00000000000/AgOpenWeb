// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// Thresholds for the steering-response identification (algorithm 3): the step-response
/// fit, the servo rate-limit detector and the backlash estimate. The values are the plan's.
/// </summary>
public sealed class SteerResponseOptions
{
    /// <summary>Shared defaults — one instance, never mutated.</summary>
    public static SteerResponseOptions Default { get; } = new();

    /// <summary>Samples required <b>before</b> the step, to measure the sensor noise and the
    /// resting angle (the plan's 10).</summary>
    public int MinPreStepSamples { get; init; } = 10;

    /// <summary>Samples required <b>after</b> the step (the plan's 20).</summary>
    public int MinPostStepSamples { get; init; } = 20;

    /// <summary>Smallest commanded step worth identifying (the plan's 1°).</summary>
    public double MinStepDegrees { get; init; } = 1.0;

    /// <summary>Coarse dead-time grid: 0 … <see cref="MaxDeadTimeSeconds"/> in
    /// <see cref="DeadTimeGridSteps"/> steps (the plan's 0–0.8 s, 41 steps).</summary>
    public double MaxDeadTimeSeconds { get; init; } = 0.8;

    /// <summary>Number of coarse dead-time grid points.</summary>
    public int DeadTimeGridSteps { get; init; } = 41;

    /// <summary>Smallest time constant in the search (the plan's 0.03 s).</summary>
    public double MinTimeConstantSeconds { get; init; } = 0.03;

    /// <summary>Largest time constant in the search (the plan's 3 s).</summary>
    public double MaxTimeConstantSeconds { get; init; } = 3.0;

    /// <summary>Logarithmic time-constant grid points (the plan's 40).</summary>
    public int TimeConstantGridSteps { get; init; } = 40;

    /// <summary>
    /// How many times the grid is narrowed around the best point and re-run, which is what
    /// turns the coarse 20 ms dead-time grid into a useful estimate.
    /// </summary>
    public int RefinementPasses { get; init; } = 2;

    /// <summary>
    /// The plan's rate-limit detector: the measured peak rate divided by the fitted model's
    /// initial rate <c>K·Δ/τ</c>. Below this the servo is judged to be rate limited.
    /// </summary>
    public double RateLimitRatio { get; init; } = 0.7;

    /// <summary>
    /// Shape check that goes with it: <c>(t95 − t10)/(t63 − t10)</c> of the measured
    /// response. A true first-order response gives ≈ 3.25; a rate-limited one ramps, so it
    /// gives ≈ 1.6. Below this the response is not exponential enough for the model to be
    /// trusted. Added because the ratio alone drifts towards 1 as the fit adapts its
    /// <c>τ</c> to a ramp — see the calibration plan's note that the threshold has to be
    /// tuned in simulation and on a real servo.
    /// </summary>
    public double MinShapeRatio { get; init; } = 2.0;

    /// <summary>
    /// Floor for the pre-step noise estimate (degrees). Noise-free simulation would divide
    /// by zero when the model residual is compared with it.
    /// </summary>
    public double SensorNoiseFloorDegrees { get; init; } = 0.01;

    /// <summary>
    /// A first-order model whose RMS residual is more than this multiple of the measured
    /// sensor noise does not describe the response, so the fit is not accepted.
    /// </summary>
    public double MaxResidualToNoiseRatio { get; init; } = 3.0;

    /// <summary>Settled points required per approach direction for the backlash estimate
    /// (the plan's 4 + 4).</summary>
    public int MinPointsPerApproach { get; init; } = 4;

    /// <summary>Smallest pivot of the column-normalized normal equations for the backlash fit.</summary>
    public double MinPivot { get; init; } = 0.01;
}
