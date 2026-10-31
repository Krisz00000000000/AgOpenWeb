// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;
using System.Collections.Generic;

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// Identifies how the steering responds and how much play the steering box has (algorithm 3).
///
/// Model. A first-order lag with dead time: a commanded step of Δ at <c>t_s</c> gives
/// <code>y(t) = y₀ + K·Δ·(1 − e^(−(t − t_s − θ)/τ))   for t &gt; t_s + θ</code>
/// with the dead time θ, the time constant τ and the steady-state gain K. θ is found on a
/// grid (the plan's 0–0.8 s, 41 steps) and τ on a logarithmic grid (0.03–3 s, 40 steps),
/// then both are refined on a narrowing grid; K is solved in closed form for every pair,
/// because the model is linear in K once θ and τ are fixed.
///
/// This is the one algorithm that moves the steering, so it only ever <i>reads</i> a test
/// path's samples: the caller runs the gated test (held button, ±10°, watchdog, no autosteer)
/// and hands the (time, commanded, measured) triples here. Nothing in this class sends a
/// command.
///
/// Pure computation: no DI, no I/O, no ambient state.
/// </summary>
public static class SteerResponseIdentifier
{
    /// <summary>Fit with <see cref="SteerResponseOptions.Default"/>.</summary>
    public static SteerStepResponseReport FitStepResponse(IReadOnlyList<SteerResponseSample> samples)
        => FitStepResponse(samples, null);

    /// <summary>
    /// Find the step in <paramref name="samples"/>, fit θ, τ and K to it, and report whether
    /// the identified parameters may be used.
    /// </summary>
    public static SteerStepResponseReport FitStepResponse(
        IReadOnlyList<SteerResponseSample> samples,
        SteerResponseOptions? options)
    {
        ArgumentNullException.ThrowIfNull(samples);
        options ??= SteerResponseOptions.Default;

        int minimum = options.MinPreStepSamples + options.MinPostStepSamples;
        if (samples.Count < minimum)
        {
            return NotDeterminate(FormattableString.Invariant(
                $"at least {minimum} samples are needed ({options.MinPreStepSamples} before the step and ")
                + FormattableString.Invariant($"{options.MinPostStepSamples} after), {samples.Count} recorded"));
        }

        for (int i = 1; i < samples.Count; i++)
        {
            if (!double.IsFinite(samples[i].TimeSeconds)
                || !double.IsFinite(samples[i].CommandedSteerAngleDegrees)
                || !double.IsFinite(samples[i].MeasuredSteerAngleDegrees))
            {
                return NotDeterminate(FormattableString.Invariant(
                    $"sample {i} carries a non-finite value"));
            }

            if (!(samples[i].TimeSeconds > samples[i - 1].TimeSeconds))
            {
                return NotDeterminate("the sample times must be strictly increasing");
            }
        }

        // The step is the largest jump in the commanded angle.
        int stepIndex = -1;
        double step = 0.0;
        double bestMagnitude = 0.0;
        for (int i = 1; i < samples.Count; i++)
        {
            double jump = samples[i].CommandedSteerAngleDegrees - samples[i - 1].CommandedSteerAngleDegrees;
            if (Math.Abs(jump) > bestMagnitude)
            {
                bestMagnitude = Math.Abs(jump);
                step = jump;
                stepIndex = i;
            }
        }

        if (stepIndex < 0)
        {
            return NotDeterminate("the commanded angle never stepped");
        }

        int preCount = stepIndex;
        int postCount = samples.Count - stepIndex;
        if (preCount < options.MinPreStepSamples)
        {
            return NotDeterminate(FormattableString.Invariant(
                $"only {preCount} sample(s) before the step, {options.MinPreStepSamples} needed"));
        }

        if (postCount < options.MinPostStepSamples)
        {
            return NotDeterminate(FormattableString.Invariant(
                $"only {postCount} sample(s) after the step, {options.MinPostStepSamples} needed"));
        }

        if (Math.Abs(step) < options.MinStepDegrees)
        {
            return NotDeterminate(FormattableString.Invariant(
                $"the step was only {step:F3}°, {options.MinStepDegrees:F1}° needed"));
        }

        // The command has to hold after the step, otherwise this is not a step response.
        double postCommand = 0.0;
        for (int i = stepIndex; i < samples.Count; i++) postCommand += samples[i].CommandedSteerAngleDegrees;
        postCommand /= postCount;

        double tolerance = Math.Max(0.5, 0.25 * Math.Abs(step));
        for (int i = stepIndex; i < samples.Count; i++)
        {
            if (Math.Abs(samples[i].CommandedSteerAngleDegrees - postCommand) > tolerance)
            {
                return NotDeterminate("the commanded angle did not hold after the step — "
                    + "this is not a step response");
            }
        }

        // Resting angle and sensor noise, from the samples before the step.
        double baseline = 0.0;
        for (int i = 0; i < stepIndex; i++) baseline += samples[i].MeasuredSteerAngleDegrees;
        baseline /= preCount;

        double noiseSquares = 0.0;
        for (int i = 0; i < stepIndex; i++)
        {
            double deviation = samples[i].MeasuredSteerAngleDegrees - baseline;
            noiseSquares += deviation * deviation;
        }

        double noise = Math.Sqrt(noiseSquares / preCount);
        double stepTime = samples[stepIndex].TimeSeconds;

        return FitCore(samples, stepIndex, stepTime, step, baseline, noise, options);
    }

    private static SteerStepResponseReport FitCore(
        IReadOnlyList<SteerResponseSample> samples,
        int stepIndex,
        double stepTime,
        double step,
        double baseline,
        double noise,
        SteerResponseOptions options)
    {
        double deadTimeLow = 0.0;
        double deadTimeHigh = options.MaxDeadTimeSeconds;
        double tauLow = options.MinTimeConstantSeconds;
        double tauHigh = options.MaxTimeConstantSeconds;

        double deadTime = 0.0;
        double tau = options.MinTimeConstantSeconds;
        double gain = 0.0;
        double sumSquares = double.MaxValue;

        for (int pass = 0; pass <= options.RefinementPasses; pass++)
        {
            SearchGrid(
                samples, stepIndex, stepTime, step, baseline,
                deadTimeLow, deadTimeHigh, tauLow, tauHigh, options,
                out deadTime, out tau, out gain, out sumSquares);

            // Narrow the grids to ± one step around the best point and go again: the coarse
            // dead-time grid is 20 ms, which is coarser than the plan's 30 ms acceptance.
            double deadStep = (deadTimeHigh - deadTimeLow) / (Math.Max(2, options.DeadTimeGridSteps) - 1.0);
            double tauStep = Math.Log(tauHigh / tauLow) / (Math.Max(2, options.TimeConstantGridSteps) - 1.0);

            deadTimeLow = Math.Max(0.0, deadTime - deadStep);
            deadTimeHigh = Math.Min(options.MaxDeadTimeSeconds, deadTime + deadStep);
            tauLow = Math.Max(options.MinTimeConstantSeconds, tau * Math.Exp(-tauStep));
            tauHigh = Math.Min(options.MaxTimeConstantSeconds, tau * Math.Exp(tauStep));
        }

        double fitRms = Math.Sqrt(sumSquares / samples.Count);
        double peakRate = PeakRateDegreesPerSecond(samples, stepIndex);
        double modelRate = Math.Abs(gain * step) / tau;
        double rateRatio = modelRate > 0.0 ? peakRate / modelRate : double.NaN;
        double shapeRatio = ShapeRatio(samples, stepIndex, baseline);

        bool rateLimited =
            (double.IsFinite(rateRatio) && rateRatio < options.RateLimitRatio)
            || (double.IsFinite(shapeRatio) && shapeRatio < options.MinShapeRatio);

        var warnings = new List<string>();
        var notes = new List<string>();

        if (rateLimited)
        {
            warnings.Add(FormattableString.Invariant(
                    $"the servo looks rate limited: peak {peakRate:F1}°/s against the model's initial ")
                + FormattableString.Invariant(
                    $"{modelRate:F1}°/s, shape ratio {shapeRatio:F2} against {options.MinShapeRatio:F1} ")
                + "— repeat the test with a smaller step");
        }

        double allowedRms = options.MaxResidualToNoiseRatio * Math.Max(noise, options.SensorNoiseFloorDegrees);
        if (fitRms > allowedRms)
        {
            warnings.Add(FormattableString.Invariant(
                $"the first-order model does not describe the response (RMS {fitRms:F3}° against a ")
                + FormattableString.Invariant($"noise floor of {allowedRms:F3}°)"));
        }

        if (deadTime >= options.MaxDeadTimeSeconds)
        {
            notes.Add(FormattableString.Invariant(
                $"the dead time sits at the search limit ({options.MaxDeadTimeSeconds:F1} s)"));
        }

        if (tau >= options.MaxTimeConstantSeconds || tau <= options.MinTimeConstantSeconds)
        {
            notes.Add(FormattableString.Invariant(
                $"the time constant sits at a search limit ({options.MinTimeConstantSeconds:F2}–{options.MaxTimeConstantSeconds:F1} s)"));
        }

        if (noise <= options.SensorNoiseFloorDegrees)
        {
            notes.Add("the samples before the step look noise-free, so the residual check is against the floor");
        }

        var fit = new SteerStepResponseFit
        {
            BaselineDegrees = baseline,
            StepAmplitudeDegrees = step,
            StepTimeSeconds = stepTime,
            DeadTimeSeconds = deadTime,
            TimeConstantSeconds = tau,
            SteadyStateGain = gain,
            FitRmsDegrees = fitRms,
            SensorNoiseDegrees = noise,
            PeakRateDegreesPerSecond = peakRate,
            ModelInitialRateDegreesPerSecond = modelRate,
            RateRatio = rateRatio,
            ShapeRatio = shapeRatio,
            IsRateLimited = rateLimited,
            SampleCount = samples.Count,
            PreStepSampleCount = stepIndex,
            PostStepSampleCount = samples.Count - stepIndex,
        };

        return new SteerStepResponseReport
        {
            IsDeterminate = true,
            IsAccepted = warnings.Count == 0,
            Fit = fit,
            Warnings = warnings,
            Notes = notes,
            Message = FormattableString.Invariant(
                    $"gain {gain:F3}, dead time {deadTime * 1000.0:F0} ms, time constant {tau * 1000.0:F0} ms")
                + FormattableString.Invariant($", RMS {fitRms:F3}°")
                + WarningSuffix(warnings),
        };
    }

    /// <summary>
    /// Grid search over (θ, τ); K is solved in closed form for every pair.
    /// </summary>
    private static void SearchGrid(
        IReadOnlyList<SteerResponseSample> samples,
        int stepIndex,
        double stepTime,
        double step,
        double baseline,
        double deadTimeLow,
        double deadTimeHigh,
        double tauLow,
        double tauHigh,
        SteerResponseOptions options,
        out double bestDeadTime,
        out double bestTau,
        out double bestGain,
        out double bestSumSquares)
    {
        int deadSteps = Math.Max(2, options.DeadTimeGridSteps);
        int tauSteps = Math.Max(2, options.TimeConstantGridSteps);

        bestDeadTime = deadTimeLow;
        bestTau = tauLow;
        bestGain = 0.0;
        bestSumSquares = double.MaxValue;

        for (int i = 0; i < deadSteps; i++)
        {
            double deadTime = deadTimeLow + ((deadTimeHigh - deadTimeLow) * i / (deadSteps - 1.0));
            for (int j = 0; j < tauSteps; j++)
            {
                double tau = tauLow * Math.Pow(tauHigh / tauLow, j / (tauSteps - 1.0));

                // K is closed form: minimise Σ(measured − baseline − K·Δ·F)², so
                // K = Σ(Δ·F·(measured − baseline)) / Σ((Δ·F)²).
                double numerator = 0.0;
                double denominator = 0.0;
                for (int k = stepIndex; k < samples.Count; k++)
                {
                    double shape = step * ResponseFraction(
                        samples[k].TimeSeconds - stepTime - deadTime, tau);
                    double target = samples[k].MeasuredSteerAngleDegrees - baseline;
                    numerator += shape * target;
                    denominator += shape * shape;
                }

                if (!(denominator > 0.0)) continue;
                double gain = numerator / denominator;

                double sumSquares = 0.0;
                for (int k = 0; k < samples.Count; k++)
                {
                    double predicted = baseline
                        + (gain * step * ResponseFraction(
                            samples[k].TimeSeconds - stepTime - deadTime, tau));
                    double residual = samples[k].MeasuredSteerAngleDegrees - predicted;
                    sumSquares += residual * residual;
                }

                if (sumSquares < bestSumSquares)
                {
                    bestSumSquares = sumSquares;
                    bestDeadTime = deadTime;
                    bestTau = tau;
                    bestGain = gain;
                }
            }
        }
    }

    /// <summary>The first-order lag's unit step response: 0 before the dead time, then
    /// <c>1 − e^(−elapsed/τ)</c>.</summary>
    private static double ResponseFraction(double elapsedSeconds, double tau)
        => elapsedSeconds <= 0.0 ? 0.0 : 1.0 - Math.Exp(-elapsedSeconds / tau);

    /// <summary>Largest measured steering rate after the step, degrees per second.</summary>
    private static double PeakRateDegreesPerSecond(
        IReadOnlyList<SteerResponseSample> samples, int stepIndex)
    {
        double peak = 0.0;
        for (int i = Math.Max(stepIndex, 1); i < samples.Count; i++)
        {
            double dt = samples[i].TimeSeconds - samples[i - 1].TimeSeconds;
            if (!(dt > 0.0)) continue;

            double rate = Math.Abs(
                samples[i].MeasuredSteerAngleDegrees - samples[i - 1].MeasuredSteerAngleDegrees) / dt;
            if (rate > peak) peak = rate;
        }

        return peak;
    }

    /// <summary>
    /// <c>(t95 − t10)/(t63 − t10)</c> of the measured response, where each time is when the
    /// response first crossed that fraction of its final value. Taking t10 as the base makes
    /// the number independent of the dead time, which would otherwise compress it: a
    /// first-order lag gives ≈ 3.25 and a rate-limited ramp ≈ 1.6.
    /// <c>NaN</c> when the response never reached the targets within the recording.
    /// </summary>
    private static double ShapeRatio(
        IReadOnlyList<SteerResponseSample> samples, int stepIndex, double baseline)
    {
        int postCount = samples.Count - stepIndex;
        int tail = Math.Max(1, postCount / 4);

        double final = 0.0;
        for (int i = samples.Count - tail; i < samples.Count; i++)
        {
            final += samples[i].MeasuredSteerAngleDegrees;
        }

        final /= tail;

        double delta = final - baseline;
        if (Math.Abs(delta) < 1e-9) return double.NaN;

        double? t10 = CrossingTimeSeconds(samples, stepIndex, baseline + (0.10 * delta));
        double? t63 = CrossingTimeSeconds(samples, stepIndex, baseline + (0.63 * delta));
        double? t95 = CrossingTimeSeconds(samples, stepIndex, baseline + (0.95 * delta));
        if (t10 is null || t63 is null || t95 is null) return double.NaN;

        double span = t63.Value - t10.Value;
        return span > 0.0 ? (t95.Value - t10.Value) / span : double.NaN;
    }

    /// <summary>First time the measured angle reaches <paramref name="target"/> after the step,
    /// linearly interpolated between samples.</summary>
    private static double? CrossingTimeSeconds(
        IReadOnlyList<SteerResponseSample> samples, int stepIndex, double target)
    {
        for (int i = stepIndex + 1; i < samples.Count; i++)
        {
            double previous = samples[i - 1].MeasuredSteerAngleDegrees;
            double current = samples[i].MeasuredSteerAngleDegrees;
            if (current == previous) continue;

            double low = Math.Min(previous, current);
            double high = Math.Max(previous, current);
            if (target < low || target > high) continue;

            double fraction = (target - previous) / (current - previous);
            return samples[i - 1].TimeSeconds
                + (fraction * (samples[i].TimeSeconds - samples[i - 1].TimeSeconds));
        }

        return null;
    }

    /// <summary>Estimate the steering-box play with <see cref="SteerResponseOptions.Default"/>.</summary>
    public static SteeringBacklashReport EstimateBacklash(IReadOnlyList<SteeringBacklashPoint> points)
        => EstimateBacklash(points, null);

    /// <summary>
    /// Estimate the play from settled points approached from both sides.
    /// Model: <c>measured = k·command + c + e·approach</c> with the approach +1 when the
    /// command was reached from below and −1 from above; the play width is <c>−2e</c>.
    /// Points from one direction alone cannot separate the play from the intercept.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">A non-finite value, or an
    /// <see cref="SteeringBacklashPoint.ApproachSign"/> other than ±1.</exception>
    public static SteeringBacklashReport EstimateBacklash(
        IReadOnlyList<SteeringBacklashPoint> points, SteerResponseOptions? options)
    {
        ArgumentNullException.ThrowIfNull(points);
        options ??= SteerResponseOptions.Default;

        var warnings = new List<string>();
        int fromBelow = 0;
        int fromAbove = 0;
        for (int i = 0; i < points.Count; i++)
        {
            var point = points[i];
            if (!double.IsFinite(point.CommandedSteerAngleDegrees)
                || !double.IsFinite(point.MeasuredSteerAngleDegrees))
            {
                throw new ArgumentOutOfRangeException(
                    nameof(points), FormattableString.Invariant($"point {i} is not finite"));
            }

            if (Math.Abs(Math.Abs(point.ApproachSign) - 1.0) > 1e-9)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(points), point.ApproachSign,
                    "The approach sign must be +1 (from below) or −1 (from above).");
            }

            if (point.ApproachSign > 0) fromBelow++;
            else fromAbove++;
        }

        if (fromBelow < options.MinPointsPerApproach)
        {
            warnings.Add(FormattableString.Invariant(
                $"only {fromBelow} settled point(s) reached from below, {options.MinPointsPerApproach} needed"));
        }

        if (fromAbove < options.MinPointsPerApproach)
        {
            warnings.Add(FormattableString.Invariant(
                $"only {fromAbove} settled point(s) reached from above, {options.MinPointsPerApproach} needed"));
        }

        if (fromBelow < options.MinPointsPerApproach || fromAbove < options.MinPointsPerApproach)
        {
            return NotDeterminateBacklash(warnings);
        }

        return FitBacklash(points, fromBelow, fromAbove, warnings, options);
    }

    /// <summary>The three-unknown backlash fit: regressors <c>[command, 1, approach]</c>.</summary>
    private static SteeringBacklashReport FitBacklash(
        IReadOnlyList<SteeringBacklashPoint> points,
        int fromBelow,
        int fromAbove,
        List<string> warnings,
        SteerResponseOptions options)
    {
        const int UnknownCount = 3;

        var normalMatrix = new double[UnknownCount, UnknownCount];
        var rightHandSide = new double[UnknownCount];
        var basis = new double[UnknownCount];

        for (int i = 0; i < points.Count; i++)
        {
            basis[0] = points[i].CommandedSteerAngleDegrees;
            basis[1] = 1.0;
            basis[2] = points[i].ApproachSign;
            LinearLeastSquares.Accumulate(
                normalMatrix, rightHandSide, basis, points[i].MeasuredSteerAngleDegrees);
        }

        var solution = LinearLeastSquares.Solve(normalMatrix, rightHandSide);
        double pivot = solution?.SmallestPivot ?? 0.0;
        if (solution is null || pivot < options.MinPivot)
        {
            warnings.Add(FormattableString.Invariant(
                $"the settled points do not separate the play from the intercept (smallest normalized pivot {pivot:F4}, ")
                + FormattableString.Invariant($"need {options.MinPivot:F2})"));
            return NotDeterminateBacklash(warnings);
        }

        double slope = solution.Coefficients[0];
        double intercept = solution.Coefficients[1];
        double approachOffset = solution.Coefficients[2];
        double backlash = -2.0 * approachOffset;

        double sumSquares = 0.0;
        for (int i = 0; i < points.Count; i++)
        {
            double predicted = (slope * points[i].CommandedSteerAngleDegrees)
                + intercept + (approachOffset * points[i].ApproachSign);
            double residual = points[i].MeasuredSteerAngleDegrees - predicted;
            sumSquares += residual * residual;
        }

        int degreesOfFreedom = points.Count - UnknownCount;
        double variance = solution.Variance(2, sumSquares, degreesOfFreedom);

        var fit = new SteeringBacklashFit
        {
            BacklashDegrees = backlash,
            Slope = slope,
            InterceptDegrees = intercept,
            ApproachOffsetDegrees = approachOffset,
            BacklashStdDevDegrees = double.IsNaN(variance) ? double.NaN : 2.0 * Math.Sqrt(variance),
            RmsDegrees = Math.Sqrt(sumSquares / points.Count),
            FromBelowCount = fromBelow,
            FromAboveCount = fromAbove,
        };

        return new SteeringBacklashReport
        {
            IsDeterminate = true,
            IsAccepted = warnings.Count == 0,
            Fit = fit,
            Warnings = warnings,
            Message = FormattableString.Invariant(
                $"steering play {backlash:F2}° from {fromBelow} + {fromAbove} settled points"),
        };
    }

    private static SteeringBacklashReport NotDeterminateBacklash(List<string> warnings)
        => new()
        {
            IsDeterminate = false,
            IsAccepted = false,
            Fit = null,
            Warnings = warnings,
            Message = string.Join("; ", warnings),
        };

    private static SteerStepResponseReport NotDeterminate(string reason)
        => new()
        {
            IsDeterminate = false,
            IsAccepted = false,
            Fit = null,
            Warnings = new[] { reason },
            Message = reason,
        };

    private static string WarningSuffix(IReadOnlyList<string> warnings)
        => warnings.Count == 0 ? string.Empty : " — " + string.Join("; ", warnings);

}




