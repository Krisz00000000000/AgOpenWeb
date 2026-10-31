// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;
using System.Collections.Generic;
using System.Globalization;
using AgOpenWeb.Services.Calibration;
using NUnit.Framework;

namespace AgOpenWeb.Services.Tests.Calibration;

/// <summary>
/// Algorithm 3 — steering step response and backlash. Simulation only (plan stage 1): known
/// dead time, time constant and gain recovered from a noisy step, a rate-limited servo
/// recognised, and a known play recovered from settled points.
/// </summary>
[TestFixture]
public class SteerResponseIdentifierTests
{
    private const double TrueDeadTime = 0.15;
    private const double TrueTau = 0.25;
    private const double TrueGain = 0.97;
    private const double TrueBacklash = 1.2;

    // The plan's acceptance for the simulation: 30 ms, 40 ms, 0.02.
    private const double DeadTimeTolerance = 0.03;
    private const double TauTolerance = 0.04;
    private const double GainTolerance = 0.02;

    private static double Gaussian(Random rng)
    {
        double u1 = 1.0 - rng.NextDouble();
        double u2 = 1.0 - rng.NextDouble();
        return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
    }

    /// <summary>
    /// Synthesize a step response: 10 samples at rest, then a commanded step, then a
    /// first-order lag with dead time (or a rate-limited ramp when
    /// <paramref name="rateLimitDegreesPerSecond"/> is given), plus white sensor noise.
    /// </summary>
    private static SteerResponseSample[] StepResponse(
        double deadTime = TrueDeadTime,
        double tau = TrueTau,
        double gain = TrueGain,
        double stepDegrees = 5.0,
        double noiseDegrees = 0.02,
        int seed = 1,
        int samplesBefore = 10,
        int samplesAfter = 90,
        double rateLimitDegreesPerSecond = 0.0)
    {
        const double sampleRateHz = 50.0;
        double dt = 1.0 / sampleRateHz;
        double stepTime = samplesBefore * dt;

        var rng = new Random(seed);
        var samples = new List<SteerResponseSample>(samplesBefore + samplesAfter);
        double previous = 0.0;

        for (int i = 0; i < samplesBefore + samplesAfter; i++)
        {
            double time = i * dt;
            double command = i < samplesBefore ? 0.0 : stepDegrees;

            double response;
            if (rateLimitDegreesPerSecond > 0.0)
            {
                double target = time > stepTime + deadTime ? gain * stepDegrees : 0.0;
                double maxMove = rateLimitDegreesPerSecond * dt;
                response = previous + Math.Clamp(target - previous, -maxMove, maxMove);
            }
            else
            {
                double elapsed = time - stepTime - deadTime;
                response = elapsed <= 0.0 ? 0.0 : gain * stepDegrees * (1.0 - Math.Exp(-elapsed / tau));
            }

            samples.Add(new SteerResponseSample(
                time, command, response + (noiseDegrees * Gaussian(rng))));
            previous = response;
        }

        return samples.ToArray();
    }

    // ── The step response ───────────────────────────────────────────────────

    [Test]
    public void FitStepResponse_NoiseFreeData_ReturnsTheInjectedParameters()
    {
        var report = SteerResponseIdentifier.FitStepResponse(StepResponse(noiseDegrees: 0.0));

        Assert.That(report.IsDeterminate, Is.True, report.Message);
        Assert.That(report.IsAccepted, Is.True, report.Message);

        var fit = report.Fit!.Value;
        Assert.That(fit.DeadTimeSeconds, Is.EqualTo(TrueDeadTime).Within(0.002));
        Assert.That(fit.TimeConstantSeconds, Is.EqualTo(TrueTau).Within(0.01));
        Assert.That(fit.SteadyStateGain, Is.EqualTo(TrueGain).Within(0.002));
        Assert.That(fit.StepAmplitudeDegrees, Is.EqualTo(5.0).Within(1e-9));
        Assert.That(fit.StepTimeSeconds, Is.EqualTo(0.2).Within(1e-9));
        Assert.That(fit.IsRateLimited, Is.False);
        Assert.That(fit.PreStepSampleCount, Is.EqualTo(10));
        Assert.That(fit.PostStepSampleCount, Is.EqualTo(90));
    }

    [TestCase(1)]
    [TestCase(7)]
    [TestCase(42)]
    [TestCase(1234)]
    [TestCase(20261031)]
    public void FitStepResponse_NoisyData_MeetsThePlanAcceptanceGates(int seed)
    {
        var report = SteerResponseIdentifier.FitStepResponse(StepResponse(seed: seed));

        Assert.That(report.IsAccepted, Is.True, report.Message);

        var fit = report.Fit!.Value;
        Assert.That(fit.DeadTimeSeconds, Is.EqualTo(TrueDeadTime).Within(DeadTimeTolerance),
            "dead time must stay inside the plan's 30 ms");
        Assert.That(fit.TimeConstantSeconds, Is.EqualTo(TrueTau).Within(TauTolerance),
            "time constant must stay inside the plan's 40 ms");
        Assert.That(fit.SteadyStateGain, Is.EqualTo(TrueGain).Within(GainTolerance),
            "gain must stay inside the plan's 0.02");

        Assert.That(fit.SensorNoiseDegrees, Is.EqualTo(0.02).Within(0.01));
        Assert.That(fit.FitRmsDegrees, Is.LessThan(0.05));
        Assert.That(fit.ShapeRatio, Is.EqualTo(3.25).Within(0.5),
            "an exponential response has (t95 − t10)/(t63 − t10) ≈ 3.25");
        Assert.That(fit.IsRateLimited, Is.False);
    }

    [Test]
    public void FitStepResponse_RateLimitedServo_IsFlaggedAndNotAccepted()
    {
        // A servo that can only move at 4 °/s ramps instead of lagging: the first-order
        // model is not valid, and the plan asks for a smaller step instead.
        var report = SteerResponseIdentifier.FitStepResponse(
            StepResponse(
                stepDegrees: 10.0,
                rateLimitDegreesPerSecond: 4.0,
                noiseDegrees: 0.0,
                samplesAfter: 290));

        Assert.That(report.IsDeterminate, Is.True, report.Message);
        Assert.That(report.IsAccepted, Is.False);
        Assert.That(report.Fit!.Value.IsRateLimited, Is.True);
        Assert.That(report.Fit!.Value.ShapeRatio, Is.LessThan(2.0),
            "a ramp has a much flatter shape ratio than an exponential");
        Assert.That(string.Join(" ", report.Warnings), Does.Contain("rate limited"));
    }

    // ── The gates ───────────────────────────────────────────────────────────

    [Test]
    public void FitStepResponse_TooFewPreStepSamples_IsNotDeterminate()
    {
        var report = SteerResponseIdentifier.FitStepResponse(
            StepResponse(samplesBefore: 4, samplesAfter: 90));

        Assert.That(report.IsDeterminate, Is.False);
        Assert.That(report.Message, Does.Contain("before the step"));
    }

    [Test]
    public void FitStepResponse_TooSmallAStep_IsNotDeterminate()
    {
        var report = SteerResponseIdentifier.FitStepResponse(StepResponse(stepDegrees: 0.4));

        Assert.That(report.IsDeterminate, Is.False);
        Assert.That(report.Message, Does.Contain("the step was only"));
    }

    [Test]
    public void FitStepResponse_CommandThatMovesTwice_IsNotAStepResponse()
    {
        var samples = StepResponse(noiseDegrees: 0.0);
        for (int i = samples.Length / 2; i < samples.Length; i++)
        {
            samples[i] = samples[i] with
            {
                CommandedSteerAngleDegrees = samples[i].CommandedSteerAngleDegrees + 5.0,
            };
        }

        var report = SteerResponseIdentifier.FitStepResponse(samples);

        Assert.That(report.IsDeterminate, Is.False);
        Assert.That(report.Message, Does.Contain("did not hold"));
    }

    // ── Backlash ────────────────────────────────────────────────────────────

    /// <summary>
    /// Settled points for a steering box with a known play:
    /// <c>measured = command + e·approach</c> with <c>e = −play/2</c>.
    /// </summary>
    private static SteeringBacklashPoint[] BacklashPoints(
        double playDegrees = TrueBacklash, double noiseDegrees = 0.0, int seed = 1)
    {
        var commands = new[] { -8.0, -4.0, 4.0, 8.0 };
        var rng = new Random(seed);
        var points = new List<SteeringBacklashPoint>(commands.Length * 2);
        double offset = -playDegrees / 2.0;

        foreach (double approach in new[] { 1.0, -1.0 })
        {
            foreach (double command in commands)
            {
                double measured = command + (offset * approach) + (noiseDegrees * Gaussian(rng));
                points.Add(new SteeringBacklashPoint(command, measured, approach));
            }
        }

        return points.ToArray();
    }

    [Test]
    public void EstimateBacklash_NoiseFreePoints_ReturnsTheInjectedPlay()
    {
        var report = SteerResponseIdentifier.EstimateBacklash(BacklashPoints());

        Assert.That(report.IsDeterminate, Is.True, report.Message);
        Assert.That(report.IsAccepted, Is.True, report.Message);

        var fit = report.Fit!.Value;
        Assert.That(fit.BacklashDegrees, Is.EqualTo(TrueBacklash).Within(1e-9));
        Assert.That(fit.Slope, Is.EqualTo(1.0).Within(1e-9));
        Assert.That(fit.ApproachOffsetDegrees, Is.EqualTo(-TrueBacklash / 2.0).Within(1e-9));
        Assert.That(fit.FromBelowCount, Is.EqualTo(4));
        Assert.That(fit.FromAboveCount, Is.EqualTo(4));
        Assert.That(fit.RmsDegrees, Is.LessThan(1e-9));
    }

    [TestCase(1)]
    [TestCase(7)]
    [TestCase(42)]
    public void EstimateBacklash_NoisyPoints_StayWithinATenthOfADegree(int seed)
    {
        var report = SteerResponseIdentifier.EstimateBacklash(
            BacklashPoints(noiseDegrees: 0.05, seed: seed));

        Assert.That(report.IsAccepted, Is.True, report.Message);
        Assert.That(report.Fit!.Value.BacklashDegrees, Is.EqualTo(TrueBacklash).Within(0.1));
        Assert.That(report.Fit!.Value.BacklashStdDevDegrees, Is.GreaterThan(0.0));
    }

    [Test]
    public void EstimateBacklash_OneApproachDirectionOnly_IsNotDeterminate()
    {
        var fromBelowOnly = new List<SteeringBacklashPoint>();
        foreach (var point in BacklashPoints())
        {
            if (point.ApproachSign > 0) fromBelowOnly.Add(point);
        }

        var report = SteerResponseIdentifier.EstimateBacklash(fromBelowOnly);

        Assert.That(report.IsDeterminate, Is.False);
        Assert.That(report.Fit, Is.Null);
        Assert.That(report.Message, Does.Contain("from above"));
    }

    [Test]
    public void EstimateBacklash_RejectsBadApproachSigns()
    {
        var points = new[] { new SteeringBacklashPoint(1.0, 1.0, 0.0) };

        Assert.That(
            () => SteerResponseIdentifier.EstimateBacklash(points),
            Throws.InstanceOf<ArgumentOutOfRangeException>());
    }

    // ── Diagnostic report and locale safety ────────────────────────────────

    [Test]
    [Explicit("Diagnostic dump — prints the fits for the seeded noisy runs.")]
    public void Report()
    {
        foreach (int seed in new[] { 1, 7, 42, 1234, 20261031 })
        {
            var fit = SteerResponseIdentifier.FitStepResponse(StepResponse(seed: seed)).Fit!.Value;
            TestContext.Out.WriteLine(FormattableString.Invariant(
                $"seed {seed,9}: dead {fit.DeadTimeSeconds * 1000.0:F1} ms (err {(fit.DeadTimeSeconds - TrueDeadTime) * 1000.0:F0} ms), tau {fit.TimeConstantSeconds * 1000.0:F0} ms (err {(fit.TimeConstantSeconds - TrueTau) * 1000.0:F0} ms), K {fit.SteadyStateGain:F3} (err {fit.SteadyStateGain - TrueGain:F3}), shape {fit.ShapeRatio:F2}, RMS {fit.FitRmsDegrees:F3}°"));
        }

        var rateLimited = SteerResponseIdentifier.FitStepResponse(
            StepResponse(stepDegrees: 10.0, rateLimitDegreesPerSecond: 4.0, noiseDegrees: 0.0, samplesAfter: 290)).Fit!.Value;
        TestContext.Out.WriteLine(FormattableString.Invariant(
            $"rate limited: rate ratio {rateLimited.RateRatio:F2}, shape {rateLimited.ShapeRatio:F2}, flagged {rateLimited.IsRateLimited}"));
    }

    [Test]
    public void Report_UsesInvariantNumberFormatting()
    {
        var previous = CultureInfo.CurrentCulture;
        try
        {
            CultureInfo.CurrentCulture = new CultureInfo("fi-FI");

            var message = SteerResponseIdentifier.FitStepResponse(StepResponse(noiseDegrees: 0.0)).Message;
            Assert.That(message, Does.Contain("0.970"));
            Assert.That(message, Does.Contain("150 ms"));
            Assert.That(message, Does.Not.Contain("0,970"));

            var backlash = SteerResponseIdentifier.EstimateBacklash(BacklashPoints()).Message;
            Assert.That(backlash, Does.Contain("1.20"));
            Assert.That(backlash, Does.Not.Contain("1,20"));
        }
        finally
        {
            CultureInfo.CurrentCulture = previous;
        }
    }
}