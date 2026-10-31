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
/// Algorithm 2 — WAS zero and gain, fitted from the yaw rate the vehicle actually turned
/// with. Simulation only (plan stage 1: known parameters, several random seeds, white
/// <b>and</b> correlated noise, plus the negative cases).
///
/// The injected truth is a sensor that reads 10% high and carries a 1.4° zero error, so the
/// fit should return zero = 1.4° and gain = 1/1.1 ≈ 0.909 (the gain is the <i>inverse</i>
/// sensor scale). The plan's acceptance gate is 0.15° on the zero and 0.02 on the gain.
/// </summary>
[TestFixture]
public class WasZeroGainCalibrationTests
{
    private const double Wheelbase = 2.5;
    private const double TrueZeroDegrees = 1.4;
    private const double SensorScale = 1.1;
    private const double TrueGain = 1.0 / SensorScale;   // what the fit's `g` should be

    private const double ZeroToleranceDegrees = 0.15;
    private const double GainTolerance = 0.02;

    // ── Simulation ───────────────────────────────────────────────────────────

    private static double Gaussian(Random rng)
    {
        double u1 = 1.0 - rng.NextDouble();
        double u2 = 1.0 - rng.NextDouble();
        return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
    }

    /// <summary>
    /// Synthesize a left/right weave. The <i>true</i> steer angle sweeps a full sine of
    /// <paramref name="amplitudeDegrees"/>, the yaw rate follows from the Ackermann
    /// geometry (<c>ω = v·tan δ / L</c>) and the sensor reads <c>scale·δ + zero</c> with
    /// optional white, time-correlated and gyro noise.
    /// </summary>
    private static WasCalibrationSample[] Weave(
        int samples,
        int seed,
        double amplitudeDegrees = 15.0,
        double speedMps = 2.0,
        double whiteNoiseDegrees = 0.0,
        double correlatedNoiseDegrees = 0.0,
        double gyroNoiseRadPerSec = 0.0,
        double rollDegrees = 0.0,
        double measuredScale = SensorScale)
    {
        const double rho = 0.9;
        const double degreesToRadians = Math.PI / 180.0;

        var rng = new Random(seed);
        var result = new List<WasCalibrationSample>(samples);
        double correlated = 0.0;

        for (int i = 0; i < samples; i++)
        {
            double phase = 2.0 * Math.PI * (i + 0.5) / samples;
            double trueSteerDegrees = amplitudeDegrees * Math.Sin(phase);
            double trueSteerRadians = trueSteerDegrees * degreesToRadians;
            double yawRate = speedMps * Math.Tan(trueSteerRadians) / Wheelbase;

            double noise = Gaussian(rng) * whiteNoiseDegrees;
            correlated = (rho * correlated)
                + (Math.Sqrt(1.0 - (rho * rho)) * correlatedNoiseDegrees * Gaussian(rng));
            noise += correlated;

            double measured = (measuredScale * trueSteerDegrees) + TrueZeroDegrees + noise;
            double measuredYawRate = yawRate + (gyroNoiseRadPerSec * Gaussian(rng));

            result.Add(new WasCalibrationSample(measured, measuredYawRate, speedMps, rollDegrees));
        }

        return result.ToArray();
    }

    /// <summary>The realistic case: white plus time-correlated sensor noise and a noisy gyro.</summary>
    private static WasCalibrationSample[] Realistic(int samples, int seed)
        => Weave(samples, seed, whiteNoiseDegrees: 0.1, correlatedNoiseDegrees: 0.15, gyroNoiseRadPerSec: 0.002);

    // ── The main case ────────────────────────────────────────────────────────

    [Test]
    public void Fit_NoiseFreeData_ReturnsTheInjectedZeroAndGainExactly()
    {
        var report = WasZeroGainCalibrator.Fit(Weave(400, seed: 1), Wheelbase);

        Assert.That(report.IsDeterminate, Is.True, report.Message);
        Assert.That(report.IsAccepted, Is.True, report.Message);
        Assert.That(report.Fit, Is.Not.Null);

        var fit = report.Fit!.Value;
        Assert.That(fit.GainObservable, Is.True);
        Assert.That(fit.MeasuredSteerAngleOffsetDegrees, Is.EqualTo(TrueZeroDegrees).Within(1e-9));
        Assert.That(fit.Gain, Is.EqualTo(TrueGain).Within(1e-9));
        Assert.That(fit.RmsDegrees, Is.LessThan(1e-9));
        Assert.That(fit.SampleCount, Is.EqualTo(400));
        // The range is in the sensor's own units: the 1.1× scale shows up here too.
        Assert.That(fit.SteerRangeDegrees, Is.EqualTo(SensorScale * 30.0).Within(0.01));
    }

    [TestCase(1)]
    [TestCase(7)]
    [TestCase(42)]
    [TestCase(1234)]
    [TestCase(20261031)]
    public void Fit_WhiteAndCorrelatedNoise_MeetsThePlanAcceptanceGates(int seed)
    {
        var report = WasZeroGainCalibrator.Fit(Realistic(600, seed), Wheelbase);

        Assert.That(report.IsAccepted, Is.True, report.Message);
        Assert.That(report.Warnings, Is.Empty, string.Join(" | ", report.Warnings));

        var fit = report.Fit!.Value;
        Assert.That(fit.MeasuredSteerAngleOffsetDegrees, Is.EqualTo(TrueZeroDegrees).Within(ZeroToleranceDegrees),
            "the zero must stay inside the plan's 0.15°");
        Assert.That(fit.Gain, Is.EqualTo(TrueGain).Within(GainTolerance),
            "the gain must stay inside the plan's 0.02");

        Assert.That(fit.RmsDegrees, Is.GreaterThan(0.0), "the noise must show up in the residuals");
        Assert.That(fit.ZeroStdDevDegrees, Is.GreaterThan(0.0));
        Assert.That(fit.ZeroStdDevDegrees, Is.LessThan(ZeroToleranceDegrees),
            "the reported uncertainty should be tighter than the gate, or the gate is luck");
        Assert.That(double.IsFinite(fit.GainStdDev), Is.True);
        Assert.That(fit.GainStdDev, Is.GreaterThan(0.0));
    }

    // ── Negative cases the plan calls for ────────────────────────────────────

    [Test]
    public void Fit_StraightDrivingOnly_ReportsTheZeroAndSaysTheGainIsNotObservable()
    {
        // 2° of steering is under the plan's 8° range: the slope hides in the noise.
        var report = WasZeroGainCalibrator.Fit(
            Weave(400, seed: 3, amplitudeDegrees: 2.0, whiteNoiseDegrees: 0.05), Wheelbase);

        Assert.That(report.IsDeterminate, Is.True, report.Message);
        Assert.That(report.IsAccepted, Is.True, report.Message);

        var fit = report.Fit!.Value;
        Assert.That(fit.GainObservable, Is.False);
        Assert.That(fit.Gain, Is.EqualTo(1.0));
        Assert.That(double.IsNaN(fit.GainStdDev), Is.True);
        Assert.That(fit.MeasuredSteerAngleOffsetDegrees, Is.EqualTo(TrueZeroDegrees).Within(ZeroToleranceDegrees));
        Assert.That(report.Notes, Is.Not.Empty);
        Assert.That(string.Join(" ", report.Notes), Does.Contain("gain"));
    }

    [Test]
    public void Fit_TooFewSamples_IsDeterminateButNotAccepted()
    {
        var report = WasZeroGainCalibrator.Fit(Weave(100, seed: 5), Wheelbase);

        Assert.That(report.IsDeterminate, Is.True, report.Message);
        Assert.That(report.IsAccepted, Is.False);
        Assert.That(report.Warnings.Count, Is.EqualTo(1));
        Assert.That(report.Warnings[0], Does.Contain("300"));
        Assert.That(report.Message, Does.Contain("300"));
    }

    [Test]
    public void Fit_ReversedSignConvention_WarnsInsteadOfAcceptingTheResult()
    {
        // A wiring fault that flips the measured angle gives a negative slope; the fit is
        // still solvable, so it must warn rather than silently hand back a bad zero.
        var report = WasZeroGainCalibrator.Fit(
            Weave(400, seed: 6, measuredScale: -SensorScale), Wheelbase);

        Assert.That(report.IsDeterminate, Is.True, report.Message);
        Assert.That(report.Fit!.Value.Gain, Is.LessThan(0.0));
        Assert.That(report.IsAccepted, Is.False);
        Assert.That(string.Join(" ", report.Warnings), Does.Contain("sign"));
    }

    [Test]
    public void Fit_NoSamples_IsNotDeterminate()
    {
        var report = WasZeroGainCalibrator.Fit(Array.Empty<WasCalibrationSample>(), Wheelbase);

        Assert.That(report.IsDeterminate, Is.False);
        Assert.That(report.Fit, Is.Null);
        Assert.That(report.Message, Does.Contain("3 usable samples"));
    }

    [Test]
    public void Fit_NonPositiveWheelbase_Throws()
    {
        Assert.That(
            () => WasZeroGainCalibrator.Fit(Weave(400, seed: 8), 0.0),
            Throws.InstanceOf<ArgumentOutOfRangeException>());
    }

    // ── The collector ────────────────────────────────────────────────────────

    [Test]
    public void Collector_FiltersEachReasonAndReportsIt()
    {
        var collector = new WasCalibrationCollector();
        collector.Start();

        Assert.That(collector.TryAdd(1.0, 0.05, 2.0, 0.0, out string? reason), Is.True);
        Assert.That(reason, Is.Null);
        Assert.That(collector.TryAdd(4.0, 0.05, 2.0, 0.0, out _), Is.True);
        Assert.That(collector.SteerRangeDegrees, Is.EqualTo(3.0).Within(1e-9));

        Assert.That(collector.TryAdd(1.0, 0.05, 0.4, 0.0, out reason), Is.False);
        Assert.That(reason, Does.Contain("speed"));
        Assert.That(collector.TryAdd(1.0, 0.05, 2.0, 6.0, out reason), Is.False);
        Assert.That(reason, Does.Contain("roll"));
        Assert.That(collector.TryAdd(1.0, 0.9, 2.0, 0.0, out reason), Is.False);
        Assert.That(reason, Does.Contain("yaw rate"));
        Assert.That(collector.TryAdd(45.0, 0.05, 2.0, 0.0, out reason), Is.False);
        Assert.That(reason, Does.Contain("steer angle"));
        Assert.That(collector.TryAdd(double.NaN, 0.05, 2.0, 0.0, out reason), Is.False);
        Assert.That(reason, Does.Contain("non-finite"));
        Assert.That(collector.Count, Is.EqualTo(2));

        collector.Stop();
        Assert.That(collector.TryAdd(1.0, 0.05, 2.0, 0.0, out reason), Is.False);
        Assert.That(reason, Does.Contain("not recording"));

        collector.Reset();
        Assert.That(collector.Count, Is.EqualTo(0));
        Assert.That(collector.SteerRangeDegrees, Is.EqualTo(0.0));
        Assert.That(collector.Fit(Wheelbase).IsDeterminate, Is.False);
    }

    [Test]
    public void Collector_FitUsesOnlyAcceptedSamples()
    {
        var collector = new WasCalibrationCollector();
        var samples = Weave(600, seed: 9, whiteNoiseDegrees: 0.1, correlatedNoiseDegrees: 0.15);

        collector.Start();
        int rejected = 0;
        for (int i = 0; i < samples.Length; i++)
        {
            var sample = samples[i];
            // Every fifth sample is taken on a slope and must not reach the fit.
            double rollDegrees = i % 5 == 0 ? 6.0 : 0.5;
            bool kept = collector.TryAdd(
                sample.MeasuredSteerAngleDegrees,
                sample.YawRateRadiansPerSecond,
                sample.SpeedMetersPerSecond,
                rollDegrees,
                out _);
            if (!kept) rejected++;
        }

        var report = collector.Fit(Wheelbase);

        Assert.That(rejected, Is.EqualTo(120));
        Assert.That(collector.Count, Is.EqualTo(samples.Length - rejected));
        Assert.That(report.IsAccepted, Is.True, report.Message);
        Assert.That(report.Fit!.Value.SampleCount, Is.EqualTo(samples.Length - rejected));
        Assert.That(report.Fit!.Value.MeasuredSteerAngleOffsetDegrees,
            Is.EqualTo(TrueZeroDegrees).Within(ZeroToleranceDegrees));
        Assert.That(report.Fit!.Value.Gain, Is.EqualTo(TrueGain).Within(GainTolerance));
    }

    // ── Diagnostic report (run on demand) ───────────────────────────────────

    [Test]
    [Explicit("Diagnostic dump — prints the fits for the seeded noisy runs.")]
    public void Report()
    {
        foreach (int seed in new[] { 1, 7, 42, 1234, 20261031 })
        {
            var fit = WasZeroGainCalibrator.Fit(Realistic(600, seed), Wheelbase).Fit!.Value;
            TestContext.Out.WriteLine(FormattableString.Invariant(
                $"seed {seed,9}: zero {fit.MeasuredSteerAngleOffsetDegrees:F3}° (err {(fit.MeasuredSteerAngleOffsetDegrees - TrueZeroDegrees) * 1000.0:F0} m°), gain {fit.Gain:F4} (err {fit.Gain - TrueGain:F4}), se {fit.ZeroStdDevDegrees:F4}°/{fit.GainStdDev:F5}, RMS {fit.RmsDegrees:F3}°"));
        }
    }

    // ── Locale safety (a comma-decimal culture must not leak into the texts) ─

    [Test]
    public void Report_UsesInvariantNumberFormatting()
    {
        var previous = CultureInfo.CurrentCulture;
        try
        {
            CultureInfo.CurrentCulture = new CultureInfo("fi-FI");

            // Noise-free: the numbers are exactly 1.40 and 0.909.
            var clean = WasZeroGainCalibrator.Fit(Weave(400, seed: 2), Wheelbase);
            Assert.That(clean.Message, Does.Contain("1.40"));
            Assert.That(clean.Message, Does.Contain("0.909"));
            Assert.That(clean.Message, Does.Not.Contain("1,40"));
            Assert.That(clean.Message, Does.Not.Contain("0,909"));

            // A gate warning that carries decimals (2.20° of steering < 8.0°).
            string notes = string.Join(" | ", WasZeroGainCalibrator.Fit(
                Weave(400, seed: 4, amplitudeDegrees: 1.0), Wheelbase).Notes);
            Assert.That(notes, Does.Contain("2.20"));
            Assert.That(notes, Does.Not.Contain("2,20"));
        }
        finally
        {
            CultureInfo.CurrentCulture = previous;
        }
    }
}


