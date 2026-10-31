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
/// Algorithm 1 — antenna lateral offset and height, fitted from the raw antenna's
/// deviation off the line. Simulation only (plan stage 1: known parameters, several
/// random seeds, white <b>and</b> correlated noise, plus the negative cases).
/// The injected truth is 0.07 m offset, 2.85 m height, 0.03 m line position, and the
/// acceptance gates are the plan's: 1 cm on the offset, 8 cm on the height.
/// </summary>
[TestFixture]
public class AntennaOffsetHeightCalibrationTests
{
    private const double TrueOffset = 0.07;
    private const double TrueHeight = 2.85;
    private const double TrueLineOffset = 0.03;

    private const double OffsetTolerance = 0.01;   // plan acceptance: 1 cm
    private const double HeightTolerance = 0.08;   // plan acceptance: 8 cm
    private const double LineOffsetTolerance = 0.05;

    // ── Simulation ───────────────────────────────────────────────────────────

    private static double Gaussian(Random rng)
    {
        double u1 = 1.0 - rng.NextDouble();
        double u2 = 1.0 - rng.NextDouble();
        return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
    }

    /// <summary>
    /// Build samples from the true parameters. The two passes are the same wheel
    /// track driven opposite ways (a slightly different roll phase so the pairing is
    /// not exact), rolling from flat through a side slope and back. The noise can be
    /// white, or white plus an AR(1) component — real RTK error is time-correlated,
    /// so a fit that only survives white noise would be optimistic.
    /// </summary>
    private static AntennaCalibrationSample[] Synthesize(
        int samplesPerDirection,
        int seed,
        double whiteNoiseStdDevMeters = 0.0,
        double correlatedNoiseStdDevMeters = 0.0,
        double rollAmplitudeDegrees = 6.0,
        double rollBiasDegrees = 0.0,
        int onlyDirection = 0)
    {
        const double rho = 0.9;
        var rng = new Random(seed);
        var samples = new List<AntennaCalibrationSample>(samplesPerDirection * 2);
        double correlated = 0.0;

        for (int pass = 0; pass < 2; pass++)
        {
            int directionSign = pass == 0 ? 1 : -1;
            if (onlyDirection != 0 && directionSign != onlyDirection) continue;

            double phaseOffset = pass == 0 ? 0.0 : 0.37;
            for (int i = 0; i < samplesPerDirection; i++)
            {
                double phase = (2.0 * Math.PI * (i + 0.5) / samplesPerDirection) + phaseOffset;
                double rollDegrees = rollBiasDegrees + (rollAmplitudeDegrees * Math.Sin(phase));

                double noise = Gaussian(rng) * whiteNoiseStdDevMeters;
                correlated = (rho * correlated)
                    + (Math.Sqrt(1.0 - (rho * rho)) * correlatedNoiseStdDevMeters * Gaussian(rng));
                noise += correlated;

                double deviation = (directionSign
                        * (TrueOffset + (TrueHeight * Math.Sin(rollDegrees * Math.PI / 180.0))))
                    + TrueLineOffset + noise;

                samples.Add(new AntennaCalibrationSample(directionSign, rollDegrees, deviation));
            }
        }

        return samples.ToArray();
    }

    /// <summary>The realistic case: white plus time-correlated noise, both directions.</summary>
    private static AntennaCalibrationSample[] Realistic(int samplesPerDirection, int seed)
        => Synthesize(samplesPerDirection, seed, whiteNoiseStdDevMeters: 0.003, correlatedNoiseStdDevMeters: 0.005);

    /// <summary>A monotone roll ramp of a given total spread, both directions.</summary>
    private static AntennaCalibrationSample[] MonotoneRoll(int samplesPerDirection, double rollSpreadDegrees)
    {
        var samples = new List<AntennaCalibrationSample>(samplesPerDirection * 2);
        int total = samplesPerDirection * 2;
        for (int pass = 0; pass < 2; pass++)
        {
            int directionSign = pass == 0 ? 1 : -1;
            for (int i = 0; i < samplesPerDirection; i++)
            {
                int index = (pass * samplesPerDirection) + i;
                double rollDegrees = (-rollSpreadDegrees / 2.0) + (rollSpreadDegrees * index / (total - 1.0));
                double deviation = (directionSign
                        * (TrueOffset + (TrueHeight * Math.Sin(rollDegrees * Math.PI / 180.0))))
                    + TrueLineOffset;
                samples.Add(new AntennaCalibrationSample(directionSign, rollDegrees, deviation));
            }
        }

        return samples.ToArray();
    }

    // ── The main case ────────────────────────────────────────────────────────

    [Test]
    public void Fit_NoiseFreeData_ReturnsTheInjectedParametersExactly()
    {
        var report = AntennaOffsetHeightCalibrator.Fit(Synthesize(150, seed: 1));

        Assert.That(report.IsDeterminate, Is.True, report.Message);
        Assert.That(report.IsAccepted, Is.True, report.Message);
        Assert.That(report.Fit, Is.Not.Null);

        var fit = report.Fit!.Value;
        Assert.That(fit.LateralOffsetMeters, Is.EqualTo(TrueOffset).Within(1e-9));
        Assert.That(fit.HeightAboveRollAxisMeters, Is.EqualTo(TrueHeight).Within(1e-9));
        Assert.That(fit.LineOffsetMeters, Is.EqualTo(TrueLineOffset).Within(1e-9));
        Assert.That(fit.RmsMeters, Is.LessThan(1e-9));
        Assert.That(fit.ForwardSampleCount, Is.EqualTo(150));
        Assert.That(fit.ReverseSampleCount, Is.EqualTo(150));
    }

    [TestCase(1)]
    [TestCase(7)]
    [TestCase(42)]
    [TestCase(1234)]
    [TestCase(20261031)]
    public void Fit_WhiteAndCorrelatedNoise_MeetsThePlanAcceptanceGates(int seed)
    {
        var report = AntennaOffsetHeightCalibrator.Fit(Realistic(samplesPerDirection: 300, seed));

        Assert.That(report.IsAccepted, Is.True, report.Message);

        var fit = report.Fit!.Value;
        Assert.That(fit.LateralOffsetMeters, Is.EqualTo(TrueOffset).Within(OffsetTolerance),
            "offset error must stay under 1 cm");
        Assert.That(fit.HeightAboveRollAxisMeters, Is.EqualTo(TrueHeight).Within(HeightTolerance),
            "height error must stay under 8 cm");
        Assert.That(fit.LineOffsetMeters, Is.EqualTo(TrueLineOffset).Within(LineOffsetTolerance));

        Assert.That(fit.RmsMeters, Is.GreaterThan(0.001), "the noise must show up in the residuals");
        Assert.That(fit.RmsMeters, Is.LessThan(0.03));
        Assert.That(fit.LateralOffsetStdDevMeters, Is.GreaterThan(0.0));
        Assert.That(fit.HeightStdDevMeters, Is.GreaterThan(fit.LateralOffsetStdDevMeters),
            "the height is the harder of the two to pin down");
        Assert.That(double.IsFinite(fit.HeightStdDevMeters), Is.True);
    }

    // ── Negative cases the plan calls for ────────────────────────────────────

    [Test]
    public void Fit_OneDirectionOnly_IsNotDeterminate()
    {
        // 400 noise-free samples, all driving A→B: nothing separates the antenna
        // offset, the antenna height and the line position.
        var report = AntennaOffsetHeightCalibrator.Fit(
            Synthesize(400, seed: 3, onlyDirection: 1));

        Assert.That(report.IsDeterminate, Is.False);
        Assert.That(report.IsAccepted, Is.False);
        Assert.That(report.Fit, Is.Null);
        Assert.That(report.Message, Does.Contain("both driving directions"));
        Assert.That(report.Warnings, Is.Not.Empty);
    }

    [Test]
    public void Fit_ConstantRollOnBothDirections_IsNotDeterminate()
    {
        // Both directions recorded, but the ground never tilts: h·sin(roll) is a
        // constant and hides inside the antenna offset.
        var report = AntennaOffsetHeightCalibrator.Fit(
            Synthesize(300, seed: 4, rollAmplitudeDegrees: 0.0, rollBiasDegrees: 5.0));

        Assert.That(report.IsDeterminate, Is.False);
        Assert.That(report.Fit, Is.Null);
        Assert.That(report.Message, Does.Contain("roll never varied"));
    }

    [Test]
    public void Fit_TooFewSamples_IsDeterminateButNotAccepted()
    {
        var report = AntennaOffsetHeightCalibrator.Fit(Synthesize(40, seed: 5));

        Assert.That(report.IsDeterminate, Is.True, report.Message);
        Assert.That(report.IsAccepted, Is.False);
        Assert.That(report.Warnings.Count, Is.EqualTo(2));
        Assert.That(string.Join(" | ", report.Warnings), Does.Contain("driving A→B"));
        Assert.That(string.Join(" | ", report.Warnings), Does.Contain("driving B→A"));
        Assert.That(report.Message, Does.Contain("gate"));
    }

    [Test]
    public void Fit_RollSpreadBelowTheGate_IsNotAccepted()
    {
        // 2° of roll instead of the required 3°.
        var report = AntennaOffsetHeightCalibrator.Fit(MonotoneRoll(samplesPerDirection: 200, rollSpreadDegrees: 2.0));

        Assert.That(report.IsDeterminate, Is.True, report.Message);
        Assert.That(report.IsAccepted, Is.False);
        Assert.That(string.Join(" | ", report.Warnings), Does.Contain("roll only moved"));
    }

    [Test]
    public void Fit_InvalidDirectionSign_Throws()
    {
        var samples = new[] { new AntennaCalibrationSample(0.5, 2.0, 0.05) };

        Assert.That(() => AntennaOffsetHeightCalibrator.Fit(samples), Throws.TypeOf<ArgumentOutOfRangeException>());
    }

    // ── Collector: gates, counters, fitting on request ───────────────────────

    private static AntennaCalibrationCollector StartedCollector(AntennaCalibrationOptions? options = null)
    {
        var collector = new AntennaCalibrationCollector(options);
        collector.Start();
        return collector;
    }

    [Test]
    public void Collector_AcceptsAWellFormedSample()
    {
        var collector = StartedCollector();

        Assert.That(collector.TryAdd(1, 2.5, 0.08, fixQuality: 4, speedMps: 1.2, out var rejection), Is.True, rejection);
        Assert.That(rejection, Is.Null);
        Assert.That(collector.Count, Is.EqualTo(1));
        Assert.That(collector.ForwardCount, Is.EqualTo(1));
        Assert.That(collector.ReverseCount, Is.EqualTo(0));
        Assert.That(collector.RollRangeDegrees, Is.EqualTo(0.0));
    }

    [TestCase(1, 3.0, 0.0, 5, 1.2, "RTK Fixed")]          // RTK float
    [TestCase(-1, 3.0, 0.0, 0, 1.2, "RTK Fixed")]         // no fix
    [TestCase(1, 3.0, 0.0, 4, 0.3, "below")]              // below the speed gate
    [TestCase(1, 3.0, 1.2, 4, 1.2, "off the line")]       // more than 1 m off
    [TestCase(-1, 3.0, -1.2, 4, 1.2, "off the line")]
    [TestCase(0, 3.0, 0.05, 4, 1.2, "must be +1 or -1")]  // bad direction sign
    [TestCase(1, double.NaN, 0.05, 4, 1.2, "non-finite")] // bad roll
    public void Collector_RejectsSamplesOutsideTheGates(
        double directionSign,
        double rollDegrees,
        double deviationMeters,
        int fixQuality,
        double speedMps,
        string expectedReasonFragment)
    {
        var collector = StartedCollector();

        bool kept = collector.TryAdd(directionSign, rollDegrees, deviationMeters, fixQuality, speedMps, out var rejection);

        Assert.That(kept, Is.False);
        Assert.That(rejection, Does.Contain(expectedReasonFragment));
        Assert.That(collector.Count, Is.EqualTo(0));
    }

    [Test]
    public void Collector_DoesNotRecordBeforeStartOrAfterStop()
    {
        var collector = new AntennaCalibrationCollector();

        Assert.That(collector.TryAdd(1, 2.0, 0.05, 4, 1.0, out var rejection), Is.False);
        Assert.That(rejection, Is.EqualTo("not recording"));

        collector.Start();
        Assert.That(collector.TryAdd(1, 2.0, 0.05, 4, 1.0, out _), Is.True);

        collector.Stop();
        Assert.That(collector.TryAdd(1, 2.0, 0.05, 4, 1.0, out rejection), Is.False);
        Assert.That(rejection, Is.EqualTo("not recording"));
        Assert.That(collector.Count, Is.EqualTo(1), "a stopped recording keeps what it has");
    }

    [Test]
    public void Collector_RejectsOnceTheBufferIsFull()
    {
        var collector = StartedCollector(new AntennaCalibrationOptions { MaxSamples = 3 });

        for (int i = 0; i < 3; i++)
            Assert.That(collector.TryAdd(1, 2.0, 0.05, 4, 1.0, out _), Is.True);

        Assert.That(collector.TryAdd(1, 2.0, 0.05, 4, 1.0, out var rejection), Is.False);
        Assert.That(rejection, Does.Contain("buffer is full"));
        Assert.That(collector.Count, Is.EqualTo(3));
    }

    [Test]
    public void Collector_TracksCountersAndRollRange()
    {
        var collector = StartedCollector();

        collector.TryAdd(1, -3.0, 0.05, 4, 1.0, out _);
        collector.TryAdd(1, 1.5, 0.06, 4, 1.0, out _);
        collector.TryAdd(-1, 4.0, 0.04, 4, 1.0, out _);

        Assert.That(collector.Count, Is.EqualTo(3));
        Assert.That(collector.ForwardCount, Is.EqualTo(2));
        Assert.That(collector.ReverseCount, Is.EqualTo(1));
        Assert.That(collector.RollRangeDegrees, Is.EqualTo(7.0).Within(1e-9));
        Assert.That(collector.IsCollecting, Is.True);
    }

    [Test]
    public void Collector_Reset_ClearsEverythingAndStops()
    {
        var collector = StartedCollector();
        collector.TryAdd(1, 2.0, 0.05, 4, 1.0, out _);
        collector.TryAdd(-1, -2.0, 0.06, 4, 1.0, out _);

        collector.Reset();

        Assert.That(collector.Count, Is.EqualTo(0));
        Assert.That(collector.ForwardCount, Is.EqualTo(0));
        Assert.That(collector.ReverseCount, Is.EqualTo(0));
        Assert.That(collector.RollRangeDegrees, Is.EqualTo(0.0));
        Assert.That(collector.IsCollecting, Is.False);
        Assert.That(collector.Fit().IsDeterminate, Is.False);
    }

    [Test]
    public void Collector_FitUsesOnlyAcceptedSamples()
    {
        var collector = StartedCollector();
        var samples = Realistic(samplesPerDirection: 300, seed: 11);
        int rejected = 0;

        for (int i = 0; i < samples.Length; i++)
        {
            var sample = samples[i];
            // Every tenth cycle the fix drops out of RTK — those must not reach the fit.
            bool kept = collector.TryAdd(
                sample.DirectionSign, sample.RollDegrees, sample.LateralDeviationMeters,
                fixQuality: i % 10 == 0 ? 5 : 4, speedMps: 1.5, out _);
            if (!kept) rejected++;
        }

        var report = collector.Fit();

        Assert.That(rejected, Is.EqualTo(60));
        Assert.That(collector.Count, Is.EqualTo(samples.Length - rejected));
        Assert.That(report.IsAccepted, Is.True, report.Message);
        Assert.That(report.Fit!.Value.SampleCount, Is.EqualTo(samples.Length - rejected));
        Assert.That(report.Fit!.Value.ForwardSampleCount, Is.EqualTo(collector.ForwardCount));
        Assert.That(report.Fit!.Value.ReverseSampleCount, Is.EqualTo(collector.ReverseCount));
        Assert.That(report.Fit!.Value.LateralOffsetMeters, Is.EqualTo(TrueOffset).Within(OffsetTolerance));
        Assert.That(report.Fit!.Value.HeightAboveRollAxisMeters, Is.EqualTo(TrueHeight).Within(HeightTolerance));
    }

    // ── Diagnostic report (run on demand) ───────────────────────────────────

    [Test]
    [Explicit("Diagnostic dump — prints the fits for the seeded noisy runs.")]
    public void Report()
    {
        foreach (int seed in new[] { 1, 7, 42, 1234, 20261031 })
        {
            var fit = AntennaOffsetHeightCalibrator.Fit(Realistic(300, seed)).Fit!.Value;
            double offsetErrorMm = (fit.LateralOffsetMeters - TrueOffset) * 1000.0;
            double heightErrorMm = (fit.HeightAboveRollAxisMeters - TrueHeight) * 1000.0;
            TestContext.Out.WriteLine(FormattableString.Invariant(
                $"seed {seed,9}: offset {fit.LateralOffsetMeters:F4} m (err {offsetErrorMm:F1} mm), height {fit.HeightAboveRollAxisMeters:F4} m (err {heightErrorMm:F0} mm), se {fit.LateralOffsetStdDevMeters * 1000.0:F1}/{fit.HeightStdDevMeters * 1000.0:F1} mm, RMS {fit.RmsMeters * 1000.0:F1} mm"));
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

            // Noise-free: the numbers are exactly 0.070 / 2.850 / 0.030.
            var clean = AntennaOffsetHeightCalibrator.Fit(Synthesize(150, seed: 2));
            Assert.That(clean.Message, Does.Contain("0.070"));
            Assert.That(clean.Message, Does.Contain("2.850"));
            Assert.That(clean.Message, Does.Not.Contain("0,070"));

            // A gate warning that carries decimals (roll spread 2.00° < 3.0°).
            string warnings = string.Join(
                " | ",
                AntennaOffsetHeightCalibrator.Fit(MonotoneRoll(200, rollSpreadDegrees: 2.0)).Warnings);
            Assert.That(warnings, Does.Contain("2.00"));
            Assert.That(warnings, Does.Not.Contain("2,00"));
        }
        finally
        {
            CultureInfo.CurrentCulture = previous;
        }
    }
}


