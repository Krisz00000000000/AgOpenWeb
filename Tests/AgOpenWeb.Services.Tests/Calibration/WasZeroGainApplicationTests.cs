// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using AgOpenWeb.Services.Calibration;
using NUnit.Framework;

namespace AgOpenWeb.Services.Tests.Calibration;

/// <summary>
/// Algorithm 2's Apply: what the fitted WAS zero (and gain) mean in the steer module's own
/// calibration — <c>wasOffset</c> in counts and <c>countsPerDegree</c> — and which proposals
/// are refused outright.
/// </summary>
[TestFixture]
public class WasZeroGainApplicationTests
{
    private static WasCalibrationFit Fit(double zero, double gain = 1.0, bool gainObservable = false)
        => new()
        {
            MeasuredSteerAngleOffsetDegrees = zero,
            Gain = gain,
            GainObservable = gainObservable,
        };

    [Test]
    public void ZeroOnly_MovesTheModuleOffsetAndLeavesTheScale()
    {
        bool ok = WasZeroGainApplication.TryCompute(0, 100.0, Fit(-1.5), out var settings);

        Assert.That(ok, Is.True);
        Assert.That(settings.WasOffset, Is.EqualTo(150), "0 − round(−1.5° × 100 counts/°)");
        Assert.That(settings.CountsPerDegree, Is.EqualTo(100.0));
        Assert.That(settings.GainApplied, Is.False);
        Assert.That(settings.WasOffsetDeltaCounts, Is.EqualTo(150));
        Assert.That(settings.Message, Does.Contain("150"));
    }

    [Test]
    public void ObservableGain_ScalesCountsPerDegree()
    {
        bool ok = WasZeroGainApplication.TryCompute(
            0, 100.0, Fit(0.0, gain: 0.8, gainObservable: true), out var settings);

        Assert.That(ok, Is.True);
        Assert.That(settings.CountsPerDegree, Is.EqualTo(125.0).Within(1e-9),
            "the sensor over-reads by 20%, so one degree needs 25% more counts");
        Assert.That(settings.GainApplied, Is.True);
    }

    [Test]
    public void Zero_BelongsToTheOldScale_WhenTheGainIsApplied()
    {
        // wasOffset' = wasOffset − x₀·countsPerDegree is independent of the angle scale; the
        // new countsPerDegree only rescales what the module reports afterwards.
        bool ok = WasZeroGainApplication.TryCompute(
            120, 90.0, Fit(-2.0, gain: 1.25, gainObservable: true), out var settings);

        Assert.That(ok, Is.True);
        Assert.That(settings.WasOffset, Is.EqualTo(120 + 180));
        Assert.That(settings.CountsPerDegree, Is.EqualTo(72.0).Within(1e-9));
    }

    [Test]
    public void NegativeGain_IsRefused_WithTheDirectionHint()
    {
        bool ok = WasZeroGainApplication.TryCompute(
            0, 100.0, Fit(0.0, gain: -1.2, gainObservable: true), out var settings);

        Assert.That(ok, Is.False);
        Assert.That(settings.Message, Does.Contain("invert WAS"));
        Assert.That(settings.CountsPerDegree, Is.EqualTo(100.0),
            "a refused proposal touches nothing");
        Assert.That(settings.WasOffset, Is.EqualTo(0));
    }

    [Test]
    public void OffsetPastTheLimit_IsRefused()
    {
        bool ok = WasZeroGainApplication.TryCompute(0, 100.0, Fit(-50.0), out var settings);

        Assert.That(ok, Is.False, "0 − round(−50° × 100) = 5000 counts, past AgOpenGPS's ±3900");
        Assert.That(settings.Message, Does.Contain("3900"));
    }

    [Test]
    public void MissingCountsPerDegree_IsRefused()
    {
        bool ok = WasZeroGainApplication.TryCompute(0, 0.0, Fit(1.0), out var settings);

        Assert.That(ok, Is.False);
        Assert.That(settings.Message, Does.Contain("counts per degree"));
    }

    [Test]
    public void NonFiniteZero_IsRefused()
    {
        bool ok = WasZeroGainApplication.TryCompute(
            0, 100.0, Fit(double.NaN), out var settings);

        Assert.That(ok, Is.False);
        Assert.That(settings.Message, Does.Contain("no usable zero"));
    }
}