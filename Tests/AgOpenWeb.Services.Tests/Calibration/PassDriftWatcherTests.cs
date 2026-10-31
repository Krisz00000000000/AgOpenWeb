// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;
using AgOpenWeb.Services.Calibration;
using NUnit.Framework;

namespace AgOpenWeb.Services.Tests.Calibration;

/// <summary>
/// Algorithm 5 — the pass-drift watcher. The plan's simulation cases: a noisy series must
/// not alarm, a per-pass growing offset must, and one bad pass must not.
/// </summary>
[TestFixture]
public class PassDriftWatcherTests
{
    private static PassDriftOptions Options(double alpha = 0.2, double threshold = 0.04, int consecutive = 3)
        => new()
        {
            SmoothingAlpha = alpha,
            AlarmThresholdMeters = threshold,
            ConsecutiveUpdatesToAlarm = consecutive,
        };

    [Test]
    public void Noise_DoesNotAlarm()
    {
        var watcher = new PassDriftWatcher(Options());
        var rng = new Random(1);

        for (int i = 0; i < 200; i++)
        {
            // ±1 cm of per-pass noise, alternating driving direction.
            double deviation = (rng.NextDouble() * 0.02) - 0.01;
            var update = watcher.Add(deviation, i % 2 == 0 ? 1.0 : -1.0);
            Assert.That(update.IsAlarming, Is.False, $"pass {i + 1} alarmed on noise");
        }

        Assert.That(watcher.Status, Is.Not.EqualTo(PassDriftStatus.Alarm));
    }

    [Test]
    public void GrowingOffset_AlarmsWithinAFewPassesButNotOnTheFirst()
    {
        // The plan's starting parameters for this case: α = 0.3, T = 4 cm, n = 3.
        var watcher = new PassDriftWatcher(Options(alpha: 0.3));

        int alarmedAt = 0;
        for (int i = 1; i <= 20 && alarmedAt == 0; i++)
        {
            if (watcher.Add(i * 0.01).IsAlarming) alarmedAt = i;
        }

        Assert.That(alarmedAt, Is.GreaterThanOrEqualTo(4),
            "one or two passes alone must not count as drift");
        Assert.That(alarmedAt, Is.LessThanOrEqualTo(12),
            "a per-pass 1 cm drift should be flagged within a dozen passes");
    }

    [Test]
    public void SingleOutlierPass_DoesNotAlarm()
    {
        var watcher = new PassDriftWatcher(Options());
        for (int i = 0; i < 10; i++) watcher.Add(0.0);

        var spike = watcher.Add(0.30);
        Assert.That(spike.IsAlarming, Is.False, "one bad pass is not drift");
        Assert.That(spike.ConsecutiveOverThreshold, Is.EqualTo(1));

        Assert.That(watcher.Add(0.0).IsAlarming, Is.False);
        var settled = watcher.Add(0.0);
        Assert.That(settled.IsAlarming, Is.False);
        Assert.That(settled.ConsecutiveOverThreshold, Is.EqualTo(0));
    }

    [Test]
    public void Watch_IsAmberBetweenHalfTheThresholdAndTheThreshold()
    {
        var watcher = new PassDriftWatcher(Options());

        var update = watcher.Add(0.03);
        Assert.That(update.Status, Is.EqualTo(PassDriftStatus.Watch));
        Assert.That(update.SmoothedMeters, Is.EqualTo(0.03).Within(1e-12));
        Assert.That(update.DeviationMeters, Is.EqualTo(0.03));
    }

    [Test]
    public void SameSignAcrossDirections_IsFlaggedAsSuspectedSideSlip()
    {
        var watcher = new PassDriftWatcher(Options());

        var first = watcher.Add(0.03, 1.0);
        Assert.That(first.SameSignAcrossDirections, Is.False, "nothing to compare with yet");

        var back = watcher.Add(0.02, -1.0);
        Assert.That(back.SameSignAcrossDirections, Is.True,
            "opposite direction, same sign — the antenna/roll model does not explain it");

        var flipped = watcher.Add(-0.02, 1.0);
        Assert.That(flipped.SameSignAcrossDirections, Is.False,
            "opposite direction, opposite sign — what the antenna/roll model predicts");
    }

    [Test]
    public void History_KeepsBoundedPerPassRecordsAndResetClearsThem()
    {
        var watcher = new PassDriftWatcher(new PassDriftOptions { MaxHistory = 2 });

        watcher.Add(0.01);
        watcher.Add(0.02);
        watcher.Add(0.03);

        Assert.That(watcher.PassCount, Is.EqualTo(3));
        Assert.That(watcher.History.Count, Is.EqualTo(2));
        Assert.That(watcher.History[1].Index, Is.EqualTo(3));
        Assert.That(watcher.History[1].DeviationMeters, Is.EqualTo(0.03));

        watcher.Reset();
        Assert.That(watcher.PassCount, Is.EqualTo(0));
        Assert.That(watcher.History, Is.Empty);
        Assert.That(watcher.Status, Is.EqualTo(PassDriftStatus.Idle));
        Assert.That(watcher.SmoothedMeters, Is.EqualTo(0.0));
    }

    [Test]
    public void Add_RejectsNonFiniteDeviationsAndOddDirectionSigns()
    {
        var watcher = new PassDriftWatcher();

        Assert.That(() => watcher.Add(double.NaN),
            Throws.InstanceOf<ArgumentOutOfRangeException>());
        Assert.That(() => watcher.Add(0.01, 0.0),
            Throws.InstanceOf<ArgumentOutOfRangeException>());
        Assert.That(watcher.PassCount, Is.EqualTo(0));
    }
}