// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using AgOpenWeb.Models;
using AgOpenWeb.Models.Configuration;
using AgOpenWeb.Models.Pipeline;
using AgOpenWeb.Models.State;
using AgOpenWeb.Models.Timing;
using AgOpenWeb.Services.Calibration;
using NSubstitute;

namespace AgOpenWeb.ViewModels.Tests;

/// <summary>
/// The automatic calibration menu's wiring (algorithms 1, 2, 4 and 5): which algorithm a
/// <c>calibration.start|n</c> may start, what sampling does to the panel state, and that the
/// live ones attach/detach and record without writing any setting.
/// </summary>
[TestFixture]
public class CalibrationPanelTests
{
    private double _wheelbase;
    private int _wasOffset;
    private double _countsPerDegree;

    [SetUp]
    public void SetUp()
    {
        // The VM reads the shared store, so put back whatever the other tests left.
        _wheelbase = ConfigurationStore.Instance.Vehicle.Wheelbase;
        _wasOffset = ConfigurationStore.Instance.AutoSteer.WasOffset;
        _countsPerDegree = ConfigurationStore.Instance.AutoSteer.CountsPerDegree;
        ConfigurationStore.Instance.Vehicle.Wheelbase = 2.5;
        ConfigurationStore.Instance.AutoSteer.WasOffset = 0;
        ConfigurationStore.Instance.AutoSteer.CountsPerDegree = 100.0;
    }

    [TearDown]
    public void TearDown()
    {
        ConfigurationStore.Instance.Vehicle.Wheelbase = _wheelbase;
        ConfigurationStore.Instance.AutoSteer.WasOffset = _wasOffset;
        ConfigurationStore.Instance.AutoSteer.CountsPerDegree = _countsPerDegree;
        Clock.Reset();
    }

    [Test]
    public void Start_IgnoresUnknownAlgorithms_AndTheSteeringTest()
    {
        var vm = new MainViewModelBuilder().Build();

        vm.StartAutomaticCalibration(3); // the gated steering test
        vm.StartAutomaticCalibration(6); // sensor fusion, not started
        vm.StartAutomaticCalibration(42);

        Assert.That(vm.IsCalibrationRecording, Is.False);
        Assert.That(vm.State.Calibration.Algorithm, Is.EqualTo(0));
    }

    [Test]
    public void WasRecording_FitsTheZeroFromTheMeasuredAngle()
    {
        var builder = new MainViewModelBuilder();
        var vm = builder.Build();
        vm.StartAutomaticCalibration(2);

        const double wheelbase = 2.5;
        for (int i = 0; i < 400; i++)
        {
            double trueAngle = 8.0 * Math.Sin(i * 0.3);          // the wheels weave ±8°
            const double speed = 2.0;                             // m/s
            // Ackermann: the yaw rate that steer angle produces (left positive).
            double yawLeft = speed * Math.Tan(trueAngle * Math.PI / 180.0) / wheelbase;

            // The sensor reads 1.5° low: the zero it needs is −1.5°.
            builder.AutoSteerService.LastSteerData.Returns(
                new SteerModuleData(trueAngle - 1.5, 0, 0, false, false, false, false, 0));

            vm.ApplyGpsCycleResult(new GpsCycleResult
            {
                GpsValid = true,
                FixQuality = 4,
                Speed = speed,
                RollDegrees = 0,
                // The cycle's derived rate is positive when turning right; the collector's
                // convention is left positive.
                YawRateRadPerSecond = -yawLeft,
            });
        }

        var cal = vm.State.Calibration;
        Assert.That(cal.WasSamples, Is.EqualTo(400));
        Assert.That(cal.WasAccepted, Is.True, cal.WasMessage);
        Assert.That(cal.WasGainObservable, Is.True);
        Assert.That(cal.WasZeroDegrees, Is.EqualTo(-1.5).Within(0.2));
        Assert.That(cal.WasGain, Is.EqualTo(1.0).Within(0.05));
        Assert.That(cal.WasProposedOffset, Is.EqualTo(150).Within(15),
            "0 − round(−1.5° × 100 counts/°)");
        Assert.That(cal.WasProposedCountsPerDegree, Is.EqualTo(100.0).Within(1.0));
        Assert.That(cal.WasApplyValid, Is.True, cal.WasApplyMessage);
    }

    [Test]
    public void WasRecording_AnObservableGainScalesCountsPerDegree()
    {
        var builder = new MainViewModelBuilder();
        var vm = builder.Build();
        vm.StartAutomaticCalibration(2);

        const double wheelbase = 2.5;
        for (int i = 0; i < 400; i++)
        {
            double trueAngle = 8.0 * Math.Sin(i * 0.3);
            const double speed = 2.0;
            double yawLeft = speed * Math.Tan(trueAngle * Math.PI / 180.0) / wheelbase;

            // The sensor over-reads by 25% (gain 0.8), with no zero error.
            builder.AutoSteerService.LastSteerData.Returns(
                new SteerModuleData(trueAngle / 0.8, 0, 0, false, false, false, false, 0));

            vm.ApplyGpsCycleResult(new GpsCycleResult
            {
                GpsValid = true,
                FixQuality = 4,
                Speed = speed,
                RollDegrees = 0,
                YawRateRadPerSecond = -yawLeft,
            });
        }

        var cal = vm.State.Calibration;
        Assert.That(cal.WasAccepted, Is.True, cal.WasMessage);
        Assert.That(cal.WasGain, Is.EqualTo(0.8).Within(0.05));
        Assert.That(cal.WasGainApplied, Is.True);
        Assert.That(cal.WasProposedCountsPerDegree, Is.EqualTo(125.0).Within(6.0),
            "100 counts/° ÷ 0.8");
    }

    [Test]
    public void WasRecording_SaysWhyItCannotFit_WithoutAWheelbase()
    {
        ConfigurationStore.Instance.Vehicle.Wheelbase = 0.0;
        var vm = new MainViewModelBuilder().Build();
        vm.StartAutomaticCalibration(2);

        var cal = vm.State.Calibration;
        Assert.That(cal.WasDeterminate, Is.False);
        Assert.That(cal.WasApplyValid, Is.False);
        Assert.That(cal.WasMessage, Does.Contain("wheelbase"));
    }

    // ── Algorithm 4: the cross-track integrator ─────────────────────────────

    [Test]
    public void Integrator_IsAttachedWhileOn_AndDetachedWhenOff()
    {
        var builder = new MainViewModelBuilder();
        var vm = builder.Build();
        var clock = new TestClock();
        Clock.Set(clock);

        vm.StartAutomaticCalibration(4);
        Assert.That(vm.State.Calibration.XteEnabled, Is.True, "the switch is on");
        builder.TrackGuidanceService.Received(1)
            .SetXteIntegrator(Arg.Is<XteIntegrator>(i => i != null));

        for (int i = 0; i < 50; i++)
        {
            clock.AdvanceMs(100); // 10 Hz fixes
            vm.ApplyGpsCycleResult(IntegratorCycle(crossTrack: 0.2));
        }

        // 50 × 0.1 s at k_i = 0.05 1/s and e = 0.2 m → 0.05 × 0.2 × 5 s = 0.05 m.
        Assert.That(vm.State.Calibration.XteCorrectionMeters, Is.EqualTo(0.05).Within(0.01));
        Assert.That(vm.State.Calibration.XteMessage, Is.Not.Empty);

        vm.StopAutomaticCalibration();
        Assert.That(vm.State.Calibration.XteEnabled, Is.False);
        Assert.That(vm.State.Calibration.XteCorrectionMeters, Is.EqualTo(0.0));
        builder.TrackGuidanceService.Received(1).SetXteIntegrator(null);
    }

    [Test]
    public void Integrator_ResetsOnDisengage()
    {
        var vm = new MainViewModelBuilder().Build();
        var clock = new TestClock();
        Clock.Set(clock);

        vm.StartAutomaticCalibration(4);
        for (int i = 0; i < 20; i++)
        {
            clock.AdvanceMs(100);
            vm.ApplyGpsCycleResult(IntegratorCycle(crossTrack: 0.2));
        }

        Assert.That(vm.State.Calibration.XteCorrectionMeters, Is.GreaterThan(0.0));

        clock.AdvanceMs(100);
        vm.ApplyGpsCycleResult(
            IntegratorCycle(crossTrack: 0.2) with { AutoSteerDisengagedThisCycle = true });

        Assert.That(vm.State.Calibration.XteCorrectionMeters, Is.EqualTo(0.0),
            "a disengage starts the integrator from zero");
    }

    [Test]
    public void Integrator_DoesNotGrow_WithoutRtkOrAnEngagedSteer()
    {
        var vm = new MainViewModelBuilder().Build();
        var clock = new TestClock();
        Clock.Set(clock);

        vm.StartAutomaticCalibration(4);
        for (int i = 0; i < 20; i++)
        {
            clock.AdvanceMs(100);
            vm.ApplyGpsCycleResult(IntegratorCycle(crossTrack: 0.2) with
            {
                FixQuality = 2,
                IsAutoSteerEngaged = false,
            });
        }

        Assert.That(vm.State.Calibration.XteCorrectionMeters, Is.EqualTo(0.0));
    }

    // ── Algorithm 5: the pass-drift watcher ────────────────────────────────

    [Test]
    public void PassDrift_RecordsOneDeviationPerPass()
    {
        var builder = new MainViewModelBuilder();
        // The neighbouring pass west of our 6 m swath is worked, 0.6 m away.
        builder.CoverageMapService
            .IsPointCovered(Arg.Any<double>(), Arg.Any<double>())
            .Returns(ci => ci.ArgAt<double>(0) < -3.6);

        var vm = builder.Build();
        vm.StartAutomaticCalibration(5);
        Assert.That(vm.State.Calibration.DriftArmed, Is.True);

        for (int i = 0; i < 10; i++) vm.ApplyGpsCycleResult(DriftCycle(pass: 0));
        Assert.That(vm.State.Calibration.DriftPasses, Is.EqualTo(0),
            "a deviation is recorded when the pass ends");

        vm.ApplyGpsCycleResult(DriftCycle(pass: 1)); // pass 0 ends here

        var cal = vm.State.Calibration;
        Assert.That(cal.DriftPasses, Is.EqualTo(1));
        Assert.That(cal.DriftSmoothedMeters, Is.EqualTo(0.65).Within(0.15));
        Assert.That(cal.DriftStatus, Is.EqualTo((int)PassDriftStatus.Watch));
        Assert.That(cal.DriftSuspect, Is.False, "the first pass has nothing to compare with");
        Assert.That(cal.DriftMessage, Is.Not.Empty);
    }

    [Test]
    public void Reset_ClearsEveryAlgorithm()
    {
        var builder = new MainViewModelBuilder();
        var vm = builder.Build();

        builder.CoverageMapService
            .IsPointCovered(Arg.Any<double>(), Arg.Any<double>())
            .Returns(false);
        vm.StartAutomaticCalibration(5);
        vm.ApplyGpsCycleResult(DriftCycle(pass: 0));
        vm.StartAutomaticCalibration(4);
        vm.ResetAutomaticCalibration();

        var cal = vm.State.Calibration;
        Assert.That(vm.IsCalibrationRecording, Is.False);
        Assert.That(cal.Algorithm, Is.EqualTo(0));
        Assert.That(cal.DriftArmed, Is.False);
        Assert.That(cal.XteEnabled, Is.False);
        Assert.That(cal.WasSamples, Is.EqualTo(0));
        builder.TrackGuidanceService.Received().SetXteIntegrator(null);
    }

    private static GpsCycleResult IntegratorCycle(double crossTrack) => new()
    {
        GpsValid = true,
        FixQuality = 4,
        IsAutoSteerEngaged = true,
        Speed = 2.0,
        Guidance = new GuidanceSnapshot
        {
            HasGuidance = true,
            CrossTrackError = crossTrack,
            SteerAngle = 1.0,
            HowManyPathsAway = 0,
        },
    };

    private static GpsCycleResult DriftCycle(int pass) => new()
    {
        GpsValid = true,
        FixQuality = 4,
        IsAutoSteerEngaged = true,
        Speed = 2.0,
        IsToolPositionReady = true,
        ToolEasting = 0.0,
        ToolNorthing = 0.0,
        ToolHeadingRadians = 0.0,
        ToolWidth = 6.0,
        IsHeadingSameWay = true,
        Guidance = new GuidanceSnapshot
        {
            HasGuidance = true,
            SteerAngle = 0.5,
            HowManyPathsAway = pass,
        },
    };
}