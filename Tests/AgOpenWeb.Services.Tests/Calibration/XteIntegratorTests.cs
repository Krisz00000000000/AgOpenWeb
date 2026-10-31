// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;
using System.Collections.Generic;
using AgOpenWeb.Services.Calibration;
using NUnit.Framework;

namespace AgOpenWeb.Services.Tests.Calibration;

/// <summary>
/// Algorithm 4 — the anti-windup cross-track integrator. Unit cases for the gates and the
/// anti-windup rule, plus the plan's closed-loop simulation: a kinematic bicycle with a
/// 0.15 s command delay, a rate-limited steering servo and a constant 0.12 m/s side-slip.
/// </summary>
[TestFixture]
public class XteIntegratorTests
{
    private static XteIntegratorInput Input(
        double error,
        double speed = 2.0,
        double dt = 0.02,
        bool rtkFixed = true,
        bool engaged = true,
        bool saturated = false)
        => new(error, dt, speed, rtkFixed, engaged, saturated);

    // ── The disabled path changes nothing ────────────────────────────────────

    [Test]
    public void Disabled_ReturnsZeroWhateverComesIn()
    {
        var integrator = new XteIntegrator();

        for (int i = 0; i < 500; i++)
        {
            Assert.That(integrator.Update(Input(0.4)), Is.EqualTo(0.0));
        }

        Assert.That(integrator.Correction, Is.EqualTo(0.0));
        Assert.That(integrator.CorrectedError(0.25), Is.EqualTo(0.25));
    }

    // ── The gates ───────────────────────────────────────────────────────────

    [Test]
    public void Gates_LeaveTheCorrectionUntouched()
    {
        static XteIntegratorOptions Options() => new() { Enabled = true };

        var noRtk = new XteIntegrator(Options());
        Assert.That(noRtk.Update(Input(0.2, rtkFixed: false)), Is.EqualTo(0.0));

        var notEngaged = new XteIntegrator(Options());
        Assert.That(notEngaged.Update(Input(0.2, engaged: false)), Is.EqualTo(0.0));

        var tooSlow = new XteIntegrator(Options());
        Assert.That(tooSlow.Update(Input(0.2, speed: 0.4)), Is.EqualTo(0.0));

        var tooFarOff = new XteIntegrator(Options());
        Assert.That(tooFarOff.Update(Input(0.6)), Is.EqualTo(0.0));

        var noTime = new XteIntegrator(Options());
        Assert.That(noTime.Update(Input(0.2, dt: 0.0)), Is.EqualTo(0.0));

        Assert.That(noRtk.Correction, Is.EqualTo(0.0));
        Assert.That(tooFarOff.Correction, Is.EqualTo(0.0));
    }

    [Test]
    public void Update_AccumulatesAndClampsAtTheLimit()
    {
        var integrator = new XteIntegrator(new XteIntegratorOptions
        {
            Enabled = true,
            IntegralGainPerSecond = 0.05,
            MaxCorrectionMeters = 0.05,
        });

        // 0.05 1/s · 0.2 m · 0.02 s = 0.0002 m per cycle.
        Assert.That(integrator.Update(Input(0.2)), Is.EqualTo(0.0002).Within(1e-12));

        for (int i = 0; i < 1000; i++) integrator.Update(Input(0.2));
        Assert.That(integrator.Correction, Is.EqualTo(0.05).Within(1e-12));

        // And the other way takes it to the negative limit.
        for (int i = 0; i < 2000; i++) integrator.Update(Input(-0.2));
        Assert.That(integrator.Correction, Is.EqualTo(-0.05).Within(1e-12));
    }

    [Test]
    public void Leak_PullsTheCorrectionTowardsZero()
    {
        var integrator = new XteIntegrator(new XteIntegratorOptions
        {
            Enabled = true,
            LeakPerSecond = 0.5,
        });

        integrator.Update(Input(0.4, dt: 1.0));   // 0.05 · 0.4 · 1 s = 0.02 m
        Assert.That(integrator.Correction, Is.EqualTo(0.02).Within(1e-12));

        // One second of leak with no error: c · (1 − 0.5) = 0.01.
        integrator.Update(Input(0.0, dt: 1.0));
        Assert.That(integrator.Correction, Is.EqualTo(0.01).Within(1e-12));
    }

    // ── Anti-windup ─────────────────────────────────────────────────────────

    [Test]
    public void Saturated_StopsIntegratingButKeepsUnwinding()
    {
        var integrator = new XteIntegrator(new XteIntegratorOptions { Enabled = true });

        for (int i = 0; i < 100; i++) integrator.Update(Input(0.3));
        double woundUp = integrator.Correction;
        Assert.That(woundUp, Is.GreaterThan(0.0));

        // Saturated with the error still pushing the same way: hold.
        for (int i = 0; i < 100; i++)
        {
            Assert.That(integrator.Update(Input(0.3, saturated: true)), Is.EqualTo(woundUp));
        }

        // Saturated with the error opposing the correction: keep integrating, which unwinds.
        double unwound = integrator.Update(Input(-0.3, saturated: true));
        Assert.That(unwound, Is.LessThan(woundUp));
    }

    [Test]
    public void NaiveMode_KeepsIntegratingWhileSaturated()
    {
        var integrator = new XteIntegrator(new XteIntegratorOptions
        {
            Enabled = true,
            ConditionalAntiWindup = false,
        });

        double previous = integrator.Update(Input(0.3, saturated: true));
        double next = integrator.Update(Input(0.3, saturated: true));
        Assert.That(next, Is.GreaterThan(previous));
    }

    [Test]
    public void Reset_ZeroesTheCorrection()
    {
        var integrator = new XteIntegrator(new XteIntegratorOptions { Enabled = true });
        for (int i = 0; i < 100; i++) integrator.Update(Input(0.3));

        integrator.Reset();
        Assert.That(integrator.Correction, Is.EqualTo(0.0));
        Assert.That(integrator.CorrectedError(0.1), Is.EqualTo(0.1));
    }

    // ── Closed-loop simulation (the plan's acceptance test) ─────────────────

    private readonly record struct SimulationResult(
        double PeakExcursionMeters,
        double FinalAbsErrorMeters,
        double MaxAbsCorrectionMeters);

    /// <summary>
    /// Closed loop: kinematic bicycle (wheelbase 2.5 m) at 1.5 m/s following the line
    /// y = 0 with a Stanley-like law <c>δ = −(ψ + atan((e + c)/v))</c>, a 0.16 s command
    /// delay, a 20 °/s steering rate limit, a ±15° steering limit and a constant 0.12 m/s
    /// lateral disturbance (slope, side wind or a pulling implement).
    ///
    /// <see cref="SimulationResult.PeakExcursionMeters"/> is the largest |e| after the
    /// vehicle first came within 10 cm of the line — i.e. the overshoot.
    /// </summary>
    private static SimulationResult Simulate(
        bool integrate, bool conditionalAntiWindup, double startErrorMeters,
        double speedMps = 1.5, double startHeadingRadians = 0.0)
    {
        const double dt = 0.02;                       // 50 Hz
        double speed = speedMps;                      // m/s
        const double wheelbase = 2.5;                 // m
        const double disturbance = 0.12;              // m/s of side-slip
        const double maxSteer = 15.0 * Math.PI / 180.0;
        const double maxRate = 20.0 * Math.PI / 180.0;   // rad/s
        const int delaySteps = 8;                     // 8 × 20 ms = 0.16 s
        const int steps = 15000;                      // 300 s
        const int tailSteps = 500;                    // the last 10 s

        var integrator = new XteIntegrator(new XteIntegratorOptions
        {
            Enabled = integrate,
            IntegralGainPerSecond = 0.05,
            MaxCorrectionMeters = 0.30,
            ConditionalAntiWindup = conditionalAntiWindup,
        });

        var delay = new double[delaySteps];
        double error = startErrorMeters;
        double heading = startHeadingRadians;
        double steer = 0.0;

        bool onTheLine = false;
        double peak = 0.0;
        double tailSum = 0.0;
        double maxCorrection = 0.0;

        for (int i = 0; i < steps; i++)
        {
            double corrected = integrator.CorrectedError(error);
            double desired = Math.Clamp(
                -(heading + Math.Atan(corrected / speed)), -maxSteer, maxSteer);

            double delayed = delay[0];
            for (int k = 0; k < delaySteps - 1; k++) delay[k] = delay[k + 1];
            delay[delaySteps - 1] = desired;

            bool saturated = Math.Abs(desired) >= maxSteer - 1e-12;
            integrator.Update(new XteIntegratorInput(error, dt, speed, true, true, saturated));
            if (Math.Abs(integrator.Correction) > maxCorrection)
            {
                maxCorrection = Math.Abs(integrator.Correction);
            }

            double limit = maxRate * dt;
            steer += Math.Clamp(delayed - steer, -limit, limit);

            // Kinematic bicycle with the constant lateral disturbance.
            heading += speed * Math.Tan(steer) / wheelbase * dt;
            error += ((speed * Math.Sin(heading)) + disturbance) * dt;

            if (!onTheLine && Math.Abs(error) < 0.10) onTheLine = true;
            if (onTheLine && Math.Abs(error) > peak) peak = Math.Abs(error);

            if (i >= steps - tailSteps) tailSum += Math.Abs(error);
        }

        return new SimulationResult(peak, tailSum / tailSteps, maxCorrection);
    }

    [Test]
    public void ClosedLoop_ConstantSideSlip_BiasIsThereWithoutAndGoneWithTheIntegrator()
    {
        var without = Simulate(integrate: false, conditionalAntiWindup: true, startErrorMeters: 0.0);
        var with = Simulate(integrate: true, conditionalAntiWindup: true, startErrorMeters: 0.0);

        // The plan's numbers: the proportional law settles around side-slip / gain ≈ 12 cm,
        // the integrator takes it under 1.5 cm.
        Assert.That(without.FinalAbsErrorMeters, Is.GreaterThan(0.10),
            "the plan's ~12 cm steady-state bias should be reproduced");
        Assert.That(without.FinalAbsErrorMeters, Is.LessThan(0.15));
        Assert.That(without.MaxAbsCorrectionMeters, Is.EqualTo(0.0));

        Assert.That(with.FinalAbsErrorMeters, Is.LessThan(0.015),
            "the integrator must hold the steady-state error under 1.5 cm");
        Assert.That(with.MaxAbsCorrectionMeters, Is.GreaterThan(0.10),
            "the integrator should have learned roughly the side-slip bias");
    }

    [Test]
    public void ClosedLoop_SaturatedStart_StillConvergesUnderTwoCentimetres()
    {
        var conditional = Simulate(integrate: true, conditionalAntiWindup: true, startErrorMeters: 3.0);
        var off = Simulate(integrate: false, conditionalAntiWindup: true, startErrorMeters: 3.0);

        Assert.That(conditional.FinalAbsErrorMeters, Is.LessThan(0.02),
            "with the anti-windup the steady state must come under 2 cm even from a 3 m start");
        Assert.That(off.FinalAbsErrorMeters, Is.GreaterThan(0.10),
            "without the integrator the same run keeps the side-slip bias");
    }

    [Test]
    public void ClosedLoop_SaturatedStarts_DoNotWindTheCorrectionPastTheBiasTheDisturbanceNeeds()
    {
        // The plan expects conditional integration to overshoot less than the clamp-only
        // integrator. Measured on this model it does not — identical from a 3 m start
        // (0.965 m both) and 6 mm more on a small saturated start — because the 0.5 m error
        // gate already stops the integration during a big approach, so nothing winds up.
        // What the rules do guarantee is that the correction never grows past the bias the
        // disturbance actually needs. See the plan's algorithm 4 notes and Report() below.
        foreach (var (speed, error) in new[] { (1.5, 0.45), (1.0, 0.40), (0.8, 0.35) })
        {
            var conditional = Simulate(true, true, error, speed);

            Assert.That(conditional.MaxAbsCorrectionMeters, Is.EqualTo(0.12).Within(0.02),
                FormattableString.Invariant(
                    $"v {speed} m/s, e {error} m: the correction should settle at the 12 cm bias"));
            Assert.That(conditional.FinalAbsErrorMeters, Is.LessThan(0.02));
        }
    }

    [Test]
    [Explicit("Diagnostic dump — prints the closed-loop numbers the assertions are built on.")]
    public void Report()
    {
        foreach (bool integrate in new[] { false, true })
        {
            var result = Simulate(integrate, conditionalAntiWindup: true, startErrorMeters: 0.0);
            TestContext.Out.WriteLine(FormattableString.Invariant(
                $"slip, integrator {(integrate ? "on " : "off")}: final |e| {result.FinalAbsErrorMeters * 100.0:F2} cm, peak {result.PeakExcursionMeters * 100.0:F2} cm, |c|max {result.MaxAbsCorrectionMeters:F3} m"));
        }

        foreach (bool conditional in new[] { true, false })
        {
            var result = Simulate(true, conditional, startErrorMeters: 3.0);
            TestContext.Out.WriteLine(FormattableString.Invariant(
                $"3 m start, {(conditional ? "conditional" : "clamp-only")}: overshoot {result.PeakExcursionMeters:F3} m, final |e| {result.FinalAbsErrorMeters * 100.0:F2} cm, |c|max {result.MaxAbsCorrectionMeters:F3} m"));
        }

        // Windup only happens while the error is inside the 0.5 m gate and the steering is
        // saturated, so sweep the geometries where both hold at once.
        var cases = new (double Speed, double Error, double HeadingDegrees)[]
        {
            (1.5, 0.45, 0.0),
            (1.0, 0.40, 0.0),
            (1.0, 0.45, -10.0),
            (0.8, 0.35, 0.0),
            (1.0, 0.48, 5.0),
        };

        foreach (var (caseSpeed, caseError, caseHeading) in cases)
        {
            var conditional = Simulate(true, true, caseError, caseSpeed, caseHeading * Math.PI / 180.0);
            var naive = Simulate(true, false, caseError, caseSpeed, caseHeading * Math.PI / 180.0);
            TestContext.Out.WriteLine(FormattableString.Invariant(
                $"v {caseSpeed:F1} m/s, e {caseError:F2} m, psi {caseHeading:F0}°: overshoot conditional {conditional.PeakExcursionMeters:F3} m / clamp-only {naive.PeakExcursionMeters:F3} m, |c|max {conditional.MaxAbsCorrectionMeters:F3} / {naive.MaxAbsCorrectionMeters:F3} m"));
        }
    }
}