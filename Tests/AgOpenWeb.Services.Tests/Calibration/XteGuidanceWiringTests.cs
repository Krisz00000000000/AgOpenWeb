// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using AgOpenWeb.Models.Base;
using AgOpenWeb.Models.Track;
using AgOpenWeb.Services.Calibration;
using AgOpenWeb.Services.Track;
using NUnit.Framework;

namespace AgOpenWeb.Services.Tests.Calibration;

/// <summary>
/// Algorithm 4's hook in the steering laws: with no integrator attached the output must be
/// exactly what it was, and a correction <c>c</c> must steer on the corrected error <c>e + c</c>
/// — i.e. a positive correction (the vehicle sits right of the line) aims further left.
/// </summary>
[TestFixture]
public class XteGuidanceWiringTests
{
    private static TrackGuidanceInput Input(bool stanley = false) => new()
    {
        // Straight AB north: A at the origin, B 100 m north.
        Track = AgOpenWeb.Models.Track.Track.FromABLine("AB", new Vec3(0, 0, 0), new Vec3(0, 100, 0)),
        // The vehicle is 2 m right (east) of the line, heading north.
        PivotPosition = new Vec3(2, 50, 0),
        SteerPosition = new Vec3(2, 52.5, 0),
        UseStanley = stanley,
        Wheelbase = 2.5,
        MaxSteerAngle = 35,
        GoalPointDistance = 5,
        FixHeading = 0,
        AvgSpeed = 10,
        IsHeadingSameWay = true,
        FindGlobalNearest = true,
        StanleyHeadingErrorGain = stanley ? 2.0 : 0.0,
        StanleyDistanceErrorGain = stanley ? 0.5 : 0.0,
    };

    /// <summary>An integrator holding <paramref name="correctionMeters"/> of correction.</summary>
    private static XteIntegrator IntegratorWith(double correctionMeters)
    {
        var integrator = new XteIntegrator(new XteIntegratorOptions
        {
            Enabled = true,
            IntegralGainPerSecond = 0.05,
            MaxCorrectionMeters = 0.5,
        });
        // One long step: c = clamp(0.05 × e × 10 s) = e / 2.
        integrator.Update(new XteIntegratorInput(
            correctionMeters * 2.0, 10.0, 2.0, true, true, false));
        return integrator;
    }

    [Test]
    public void PurePursuit_WithADisabledIntegrator_IsUnchanged()
    {
        var service = new TrackGuidanceService();
        double before = service.CalculateGuidance(Input()).SteerAngle;

        service.SetXteIntegrator(new XteIntegrator()); // created disabled

        Assert.That(service.CalculateGuidance(Input()).SteerAngle, Is.EqualTo(before).Within(1e-9));
    }

    [Test]
    public void PurePursuit_SteersFurtherLeft_WhenTheCorrectionPushesLeft()
    {
        var service = new TrackGuidanceService();
        double plain = service.CalculateGuidance(Input()).SteerAngle;
        Assert.That(plain, Is.LessThan(0.0), "the vehicle starts right of the line, so it steers left");

        service.SetXteIntegrator(IntegratorWith(0.2));
        double corrected = service.CalculateGuidance(Input()).SteerAngle;

        Assert.That(corrected, Is.LessThan(plain),
            "c > 0 aims at a line shifted left, so the law steers further left");
    }

    [Test]
    public void Stanley_SteersFurtherLeft_WhenTheCorrectionPushesLeft()
    {
        var service = new TrackGuidanceService();
        double plain = service.CalculateGuidance(Input(stanley: true)).SteerAngle;

        service.SetXteIntegrator(IntegratorWith(0.2));
        double corrected = service.CalculateGuidance(Input(stanley: true)).SteerAngle;

        Assert.That(corrected, Is.LessThan(plain));
    }

    [Test]
    public void Detaching_RestoresThePlainLaw()
    {
        var service = new TrackGuidanceService();
        double plain = service.CalculateGuidance(Input()).SteerAngle;

        service.SetXteIntegrator(IntegratorWith(0.2));
        double corrected = service.CalculateGuidance(Input()).SteerAngle;
        Assert.That(corrected, Is.Not.EqualTo(plain).Within(1e-6));

        service.SetXteIntegrator(null);
        Assert.That(service.CalculateGuidance(Input()).SteerAngle, Is.EqualTo(plain).Within(1e-9));
    }

    [Test]
    public void CorrectionOfZero_ChangesNothing()
    {
        var service = new TrackGuidanceService();
        double plain = service.CalculateGuidance(Input()).SteerAngle;

        service.SetXteIntegrator(new XteIntegrator(new XteIntegratorOptions { Enabled = true }));

        Assert.That(service.CalculateGuidance(Input()).SteerAngle, Is.EqualTo(plain).Within(1e-9));
    }
}