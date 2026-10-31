// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;
using AgOpenWeb.Models.Base;
using AgOpenWeb.Services.Calibration;
using NUnit.Framework;
using TrackModel = AgOpenWeb.Models.Track.Track;

namespace AgOpenWeb.Services.Tests.Calibration;

/// <summary>
/// The calibration sampling geometry: the signed distance must mean what the fit assumes
/// (positive to the right of the track's A→B direction), and must agree with the pipeline's
/// own cross-track convention — a flipped sign here would silently swap "antenna to the
/// right" with "line to the left" and the proposal would be wrong by 2·a.
/// </summary>
[TestFixture]
public class CalibrationGeometryTests
{
    private static TrackModel NorthLine() =>
        TrackModel.FromABLine("ab", new Vec3(0, 0, 0), new Vec3(0, 10, 0));

    [Test]
    public void EastOfANorthBoundLine_IsPositive()
    {
        Assert.That(CalibrationGeometry.SignedDistanceToTrack(NorthLine(), 1.0, 5.0),
            Is.EqualTo(1.0).Within(1e-9), "right of A→B (east of a north-bound line) is positive");
    }

    [Test]
    public void WestOfANorthBoundLine_IsNegative()
    {
        Assert.That(CalibrationGeometry.SignedDistanceToTrack(NorthLine(), -0.25, 5.0),
            Is.EqualTo(-0.25).Within(1e-9));
    }

    [Test]
    public void DrivingTheLineTheOtherWay_FlippedTheSign()
    {
        // Same physical point, recorded direction reversed: the sign must flip, which is
        // exactly what the direction sign (±1) cancels in the fit's model.
        var reversed = TrackModel.FromABLine("ba", new Vec3(0, 10, 0), new Vec3(0, 0, 0));
        Assert.That(CalibrationGeometry.SignedDistanceToTrack(reversed, 1.0, 5.0),
            Is.EqualTo(-1.0).Within(1e-9));
    }

    [Test]
    public void PointOnTheLine_IsZero()
    {
        Assert.That(CalibrationGeometry.SignedDistanceToTrack(NorthLine(), 0.0, 4.2),
            Is.EqualTo(0.0).Within(1e-9));
    }

    [Test]
    public void Curve_ReturnsTheNearestSegmentsSign()
    {
        // The same north-bound line, but as a three-point curve whose second segment bends
        // east: a point 1 m east of the southern half is still +1.
        var curve = TrackModel.FromCurve("curve",
            [new Vec3(0, 0, 0), new Vec3(0, 5, 0), new Vec3(5, 10, 0)]);

        Assert.That(CalibrationGeometry.SignedDistanceToTrack(curve, 1.0, 2.0),
            Is.EqualTo(1.0).Within(0.2), "right of the first (north-bound) segment");
        Assert.That(CalibrationGeometry.SignedDistanceToTrack(curve, 1.0, 2.0), Is.GreaterThan(0.0));
    }

    [Test]
    public void DegenerateTrack_ReturnsZeroInsteadOfNaN()
    {
        var point = TrackModel.FromABLine("degenerate", new Vec3(1, 1, 0), new Vec3(1, 1, 0));
        Assert.That(CalibrationGeometry.SignedDistanceToTrack(point, 5.0, 5.0), Is.EqualTo(0.0));
    }

    [Test]
    public void NullTrack_Throws()
    {
        Assert.That(() => CalibrationGeometry.SignedDistanceToTrack(null!, 0, 0),
            Throws.TypeOf<ArgumentNullException>());
    }
}
