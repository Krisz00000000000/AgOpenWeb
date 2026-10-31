// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using AgOpenWeb.Services.Calibration;
using NUnit.Framework;

namespace AgOpenWeb.Services.Tests.Calibration;

/// <summary>
/// Algorithm 5's seam source: walking outwards from the swath edge and reading the covered /
/// uncovered pattern back. The signs and the three outcomes (no neighbour, gap, overlap) are
/// what the pass-drift watcher's thresholds are expressed in.
/// </summary>
[TestFixture]
public class SwathSeamProbeTests
{
    private const double Width = 6.0;      // tool width, so half = 3 m
    private const double Heading = 0.0;    // north: the right of the heading is +easting

    /// <summary>Everything west of <paramref name="x"/> is worked (a neighbour pass there).</summary>
    private static Func<double, double, bool> WorkedWestOf(double x)
        => (easting, _) => easting < x;

    [Test]
    public void NoNeighbour_OnEitherSide_MeasuresNothing()
    {
        // Nothing worked anywhere: both probes only ever see holes.
        Assert.That(
            SwathSeamProbe.Measure((_, _) => false, 0, 0, Heading, Width), Is.Null);
    }

    [Test]
    public void WorkedRightUpToTheEdge_MeasuresNothing()
    {
        // Our own swath (|easting| < 3) is covered and so is everything west of it: the left
        // probe never leaves the coverage, and the right probe finds only a hole with nothing
        // beyond it — not a neighbouring pass.
        Assert.That(
            SwathSeamProbe.Measure(WorkedWestOf(0.0), 0, 0, Heading, Width), Is.Null);
    }

    [Test]
    public void Gap_IsReportedAtTheDistanceTheNeighbourStarts()
    {
        // The neighbour's strip starts 0.6 m west of our left edge (easting −3.6).
        double? seam = SwathSeamProbe.MeasureSide(
            WorkedWestOf(-3.6), 0, 0, Heading, Width, side: -1);

        Assert.That(seam, Is.Not.Null);
        Assert.That(seam!.Value, Is.EqualTo(0.6).Within(2 * SwathSeamProbe.DefaultStepMeters));
        Assert.That(seam.Value, Is.GreaterThan(0.0), "a gap is positive");
    }

    [Test]
    public void Overlap_IsReportedNegative()
    {
        // The neighbour's strip overlaps our left edge by 0.1 m and runs one tool width out:
        // the union beside us therefore ends 5.9 m from our edge, which is width − 0.1.
        Func<double, double, bool> covered = (easting, _) => easting < -2.9 && easting > -8.9;

        double? seam = SwathSeamProbe.MeasureSide(covered, 0, 0, Heading, Width, side: -1);

        Assert.That(seam, Is.Not.Null);
        Assert.That(seam!.Value, Is.LessThan(0.0), "an overlap is negative");
        Assert.That(seam.Value, Is.EqualTo(-0.1).Within(0.15));
    }

    [Test]
    public void BothSides_ReportTheNearerSeam()
    {
        // A worked strip 0.5 m west of the left edge and one 0.2 m east of the right edge: the
        // right one is what the operator is actually working against.
        Func<double, double, bool> covered = (easting, _) => easting < -3.5 || easting > 3.2;

        double? seam = SwathSeamProbe.Measure(covered, 0, 0, Heading, Width);

        Assert.That(seam, Is.Not.Null);
        Assert.That(seam!.Value, Is.EqualTo(0.2).Within(2 * SwathSeamProbe.DefaultStepMeters));
    }

    [Test]
    public void Side_UsesTheRightOfTheHeading()
    {
        // Facing east (heading π/2) the right of the heading is south, i.e. −northing.
        Func<double, double, bool> covered = (_, northing) => northing < -3.5;

        double? seam = SwathSeamProbe.MeasureSide(
            covered, 0, 0, System.Math.PI / 2.0, Width, side: 1);

        Assert.That(seam, Is.Not.Null);
        Assert.That(seam!.Value, Is.EqualTo(0.5).Within(2 * SwathSeamProbe.DefaultStepMeters));
    }

    [Test]
    public void DegenerateInputs_MeasureNothing()
    {
        Assert.That(SwathSeamProbe.MeasureSide((_, _) => true, 0, 0, Heading, 0.0, 1), Is.Null);
        Assert.That(SwathSeamProbe.MeasureSide((_, _) => true, 0, 0, Heading, Width, 0), Is.Null);
    }
}