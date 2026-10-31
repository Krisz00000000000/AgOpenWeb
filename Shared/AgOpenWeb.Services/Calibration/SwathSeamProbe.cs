// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// Measures the seam between the pass being driven and the neighbouring pass already worked
/// (algorithm 5's input). It probes the coverage outward from one edge of the tool's swath
/// and reads the covered / uncovered pattern back:
///
/// <code>
///        our swath        neighbour pass
///   ...|-------------| gap |-------------|...
///   0                 w/2   e             w
/// </code>
///
/// Walking outward from our edge, the first uncovered distance is
/// <list type="bullet">
/// <item>≈ 0 — nothing beside us: the neighbouring pass is not there, or the probe side is
/// the unworked side → no measurement (<c>null</c>);</item>
/// <item>small, and covered again after it → that run <b>is the gap</b> (positive deviation);</item>
/// <item>≈ a full tool width, with nothing covered after it → the neighbour overlaps us and
/// the union ends at <c>width + e</c>, so <c>e = distance − width</c> (negative = overlap).</item>
/// </list>
///
/// Pure function of a coverage predicate, so it is unit-testable without a coverage service.
/// </summary>
public static class SwathSeamProbe
{
    /// <summary>Probe resolution, metres.</summary>
    public const double DefaultStepMeters = 0.05;

    /// <summary>
    /// How far outside the swath edge to look, metres. A neighbouring pass is one tool width
    /// away, and an <b>overlap</b> only reads when the neighbour's far edge is inside the probe:
    /// the union then runs nearly a full width past our edge. Hence more than a wide tool.
    /// </summary>
    public const double DefaultMaxDistanceMeters = 8.0;

    /// <summary>
    /// The seam deviation on one side of the swath, in metres: positive = a gap was left,
    /// negative = the passes overlap, <c>null</c> = no neighbouring pass on that side.
    /// </summary>
    /// <param name="isCovered">Whether a world point has been worked.</param>
    /// <param name="easting">The tool's centre, easting.</param>
    /// <param name="northing">The tool's centre, northing.</param>
    /// <param name="headingRadians">The tool's heading (AgOpenWeb: <c>atan2(east, north)</c>).</param>
    /// <param name="widthMeters">The tool's working width.</param>
    /// <param name="side">+1 = the right of the heading, −1 = the left.</param>
    /// <param name="stepMeters">Probe resolution.</param>
    /// <param name="maxDistanceMeters">How far outside the edge to look.</param>
    public static double? MeasureSide(
        Func<double, double, bool> isCovered,
        double easting,
        double northing,
        double headingRadians,
        double widthMeters,
        int side,
        double stepMeters = DefaultStepMeters,
        double maxDistanceMeters = DefaultMaxDistanceMeters)
    {
        ArgumentNullException.ThrowIfNull(isCovered);
        if (!(widthMeters > 0.0) || !(stepMeters > 0.0)) return null;
        if (side != 1 && side != -1) return null;

        double halfWidth = widthMeters / 2.0;

        // Right of the heading in AgOpenWeb's convention (east = sin(h), north = cos(h)).
        double rightEast = Math.Cos(headingRadians);
        double rightNorth = -Math.Sin(headingRadians);
        double offsetEast = side * rightEast;
        double offsetNorth = side * rightNorth;

        double firstUncovered = double.NaN;
        bool coveredAfterUncovered = false;
        double coveredAgainAt = double.NaN;

        for (double distance = stepMeters; distance <= maxDistanceMeters; distance += stepMeters)
        {
            double probeE = easting + (offsetEast * (halfWidth + distance));
            double probeN = northing + (offsetNorth * (halfWidth + distance));
            bool covered = isCovered(probeE, probeN);

            if (double.IsNaN(firstUncovered))
            {
                if (!covered) firstUncovered = distance;
                continue;
            }

            if (covered)
            {
                coveredAfterUncovered = true;
                coveredAgainAt = distance;
                break;
            }
        }

        // Never found an edge: either the field is covered all the way out (nothing to read)
        // or the probe never left the swath.
        if (double.IsNaN(firstUncovered)) return null;

        // Covered again past the hole: the neighbour's strip starts there, so the gap between
        // our swath edge and it is that distance — resolved to the probe step. (Returning the
        // first uncovered step would report ~0 for every gap, which is no measurement at all.)
        if (coveredAfterUncovered && coveredAgainAt < widthMeters) return coveredAgainAt;

        // A full-width stretch before the first hole: the neighbour overlaps by that much.
        if (firstUncovered >= widthMeters * 0.9) return firstUncovered - widthMeters;

        // A small hole with nothing beyond it: the neighbouring pass is not on this side.
        return null;
    }

    /// <summary>
    /// The seam on whichever side has a neighbouring pass, or <c>null</c> when neither side
    /// does. When both sides report one (a narrow pass in a fully worked block), the smaller
    /// magnitude wins — that is the seam the operator is working against.
    /// </summary>
    public static double? Measure(
        Func<double, double, bool> isCovered,
        double easting,
        double northing,
        double headingRadians,
        double widthMeters,
        double stepMeters = DefaultStepMeters,
        double maxDistanceMeters = DefaultMaxDistanceMeters)
    {
        double? right = MeasureSide(
            isCovered, easting, northing, headingRadians, widthMeters, 1, stepMeters, maxDistanceMeters);
        double? left = MeasureSide(
            isCovered, easting, northing, headingRadians, widthMeters, -1, stepMeters, maxDistanceMeters);

        if (right is null) return left;
        if (left is null) return right;
        return Math.Abs(right.Value) <= Math.Abs(left.Value) ? right : left;
    }
}