// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;
using AgOpenWeb.Models.Base;

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// Geometry the calibration algorithms sample with. Signed perpendicular distance is
/// <b>positive to the right of the A→B direction</b> — the same convention as
/// <c>TrackGuidanceOutput.CrossTrackError</c> before the pipeline flips it for a
/// reverse-direction pass, and the one <see cref="AntennaCalibrationSample"/> documents.
///
/// The formula mirrors <c>GpsPipelineService.CalculatePerpendicularDistance</c> (which is
/// private to the cycle). Promoting that one helper and deleting this copy is the
/// follow-up once the pipeline is touched again; until then the duplicated lines are
/// covered by a test so the two cannot drift apart silently.
/// </summary>
public static class CalibrationGeometry
{
    /// <summary>
    /// Signed perpendicular distance from a point to <paramref name="track"/>, in metres,
    /// positive to the right of the track's recorded (A→B) direction. For a curve the
    /// nearest segment wins. Returns 0 for a track whose points are too close together.
    /// </summary>
    public static double SignedDistanceToTrack(Models.Track.Track track, double easting, double northing)
    {
        ArgumentNullException.ThrowIfNull(track);

        var points = track.Points;
        if (points.Count == 2)
            return SignedDistanceToSegment(points[0], points[1], easting, northing, out _);

        double bestDistance = double.MaxValue;
        double signed = 0;
        for (int i = 0; i < points.Count - 1; i++)
        {
            double candidate = SignedDistanceToSegment(points[i], points[i + 1], easting, northing, out double length);
            if (length < 0.01) continue;

            // The segment's unsigned distance decides which one is the closest; the signed
            // value from that same segment is what we return.
            var a = points[i];
            var b = points[i + 1];
            double dx = b.Easting - a.Easting;
            double dn = b.Northing - a.Northing;
            double t = Math.Clamp(((easting - a.Easting) * dx + (northing - a.Northing) * dn) / (length * length), 0, 1);
            double projE = a.Easting + (t * dx);
            double projN = a.Northing + (t * dn);
            double distance = Math.Sqrt(((easting - projE) * (easting - projE)) + ((northing - projN) * (northing - projN)));

            if (distance < bestDistance)
            {
                bestDistance = distance;
                signed = candidate;
            }
        }

        return signed;
    }

    private static double SignedDistanceToSegment(
        Vec3 a, Vec3 b, double easting, double northing, out double length)
    {
        double dx = b.Easting - a.Easting;
        double dn = b.Northing - a.Northing;
        length = Math.Sqrt((dx * dx) + (dn * dn));
        if (length < 0.01) return 0;

        // (E−aE)·dn − (N−aN)·dx, divided by the length: positive = right of A→B.
        return (((easting - a.Easting) * dn) - ((northing - a.Northing) * dx)) / length;
    }
}