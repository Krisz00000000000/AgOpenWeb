// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// Thresholds for the pass-drift watcher (algorithm 5). The plan's starting values; on a
/// real field the threshold has to be matched to the implement width and the overlap that
/// is acceptable.
/// </summary>
public sealed class PassDriftOptions
{
    /// <summary>Shared defaults — one instance, never mutated.</summary>
    public static PassDriftOptions Default { get; } = new();

    /// <summary>Exponential smoothing factor α for the per-pass deviation (0–1).</summary>
    public double SmoothingAlpha { get; init; } = 0.2;

    /// <summary>Alarm threshold T on the smoothed deviation, metres (4 cm).</summary>
    public double AlarmThresholdMeters { get; init; } = 0.04;

    /// <summary>
    /// How many consecutive updates the smoothed deviation must stay past the threshold
    /// before it counts as drift rather than one bad pass.
    /// </summary>
    public int ConsecutiveUpdatesToAlarm { get; init; } = 3;

    /// <summary>From this fraction of the threshold the indicator turns amber.</summary>
    public double WatchFraction { get; init; } = 0.5;

    /// <summary>Passes kept for the field log (oldest dropped first).</summary>
    public int MaxHistory { get; init; } = 500;
}
