// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// The drift indicator's traffic light. Green <see cref="Ok"/>, amber <see cref="Watch"/>,
/// red <see cref="Alarm"/>; the panel maps these straight onto colours.
/// </summary>
public enum PassDriftStatus
{
    /// <summary>Nothing recorded yet.</summary>
    Idle = 0,

    /// <summary>Inside half the threshold — carry on.</summary>
    Ok = 1,

    /// <summary>Between half the threshold and the threshold: worth watching, not alarming.</summary>
    Watch = 2,

    /// <summary>The smoothed deviation stayed past the threshold for the required number of
    /// consecutive passes — check the antenna and roll calibration.</summary>
    Alarm = 3,
}

/// <summary>One pass as the watcher recorded it, for the per-field log.</summary>
/// <param name="Index">1-based pass number since the last reset.</param>
/// <param name="DirectionSign">+1 driving the track's A→B order, −1 driving it B→A.</param>
/// <param name="DeviationMeters">The pass's signed deviation from the neighbouring seam.</param>
/// <param name="SmoothedMeters">The smoothed deviation after this pass.</param>
/// <param name="Status">The indicator after this pass.</param>
public readonly record struct PassDriftRecord(
    int Index,
    double DirectionSign,
    double DeviationMeters,
    double SmoothedMeters,
    PassDriftStatus Status);

/// <summary>What the watcher returns for one pass.</summary>
public readonly struct PassDriftUpdate
{
    /// <summary>The traffic light after this pass.</summary>
    public PassDriftStatus Status { get; init; }

    /// <summary>The smoothed deviation after this pass, metres.</summary>
    public double SmoothedMeters { get; init; }

    /// <summary>The raw deviation of this pass, metres.</summary>
    public double DeviationMeters { get; init; }

    /// <summary>How many passes in a row have been past the threshold.</summary>
    public int ConsecutiveOverThreshold { get; init; }

    /// <summary>True on the pass the alarm fires (and on every pass after it that stays past
    /// the threshold).</summary>
    public bool IsAlarming => Status == PassDriftStatus.Alarm;

    /// <summary>
    /// Diagnostic hint, not a diagnosis: the previous pass was driven the other way and its
    /// deviation had the <b>same</b> sign as this one. The antenna/roll model predicts that a
    /// direction-dependent error flips sign with the driving direction, so a same-signed pair
    /// suggests side-slip, wind, implement pull or the steering rather than the calibration.
    /// The watcher cannot tell which, which is why the panel only ever says "suspected".
    /// </summary>
    public bool SameSignAcrossDirections { get; init; }
}
