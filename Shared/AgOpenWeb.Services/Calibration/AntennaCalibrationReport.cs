// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;
using System.Collections.Generic;

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// The three fitted unknowns plus their uncertainties. Produced by
/// <see cref="AntennaOffsetHeightCalibrator.Fit(System.Collections.Generic.IReadOnlyList{AntennaCalibrationSample}, AntennaCalibrationOptions)"/>.
/// </summary>
public readonly struct AntennaCalibrationFit
{
    /// <summary>Antenna lateral offset in metres, positive = right of the centreline.
    /// This is <c>VehicleConfig.AntennaOffset</c>.</summary>
    public double LateralOffsetMeters { get; init; }

    /// <summary>Antenna height above the point the vehicle rolls about, in metres.
    /// <c>AntennaToPivotTransform</c> applies <c>h·sin(roll)</c> with
    /// <c>VehicleConfig.AntennaHeight</c>, so this is the value that setting expects.</summary>
    public double HeightAboveRollAxisMeters { get; init; }

    /// <summary>Where the recorded pass sat relative to the line, in metres. A nuisance
    /// parameter — it is what the two-direction recording removes from the other two.</summary>
    public double LineOffsetMeters { get; init; }

    /// <summary>Standard error of <see cref="LateralOffsetMeters"/> (metres), NaN when the
    /// residual degrees of freedom ran out.</summary>
    public double LateralOffsetStdDevMeters { get; init; }

    /// <summary>Standard error of <see cref="HeightAboveRollAxisMeters"/> (metres).</summary>
    public double HeightStdDevMeters { get; init; }

    /// <summary>Standard error of <see cref="LineOffsetMeters"/> (metres).</summary>
    public double LineOffsetStdDevMeters { get; init; }

    /// <summary>RMS of the fit residuals in metres.</summary>
    public double RmsMeters { get; init; }

    /// <summary>Samples used with <c>DirectionSign == +1</c>.</summary>
    public int ForwardSampleCount { get; init; }

    /// <summary>Samples used with <c>DirectionSign == −1</c>.</summary>
    public int ReverseSampleCount { get; init; }

    /// <summary>Peak-to-peak roll covered by the samples, in degrees.</summary>
    public double RollRangeDegrees { get; init; }

    /// <summary>Total samples used.</summary>
    public int SampleCount { get; init; }
}

/// <summary>
/// Outcome of an antenna calibration fit: whether the unknowns could be separated
/// (<see cref="IsDeterminate"/>), whether every gate passed (<see cref="IsAccepted"/>)
/// and, when they could, the values themselves.
/// The service layer never writes settings on its own — the operator applies the fit.
/// </summary>
public sealed class AntennaCalibrationReport
{
    /// <summary>True when the normal equations could be solved: both driving
    /// directions present, roll varying, and the smallest pivot above the gate.</summary>
    public bool IsDeterminate { get; init; }

    /// <summary>True when the fit is determinate, both directions have enough samples
    /// and the roll range gate passed — i.e. <see cref="Warnings"/> is empty.</summary>
    public bool IsAccepted { get; init; }

    /// <summary>The fitted values; <c>null</c> when <see cref="IsDeterminate"/> is false.</summary>
    public AntennaCalibrationFit? Fit { get; init; }

    /// <summary>Human-readable reasons the fit cannot be accepted yet (empty when it can).
    /// Always populated for a non-determinate fit.</summary>
    public IReadOnlyList<string> Warnings { get; init; } = Array.Empty<string>();

    /// <summary>One-line summary for the panel/console (invariant number formatting).</summary>
    public string Message { get; init; } = string.Empty;
}
