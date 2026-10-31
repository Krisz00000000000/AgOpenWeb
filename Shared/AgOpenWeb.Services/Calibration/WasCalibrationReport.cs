// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;
using System.Collections.Generic;

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// The fitted WAS zero and gain. Produced by
/// <see cref="WasZeroGainCalibrator.Fit(IReadOnlyList{WasCalibrationSample}, double, WasCalibrationOptions?)"/>.
/// </summary>
public readonly struct WasCalibrationFit
{
    /// <summary>The sensor reading that corresponds to driving straight, in degrees
    /// (positive = the module reads left). Applying it means subtracting it from the
    /// measured angle. This is <c>−b/g</c> of the model.</summary>
    public double MeasuredSteerAngleOffsetDegrees { get; init; }

    /// <summary>The fitted gain <c>g</c> — the inverse sensor scale. 1.0 when the sensor's
    /// scale is right, &gt;1 when the sensor under-reads the angle.</summary>
    public double Gain { get; init; }

    /// <summary>True when the steering moved far enough for <see cref="Gain"/> to be
    /// observable. When false the gain was <b>assumed</b> to be 1 and only the zero was
    /// fitted (the report carries a note saying so).</summary>
    public bool GainObservable { get; init; }

    /// <summary>Standard error of <see cref="MeasuredSteerAngleOffsetDegrees"/> (degrees),
    /// <c>NaN</c> when the residual degrees of freedom ran out.</summary>
    public double ZeroStdDevDegrees { get; init; }

    /// <summary>Standard error of <see cref="Gain"/>.</summary>
    public double GainStdDev { get; init; }

    /// <summary>RMS of the fit residuals in degrees.</summary>
    public double RmsDegrees { get; init; }

    /// <summary>Samples used.</summary>
    public int SampleCount { get; init; }

    /// <summary>Steer-angle range the samples covered, degrees (peak-to-peak).</summary>
    public double SteerRangeDegrees { get; init; }
}

/// <summary>
/// Outcome of a WAS zero / gain fit: whether the unknowns could be separated
/// (<see cref="IsDeterminate"/>), whether every gate passed (<see cref="IsAccepted"/>)
/// and, when they could, the values themselves.
/// Nothing here writes a setting; the operator applies the result through the ordinary
/// configuration (or module PGN) path — see the plan's note that the WAS zero may live in
/// the steer module rather than in AgOpenWeb's own config.
/// </summary>
public sealed class WasCalibrationReport
{
    /// <summary>True when the fit produced usable unknowns.</summary>
    public bool IsDeterminate { get; init; }

    /// <summary>True when the fit is determinate and every blocking gate passed
    /// (enough usable samples) — i.e. <see cref="Warnings"/> is empty.</summary>
    public bool IsAccepted { get; init; }

    /// <summary>The fitted values; <c>null</c> when <see cref="IsDeterminate"/> is false.</summary>
    public WasCalibrationFit? Fit { get; init; }

    /// <summary>Human-readable reasons the fit cannot be accepted yet (empty when it can).</summary>
    public IReadOnlyList<string> Warnings { get; init; } = Array.Empty<string>();

    /// <summary>Information that does not block acceptance — the common one being that the
    /// steering never moved enough for the gain, so only the zero was fitted.</summary>
    public IReadOnlyList<string> Notes { get; init; } = Array.Empty<string>();

    /// <summary>One-line summary for the panel (invariant number formatting).</summary>
    public string Message { get; init; } = string.Empty;
}
