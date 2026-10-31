// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;
using System.Collections.Generic;

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// The identified steering-box play, from settled points approached from both sides.
/// Produced by <see cref="SteerResponseIdentifier.EstimateBacklash"/>.
/// </summary>
public readonly struct SteeringBacklashFit
{
    /// <summary>Play width in degrees: <c>−2e</c> of the model.</summary>
    public double BacklashDegrees { get; init; }

    /// <summary>Slope k of <c>measured = k·command + c + e·approach</c>.</summary>
    public double Slope { get; init; }

    /// <summary>Intercept c, degrees.</summary>
    public double InterceptDegrees { get; init; }

    /// <summary>The approach-dependent offset e, degrees (half the play, with its sign).</summary>
    public double ApproachOffsetDegrees { get; init; }

    /// <summary>Standard error of <see cref="BacklashDegrees"/>, degrees (<c>NaN</c> when the
    /// residual degrees of freedom ran out).</summary>
    public double BacklashStdDevDegrees { get; init; }

    /// <summary>RMS of the fit residuals, degrees.</summary>
    public double RmsDegrees { get; init; }

    /// <summary>Points settled from below (approach +1).</summary>
    public int FromBelowCount { get; init; }

    /// <summary>Points settled from above (approach −1).</summary>
    public int FromAboveCount { get; init; }
}

/// <summary>
/// Outcome of a backlash estimate. Same shape as <see cref="SteerStepResponseReport"/>;
/// nothing here ever moves the steering, the caller runs the test path.
/// </summary>
public sealed class SteeringBacklashReport
{
    /// <summary>True when both approach directions have enough points and the fit solved.</summary>
    public bool IsDeterminate { get; init; }

    /// <summary>True when the fit is determinate and every gate passed.</summary>
    public bool IsAccepted { get; init; }

    /// <summary>The identified play; <c>null</c> when <see cref="IsDeterminate"/> is false.</summary>
    public SteeringBacklashFit? Fit { get; init; }

    /// <summary>Human-readable reasons the estimate cannot be accepted yet.</summary>
    public IReadOnlyList<string> Warnings { get; init; } = Array.Empty<string>();

    /// <summary>One-line summary for the panel (invariant number formatting).</summary>
    public string Message { get; init; } = string.Empty;
}
