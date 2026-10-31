// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;
using System.Globalization;
using AgOpenWeb.Models.Configuration;

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// What the operator's Apply writes for algorithm 2 (WAS zero + gain), derived from a fitted
/// <see cref="WasCalibrationFit"/>. Pure arithmetic, so the proposal the panel shows and the
/// numbers the operator applies cannot drift apart.
/// </summary>
/// <param name="WasOffset">New <c>AutoSteer.WasOffset</c>, raw module counts.</param>
/// <param name="CountsPerDegree">New <c>AutoSteer.CountsPerDegree</c> (unchanged when the gain
/// was not observable).</param>
/// <param name="GainApplied">True when the fitted gain went into <see cref="CountsPerDegree"/>.</param>
/// <param name="WasOffsetDeltaCounts">How far the offset moved (counts) — the panel's readback.</param>
/// <param name="Message">One line for the panel (invariant number formatting).</param>
public readonly record struct WasZeroGainSettings(
    int WasOffset,
    double CountsPerDegree,
    bool GainApplied,
    int WasOffsetDeltaCounts,
    string Message);

/// <summary>
/// Turns a WAS fit into module settings (algorithm 2's Apply). Both values live in the steer
/// module's calibration (PGN 251/252 through <c>AutoSteerConfig</c>), which is why this is a
/// proposal only: nothing here writes a setting.
///
/// The module turns raw counts into an angle as
/// <code>angle = (raw − 6805 + wasOffset) / countsPerDegree</code>
/// (see <see cref="WasCalibration"/>), so:
/// <list type="bullet">
/// <item>the fitted zero <c>x₀</c> (the reading that means "wheels straight") is removed with
/// <c>wasOffset' = wasOffset − x₀·countsPerDegree</c> — AgOpenGPS's own Zero WAS;</item>
/// <item>a fitted gain <c>g</c> (the inverse sensor scale) is applied by scaling the sensor:
/// <c>countsPerDegree' = countsPerDegree / g</c>, which makes the module report
/// <c>angle·g</c> at the same raw reading. The zero above uses the <b>old</b>
/// countsPerDegree on purpose: the raw offset is independent of the angle scale, and the
/// pair (x₀, cpd) then reads zero at the same raw count under the new scale too.</item>
/// </list>
///
/// Refusals (the plan's guards): a gain at or below zero would flip the sensor's direction —
/// that is an inverted WAS or an inverted yaw-rate sign, not a calibration value — and an
/// offset past <see cref="WasCalibration.MaxOffset"/> is AgOpenGPS's "Excessive Steer Angle —
/// Cannot Zero".
/// </summary>
public static class WasZeroGainApplication
{
    /// <summary>Appended to the panel's line when the steering never moved enough for the gain.</summary>
    private const string GainNotMeasuredText = "(the steering never moved far enough to measure it)";

    /// <summary>
    /// The settings Apply would write, or <c>false</c> with the reason in
    /// <paramref name="settings"/>'s message when they may not be written.
    /// </summary>
    public static bool TryCompute(
        int currentWasOffset,
        double countsPerDegree,
        WasCalibrationFit fit,
        out WasZeroGainSettings settings)
    {
        if (!(countsPerDegree > 0.0) || !double.IsFinite(countsPerDegree))
        {
            settings = Invalid(currentWasOffset, countsPerDegree,
                "counts per degree is not set for this vehicle, so the WAS cannot be zeroed");
            return false;
        }

        if (!double.IsFinite(fit.MeasuredSteerAngleOffsetDegrees))
        {
            settings = Invalid(currentWasOffset, countsPerDegree, "the fit carries no usable zero");
            return false;
        }

        bool gainObservable = fit.GainObservable;
        if (gainObservable && (!(fit.Gain > 0.0) || !double.IsFinite(fit.Gain)))
        {
            settings = Invalid(currentWasOffset, countsPerDegree,
                FormattableString.Invariant(
                    $"the fitted gain {fit.Gain:F3} is not positive")
                + " — check the WAS direction (invert WAS) and the yaw-rate sign before applying");
            return false;
        }

        int wasOffset = WasCalibration.ZeroedOffset(
            currentWasOffset, fit.MeasuredSteerAngleOffsetDegrees, countsPerDegree);
        if (Math.Abs(wasOffset) > WasCalibration.MaxOffset)
        {
            settings = Invalid(currentWasOffset, countsPerDegree,
                FormattableString.Invariant(
                    $"the zero would need offset {wasOffset} counts, past the ±{WasCalibration.MaxOffset} limit"));
            return false;
        }

        double newCountsPerDegree = gainObservable ? countsPerDegree / fit.Gain : countsPerDegree;
        int delta = wasOffset - currentWasOffset;
        string cpdText = newCountsPerDegree.ToString("F1", CultureInfo.InvariantCulture);
        string gainNote = GainNotMeasuredText;

        string message = gainObservable
            ? FormattableString.Invariant(
                $"WAS zero {wasOffset} counts ({delta:+#;-#;0}), gain {fit.Gain:F3} → {cpdText} counts/deg")
            : FormattableString.Invariant(
                $"WAS zero {wasOffset} counts ({delta:+#;-#;0}); the gain stayed {cpdText} counts/deg {gainNote}");

        settings = new WasZeroGainSettings(wasOffset, newCountsPerDegree, gainObservable, delta, message);
        return true;
    }

    private static WasZeroGainSettings Invalid(
        int currentWasOffset, double countsPerDegree, string message)
        => new(currentWasOffset, countsPerDegree, false, 0, message);
}
