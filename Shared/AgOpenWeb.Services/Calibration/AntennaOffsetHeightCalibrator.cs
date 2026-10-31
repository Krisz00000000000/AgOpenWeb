// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;
using System.Collections.Generic;

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// Least-squares fit of the antenna lateral offset and the antenna height from
/// the raw antenna's deviation off the guidance line (algorithm 1).
///
/// Model, line-relative (see <see cref="AntennaCalibrationSample"/> for the signs):
/// <code>deviation = s · (a + h · sin(roll)) + c</code>
/// Regressors <c>[s, s·sin(roll), 1]</c>, three unknowns solved with a
/// column-normalized normal equation system (three unknowns are small enough for
/// Gauss–Jordan elimination). Driving the same wheel track in both directions keeps
/// <c>c</c> — where the pass itself sits — from absorbing <c>a</c>, and a pass that
/// rolls (slope plus flat) is what separates <c>h</c> from <c>a</c>.
///
/// This class is pure computation: no DI, no I/O, no ambient state, so it is
/// unit-testable with known parameters. Nothing here writes a setting; the caller
/// turns the report into a proposal the operator applies.
/// </summary>
public static class AntennaOffsetHeightCalibrator
{
    private const int UnknownCount = 3;
    private const double DegreesToRadians = Math.PI / 180.0;
    private const double NormEpsilon = 1e-12;

    /// <summary>Fit with <see cref="AntennaCalibrationOptions.Default"/>.</summary>
    public static AntennaCalibrationReport Fit(IReadOnlyList<AntennaCalibrationSample> samples)
        => Fit(samples, null);

    /// <summary>
    /// Fit the antenna lateral offset, the antenna height and the pass position
    /// from <paramref name="samples"/>, and report whether the result may be accepted.
    /// Samples are expected to have passed <see cref="AntennaCalibrationCollector"/>'s gates.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">
    /// A sample's <see cref="AntennaCalibrationSample.DirectionSign"/> is not +1 or −1.
    /// </exception>
    public static AntennaCalibrationReport Fit(
        IReadOnlyList<AntennaCalibrationSample> samples,
        AntennaCalibrationOptions? options)
    {
        ArgumentNullException.ThrowIfNull(samples);
        options ??= AntennaCalibrationOptions.Default;

        int forward = 0;
        int reverse = 0;
        double minRoll = double.PositiveInfinity;
        double maxRoll = double.NegativeInfinity;

        for (int i = 0; i < samples.Count; i++)
        {
            var sample = samples[i];
            if (Math.Abs(Math.Abs(sample.DirectionSign) - 1.0) > 1e-9)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(samples),
                    sample.DirectionSign,
                    "DirectionSign must be +1 (A→B) or −1 (B→A).");
            }

            if (sample.DirectionSign > 0) forward++;
            else reverse++;

            if (sample.RollDegrees < minRoll) minRoll = sample.RollDegrees;
            if (sample.RollDegrees > maxRoll) maxRoll = sample.RollDegrees;
        }

        double rollRange = samples.Count == 0 ? 0.0 : maxRoll - minRoll;

        // Gates the operator can act on ("more samples", "more roll"), independent of
        // whether the solver itself succeeded.
        var warnings = new List<string>();
        if (forward < options.MinSamplesPerDirection)
        {
            warnings.Add(FormattableString.Invariant(
                $"only {forward} usable sample(s) driving A→B, {options.MinSamplesPerDirection} needed"));
        }

        if (reverse < options.MinSamplesPerDirection)
        {
            warnings.Add(FormattableString.Invariant(
                $"only {reverse} usable sample(s) driving B→A, {options.MinSamplesPerDirection} needed"));
        }

        if (rollRange < options.MinRollRangeDegrees)
        {
            warnings.Add(FormattableString.Invariant(
                $"the roll only moved {rollRange:F2}°, {options.MinRollRangeDegrees:F1}° needed"));
        }

        if (samples.Count < UnknownCount + 1)
        {
            return NotDeterminate(warnings, FormattableString.Invariant(
                $"at least {UnknownCount + 1} samples are needed, {samples.Count} recorded"));
        }

        // ── Normal equations: AᵀA and Aᵀy ─────────────────────────────────────
        var ata = new double[UnknownCount, UnknownCount];
        var aty = new double[UnknownCount];
        var basis = new double[UnknownCount];

        for (int i = 0; i < samples.Count; i++)
        {
            var sample = samples[i];
            double s = sample.DirectionSign;
            basis[0] = s;
            basis[1] = s * Math.Sin(sample.RollDegrees * DegreesToRadians);
            basis[2] = 1.0;
            double y = sample.LateralDeviationMeters;

            for (int row = 0; row < UnknownCount; row++)
            {
                aty[row] += basis[row] * y;
                for (int col = 0; col < UnknownCount; col++)
                {
                    ata[row, col] += basis[row] * basis[col];
                }
            }
        }

        // Normalize each column so a pivot compares like with like (the direction and
        // the constant column have norm √n; the roll column's norm is the roll spread).
        var columnNorm = new double[UnknownCount];
        for (int j = 0; j < UnknownCount; j++) columnNorm[j] = Math.Sqrt(ata[j, j]);

        if (columnNorm[0] <= NormEpsilon || columnNorm[1] <= NormEpsilon || columnNorm[2] <= NormEpsilon)
        {
            return NotDeterminate(warnings,
                Diagnose(forward, reverse, rollRange, samples.Count, options, 0.0));
        }

        var normalized = new double[UnknownCount, UnknownCount];
        var rhs = new double[UnknownCount];
        for (int row = 0; row < UnknownCount; row++)
        {
            for (int col = 0; col < UnknownCount; col++)
            {
                normalized[row, col] = ata[row, col] / (columnNorm[row] * columnNorm[col]);
            }

            rhs[row] = aty[row] / columnNorm[row];
        }

        if (!TryGaussJordan(normalized, rhs, out var z, out var inverse, out double minPivot)
            || minPivot < options.MinPivot)
        {
            return NotDeterminate(warnings,
                Diagnose(forward, reverse, rollRange, samples.Count, options, minPivot));
        }

        var unknowns = new double[UnknownCount];
        for (int j = 0; j < UnknownCount; j++) unknowns[j] = z[j] / columnNorm[j];

        // ── Residuals and the covariance of the estimate ─────────────────────
        double sumSquares = 0.0;
        for (int i = 0; i < samples.Count; i++)
        {
            var sample = samples[i];
            double s = sample.DirectionSign;
            basis[0] = s;
            basis[1] = s * Math.Sin(sample.RollDegrees * DegreesToRadians);
            basis[2] = 1.0;

            double predicted = 0.0;
            for (int j = 0; j < UnknownCount; j++) predicted += basis[j] * unknowns[j];

            double residual = sample.LateralDeviationMeters - predicted;
            sumSquares += residual * residual;
        }

        double rms = Math.Sqrt(sumSquares / samples.Count);
        int degreesOfFreedom = samples.Count - UnknownCount;
        double sigmaSquared = degreesOfFreedom > 0 ? sumSquares / degreesOfFreedom : double.NaN;

        double StdDev(int index) => degreesOfFreedom > 0
            ? Math.Sqrt(sigmaSquared * inverse[index, index]) / columnNorm[index]
            : double.NaN;

        var fit = new AntennaCalibrationFit
        {
            LateralOffsetMeters = unknowns[0],
            HeightAboveRollAxisMeters = unknowns[1],
            LineOffsetMeters = unknowns[2],
            LateralOffsetStdDevMeters = StdDev(0),
            HeightStdDevMeters = StdDev(1),
            LineOffsetStdDevMeters = StdDev(2),
            RmsMeters = rms,
            ForwardSampleCount = forward,
            ReverseSampleCount = reverse,
            RollRangeDegrees = rollRange,
            SampleCount = samples.Count,
        };

        bool accepted = warnings.Count == 0;
        return new AntennaCalibrationReport
        {
            IsDeterminate = true,
            IsAccepted = accepted,
            Fit = fit,
            Warnings = warnings,
            Message = accepted
                ? FormattableString.Invariant(
                    $"antenna offset {fit.LateralOffsetMeters:F3} m, height {fit.HeightAboveRollAxisMeters:F3} m, ")
                    + FormattableString.Invariant($"line position {fit.LineOffsetMeters:F3} m, RMS {fit.RmsMeters:F3} m from {fit.SampleCount} samples")
                : FormattableString.Invariant(
                    $"fitted from {samples.Count} samples but {warnings.Count} gate(s) not met: {string.Join("; ", warnings)}"),
        };
    }

    private static AntennaCalibrationReport NotDeterminate(List<string> warnings, string reason)
    {
        // A gate warning is the actionable part; a fit that cannot be solved at all is
        // reported as its own reason and never as an accepted proposal.
        if (warnings.Count == 0) warnings.Add(reason);

        return new AntennaCalibrationReport
        {
            IsDeterminate = false,
            IsAccepted = false,
            Fit = null,
            Warnings = warnings,
            Message = reason,
        };
    }

    private static string Diagnose(
        int forward,
        int reverse,
        double rollRange,
        int sampleCount,
        AntennaCalibrationOptions options,
        double minPivot)
    {
        if (sampleCount < UnknownCount + 1)
        {
            return FormattableString.Invariant(
                $"at least {UnknownCount + 1} samples are needed, {sampleCount} recorded");
        }

        if (forward == 0 || reverse == 0)
        {
            return FormattableString.Invariant(
                $"both driving directions are needed — recorded {forward} sample(s) A→B and {reverse} B→A; ")
                + "from one direction alone the antenna offset, the antenna height and the line position "
                + "cannot be told apart";
        }

        if (rollRange < options.MinRollRangeDegrees)
        {
            return FormattableString.Invariant(
                $"the roll never varied (range {rollRange:F2}°, need {options.MinRollRangeDegrees:F1}°), so the ")
                + "height term h*sin(roll) is a constant and cannot be separated from the antenna offset; "
                + "record part of the pass on a slope and part on the flat";
        }

        return FormattableString.Invariant(
            $"the recorded samples do not separate the unknowns (smallest normalized pivot {minPivot:F4}, ")
            + FormattableString.Invariant($"need {options.MinPivot:F2})");
    }

    /// <summary>
    /// Solve the (already column-normalized) normal equations by Gauss–Jordan
    /// elimination with partial pivoting, and hand back the inverse of the system
    /// matrix so the caller can build the parameter uncertainties.
    /// </summary>
    private static bool TryGaussJordan(
        double[,] matrix,
        double[] rhs,
        out double[] solution,
        out double[,] inverse,
        out double minPivot)
    {
        const int width = 2 * UnknownCount + 1;
        var augmented = new double[UnknownCount, width];
        for (int row = 0; row < UnknownCount; row++)
        {
            for (int col = 0; col < UnknownCount; col++) augmented[row, col] = matrix[row, col];
            augmented[row, UnknownCount + row] = 1.0;
            augmented[row, 2 * UnknownCount] = rhs[row];
        }

        minPivot = double.MaxValue;
        for (int col = 0; col < UnknownCount; col++)
        {
            int pivotRow = col;
            double best = Math.Abs(augmented[col, col]);
            for (int row = col + 1; row < UnknownCount; row++)
            {
                double candidate = Math.Abs(augmented[row, col]);
                if (candidate > best)
                {
                    best = candidate;
                    pivotRow = row;
                }
            }

            if (best < minPivot) minPivot = best;

            if (best < NormEpsilon)
            {
                solution = new double[UnknownCount];
                inverse = new double[UnknownCount, UnknownCount];
                return false;
            }

            if (pivotRow != col)
            {
                for (int j = 0; j < width; j++)
                {
                    (augmented[col, j], augmented[pivotRow, j]) = (augmented[pivotRow, j], augmented[col, j]);
                }
            }

            double pivot = augmented[col, col];
            for (int j = col; j < width; j++) augmented[col, j] /= pivot;

            for (int row = 0; row < UnknownCount; row++)
            {
                if (row == col) continue;

                double factor = augmented[row, col];
                if (factor == 0.0) continue;

                for (int j = col; j < width; j++) augmented[row, j] -= factor * augmented[col, j];
            }
        }

        solution = new double[UnknownCount];
        inverse = new double[UnknownCount, UnknownCount];
        for (int row = 0; row < UnknownCount; row++)
        {
            solution[row] = augmented[row, 2 * UnknownCount];
            for (int col = 0; col < UnknownCount; col++) inverse[row, col] = augmented[row, UnknownCount + col];
        }

        return true;
    }
}

