// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// The small dense least-squares solver the calibration fitters share: build the normal
/// equations (or let <see cref="Accumulate"/> do it), then solve them with Gauss–Jordan
/// elimination with partial pivoting.
///
/// Columns are normalized before the elimination, so
/// <see cref="LeastSquaresSolution.SmallestPivot"/> compares between fits whose regressors
/// live on different scales (metres, degrees, direction signs) — that pivot is what the
/// callers gate "the unknowns cannot be separated" on. The inverse of the normalized system
/// is handed back so a fit can report parameter uncertainties.
///
/// Pure math: no DI, no I/O, no ambient state.
/// </summary>
public static class LinearLeastSquares
{
    /// <summary>Below this a pivot counts as zero (a singular system).</summary>
    public const double Epsilon = 1e-12;

    /// <summary>
    /// Add one observation to the normal equations: <c>ATA += basis·basisᵀ</c> and
    /// <c>ATy += basis·y</c>. <paramref name="normalMatrix"/> must be
    /// <c>unknowns × unknowns</c> and <paramref name="rightHandSide"/> length
    /// <c>unknowns</c>; both must be zeroed by the caller before the first observation.
    /// </summary>
    public static void Accumulate(
        double[,] normalMatrix,
        double[] rightHandSide,
        ReadOnlySpan<double> basis,
        double y)
    {
        ArgumentNullException.ThrowIfNull(normalMatrix);
        ArgumentNullException.ThrowIfNull(rightHandSide);
        if (basis.Length != rightHandSide.Length)
        {
            throw new ArgumentException(
                "The basis length must match the number of unknowns.", nameof(basis));
        }

        for (int row = 0; row < basis.Length; row++)
        {
            rightHandSide[row] += basis[row] * y;
            for (int col = 0; col < basis.Length; col++)
            {
                normalMatrix[row, col] += basis[row] * basis[col];
            }
        }
    }

    /// <summary>
    /// Solve <c>ATA·x = ATy</c>. Returns <c>null</c> when a column is empty or a pivot
    /// vanishes, i.e. the system does not determine its unknowns. Whether the solution is
    /// precise enough to act on is the caller's gate
    /// (<see cref="LeastSquaresSolution.SmallestPivot"/>).
    /// </summary>
    public static LeastSquaresSolution? Solve(double[,] normalMatrix, double[] rightHandSide)
    {
        ArgumentNullException.ThrowIfNull(normalMatrix);
        ArgumentNullException.ThrowIfNull(rightHandSide);

        int unknowns = rightHandSide.Length;
        if (normalMatrix.GetLength(0) != unknowns || normalMatrix.GetLength(1) != unknowns)
        {
            throw new ArgumentException(
                "The normal matrix must be square and match the right-hand side.", nameof(normalMatrix));
        }

        if (unknowns == 0) return null;

        // One norm per column, from the diagonal of ATA. An empty (or degenerate) column
        // means the regressor never varied, so its unknown is not observable at all.
        var columnNorms = new double[unknowns];
        for (int j = 0; j < unknowns; j++)
        {
            double diagonal = normalMatrix[j, j];
            if (!(diagonal > Epsilon * Epsilon)) return null;
            columnNorms[j] = Math.Sqrt(diagonal);
        }

        // Solve the column-normalized system, which is far better conditioned, then scale
        // the solution back. The normalized inverse is what the uncertainties come from.
        int width = unknowns * 3;
        var augmented = new double[unknowns, width];
        for (int row = 0; row < unknowns; row++)
        {
            for (int col = 0; col < unknowns; col++)
            {
                augmented[row, col] = normalMatrix[row, col] / (columnNorms[row] * columnNorms[col]);
            }

            augmented[row, unknowns + row] = 1.0;
            augmented[row, unknowns * 2] = rightHandSide[row] / columnNorms[row];
        }

        double smallestPivot = double.MaxValue;
        for (int col = 0; col < unknowns; col++)
        {
            int pivotRow = col;
            double best = Math.Abs(augmented[col, col]);
            for (int row = col + 1; row < unknowns; row++)
            {
                double candidate = Math.Abs(augmented[row, col]);
                if (candidate > best)
                {
                    best = candidate;
                    pivotRow = row;
                }
            }

            if (best < smallestPivot) smallestPivot = best;
            if (best < Epsilon) return null;

            if (pivotRow != col)
            {
                for (int j = 0; j < width; j++)
                {
                    (augmented[col, j], augmented[pivotRow, j]) = (augmented[pivotRow, j], augmented[col, j]);
                }
            }

            double pivot = augmented[col, col];
            for (int j = col; j < width; j++) augmented[col, j] /= pivot;

            for (int row = 0; row < unknowns; row++)
            {
                if (row == col) continue;

                double factor = augmented[row, col];
                if (factor == 0.0) continue;

                for (int j = col; j < width; j++) augmented[row, j] -= factor * augmented[col, j];
            }
        }

        var coefficients = new double[unknowns];
        var normalizedInverse = new double[unknowns, unknowns];
        for (int row = 0; row < unknowns; row++)
        {
            coefficients[row] = augmented[row, unknowns * 2] / columnNorms[row];
            for (int col = 0; col < unknowns; col++)
            {
                normalizedInverse[row, col] = augmented[row, unknowns + col];
            }
        }

        return new LeastSquaresSolution(coefficients, columnNorms, normalizedInverse, smallestPivot);
    }
}
