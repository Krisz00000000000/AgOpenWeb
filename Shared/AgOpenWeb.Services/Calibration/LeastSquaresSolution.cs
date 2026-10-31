// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;
using System.Collections.Generic;

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// A solved least-squares system: the coefficients, the column norms, the inverse of the
/// <b>normalized</b> normal matrix and the smallest pivot seen while eliminating.
/// </summary>
public sealed class LeastSquaresSolution
{
    internal LeastSquaresSolution(
        double[] coefficients,
        double[] columnNorms,
        double[,] normalizedInverse,
        double smallestPivot)
    {
        Coefficients = coefficients;
        ColumnNorms = columnNorms;
        NormalizedInverse = normalizedInverse;
        SmallestPivot = smallestPivot;
    }

    /// <summary>The fitted coefficients, in the order the regressors were supplied.</summary>
    public IReadOnlyList<double> Coefficients { get; }

    /// <summary>The norm of each regressor column (√ of the ATA diagonal).</summary>
    public IReadOnlyList<double> ColumnNorms { get; }

    /// <summary>Inverse of the column-normalized normal matrix — the covariance shape.</summary>
    public double[,] NormalizedInverse { get; }

    /// <summary>Smallest pivot of the elimination; a small one means the unknowns are
    /// nearly dependent and the coefficients are not usable.</summary>
    public double SmallestPivot { get; }

    /// <summary>Variance of coefficient <paramref name="index"/>'s estimate
    /// (<c>σ² · (AᵀA)⁻¹</c>), <c>NaN</c> when the degrees of freedom ran out.</summary>
    public double Variance(int index, double residualSumSquares, int degreesOfFreedom)
        => degreesOfFreedom > 0
            ? (residualSumSquares / degreesOfFreedom) * NormalizedInverse[index, index]
                / (ColumnNorms[index] * ColumnNorms[index])
            : double.NaN;

    /// <summary>Covariance of two coefficients' estimates, <c>NaN</c> when the degrees of
    /// freedom ran out.</summary>
    public double Covariance(
        int first, int second, double residualSumSquares, int degreesOfFreedom)
        => degreesOfFreedom > 0
            ? (residualSumSquares / degreesOfFreedom) * NormalizedInverse[first, second]
                / (ColumnNorms[first] * ColumnNorms[second])
            : double.NaN;

    /// <summary>Standard error of coefficient <paramref name="index"/>.</summary>
    public double StandardError(int index, double residualSumSquares, int degreesOfFreedom)
    {
        double variance = Variance(index, residualSumSquares, degreesOfFreedom);
        return double.IsNaN(variance) ? double.NaN : Math.Sqrt(variance);
    }
}
