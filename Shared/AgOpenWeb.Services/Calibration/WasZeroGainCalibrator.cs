// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;
using System.Collections.Generic;

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// Least-squares fit of the WAS (wheel angle sensor) zero and gain from the yaw rate the
/// vehicle actually turned with (algorithm 2).
///
/// Model — see <see cref="WasCalibrationSample"/> for the signs:
/// <code>δ_true = g·x + b,   where δ_true = atan(L·ω / v)</code>
/// with <c>x</c> the module's reported steer angle, <c>L</c> the wheelbase, <c>ω</c> the IMU
/// yaw rate and <c>v</c> the ground speed. The sensor's zero is <c>−b/g</c> and the gain
/// <c>g</c> is the inverse sensor scale (≈ 1 when the scale is right).
///
/// The gain is only observable if the operator actually steered: the plan's gate is 8° of
/// steer-angle range. Below that the fit reports the zero alone, with the gain fixed at 1,
/// and says so in <see cref="WasCalibrationReport.Notes"/> — driving straight is enough to
/// re-zero, but it cannot measure a scale error.
///
/// This class is pure computation: no DI, no I/O, no ambient state. Nothing here writes a
/// setting; the caller turns the report into a proposal the operator applies.
/// </summary>
public static class WasZeroGainCalibrator
{
    private const double RadiansToDegrees = 180.0 / Math.PI;

    /// <summary>
    /// The true steer angle implied by the yaw-rate geometry:
    /// <c>atan(L·ω / v)</c> in degrees.
    /// </summary>
    /// <param name="wheelbaseMeters">Axle distance, metres (<c>VehicleConfig.Wheelbase</c>).</param>
    /// <param name="yawRateRadiansPerSecond">Yaw rate from the IMU gyro, positive = turning left.</param>
    /// <param name="speedMetersPerSecond">Ground speed, m/s. Must be positive.</param>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="wheelbaseMeters"/> is not positive.</exception>
    public static double TrueSteerAngleDegrees(
        double wheelbaseMeters, double yawRateRadiansPerSecond, double speedMetersPerSecond)
    {
        if (!(wheelbaseMeters > 0.0))
        {
            throw new ArgumentOutOfRangeException(
                nameof(wheelbaseMeters), wheelbaseMeters, "The wheelbase must be positive.");
        }

        return Math.Atan(wheelbaseMeters * yawRateRadiansPerSecond / speedMetersPerSecond)
            * RadiansToDegrees;
    }

    /// <summary>Fit with <see cref="WasCalibrationOptions.Default"/>.</summary>
    public static WasCalibrationReport Fit(
        IReadOnlyList<WasCalibrationSample> samples, double wheelbaseMeters)
        => Fit(samples, wheelbaseMeters, null);

    /// <summary>
    /// Fit the WAS zero and (when the steering moved enough) the gain from
    /// <paramref name="samples"/>, and report whether the result may be accepted.
    /// Samples are expected to have passed <see cref="WasCalibrationCollector"/>'s gates.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="wheelbaseMeters"/> is not positive.</exception>
    public static WasCalibrationReport Fit(
        IReadOnlyList<WasCalibrationSample> samples,
        double wheelbaseMeters,
        WasCalibrationOptions? options)
    {
        ArgumentNullException.ThrowIfNull(samples);
        if (!(wheelbaseMeters > 0.0))
        {
            throw new ArgumentOutOfRangeException(
                nameof(wheelbaseMeters), wheelbaseMeters, "The wheelbase must be positive.");
        }

        options ??= WasCalibrationOptions.Default;

        // Only finite samples with a usable speed contribute: the geometry divides by v.
        double minSteer = double.PositiveInfinity;
        double maxSteer = double.NegativeInfinity;
        int usable = 0;
        for (int i = 0; i < samples.Count; i++)
        {
            var sample = samples[i];
            if (!double.IsFinite(sample.MeasuredSteerAngleDegrees)
                || !double.IsFinite(sample.YawRateRadiansPerSecond)
                || !double.IsFinite(sample.SpeedMetersPerSecond)
                || sample.SpeedMetersPerSecond <= 0.0)
            {
                continue;
            }

            usable++;
            if (sample.MeasuredSteerAngleDegrees < minSteer) minSteer = sample.MeasuredSteerAngleDegrees;
            if (sample.MeasuredSteerAngleDegrees > maxSteer) maxSteer = sample.MeasuredSteerAngleDegrees;
        }

        double steerRange = usable == 0 ? 0.0 : maxSteer - minSteer;

        var warnings = new List<string>();
        if (usable < options.MinSamples)
        {
            warnings.Add(FormattableString.Invariant(
                $"only {usable} usable sample(s), {options.MinSamples} needed"));
        }

        if (usable < 3)
        {
            return NotDeterminate(
                warnings,
                FormattableString.Invariant(
                    $"at least 3 usable samples are needed, {usable} recorded"));
        }

        // Not enough steering for the slope to be separated from the noise: report the zero
        // alone with the gain fixed at 1 (the plan's rule), and say so.
        if (steerRange < options.MinSteerRangeDegrees)
        {
            return FitZeroOnly(samples, wheelbaseMeters, warnings, usable, steerRange, options);
        }

        return FitZeroAndGain(samples, wheelbaseMeters, warnings, usable, steerRange, options);
    }

    /// <summary>
    /// The zero-only fit, used when the steering never moved enough for the slope to be
    /// observable: with the gain assumed to be 1 the model collapses to
    /// <c>b = mean(δ_true − x)</c> and the zero is <c>−b</c>.
    /// </summary>
    private static WasCalibrationReport FitZeroOnly(
        IReadOnlyList<WasCalibrationSample> samples,
        double wheelbaseMeters,
        List<string> warnings,
        int usable,
        double steerRange,
        WasCalibrationOptions options)
    {
        double sum = 0.0;
        for (int i = 0; i < samples.Count; i++)
        {
            var sample = samples[i];
            if (!IsUsable(sample)) continue;
            sum += TrueSteerAngleDegrees(
                    wheelbaseMeters, sample.YawRateRadiansPerSecond, sample.SpeedMetersPerSecond)
                - sample.MeasuredSteerAngleDegrees;
        }

        double intercept = sum / usable;
        double zero = -intercept;

        double sumSquares = 0.0;
        for (int i = 0; i < samples.Count; i++)
        {
            var sample = samples[i];
            if (!IsUsable(sample)) continue;
            double predicted = sample.MeasuredSteerAngleDegrees + intercept;
            double residual = TrueSteerAngleDegrees(
                    wheelbaseMeters, sample.YawRateRadiansPerSecond, sample.SpeedMetersPerSecond)
                - predicted;
            sumSquares += residual * residual;
        }

        int degreesOfFreedom = usable - 1;
        double zeroStdDev = degreesOfFreedom > 0
            ? Math.Sqrt(sumSquares / degreesOfFreedom / usable)
            : double.NaN;

        var notes = new List<string>
        {
            FormattableString.Invariant(
                $"the steering only moved {steerRange:F2}°, {options.MinSteerRangeDegrees:F1}° needed")
            + ", so the gain cannot be told apart from the noise: it was fitted with the gain fixed at 1",
        };

        var fit = new WasCalibrationFit
        {
            MeasuredSteerAngleOffsetDegrees = zero,
            Gain = 1.0,
            GainObservable = false,
            ZeroStdDevDegrees = zeroStdDev,
            GainStdDev = double.NaN,
            RmsDegrees = Math.Sqrt(sumSquares / usable),
            SampleCount = usable,
            SteerRangeDegrees = steerRange,
        };

        return new WasCalibrationReport
        {
            IsDeterminate = true,
            IsAccepted = warnings.Count == 0,
            Fit = fit,
            Warnings = warnings,
            Notes = notes,
            Message = FormattableString.Invariant(
                    $"WAS zero {zero:F2}° (gain assumed 1) from {usable} samples")
                + WarningSuffix(warnings),
        };
    }

    /// <summary>Finite and moving: the geometry divides by the speed.</summary>
    private static bool IsUsable(WasCalibrationSample sample)
        => double.IsFinite(sample.MeasuredSteerAngleDegrees)
            && double.IsFinite(sample.YawRateRadiansPerSecond)
            && double.IsFinite(sample.SpeedMetersPerSecond)
            && sample.SpeedMetersPerSecond > 0.0;

    /// <summary>
    /// The two-unknown fit: regressors <c>[x, 1]</c>, yaw-rate target
    /// <c>atan(L·ω/v)</c>. The zero is <c>−b/g</c>.
    /// </summary>
    private static WasCalibrationReport FitZeroAndGain(
        IReadOnlyList<WasCalibrationSample> samples,
        double wheelbaseMeters,
        List<string> warnings,
        int usable,
        double steerRange,
        WasCalibrationOptions options)
    {
        const int UnknownCount = 2;

        var normalMatrix = new double[UnknownCount, UnknownCount];
        var rightHandSide = new double[UnknownCount];
        var basis = new double[UnknownCount];

        for (int i = 0; i < samples.Count; i++)
        {
            var sample = samples[i];
            if (!IsUsable(sample)) continue;

            basis[0] = sample.MeasuredSteerAngleDegrees;
            basis[1] = 1.0;
            LinearLeastSquares.Accumulate(
                normalMatrix,
                rightHandSide,
                basis,
                TrueSteerAngleDegrees(
                    wheelbaseMeters, sample.YawRateRadiansPerSecond, sample.SpeedMetersPerSecond));
        }

        var solution = LinearLeastSquares.Solve(normalMatrix, rightHandSide);
        double pivot = solution?.SmallestPivot ?? 0.0;
        if (solution is null || pivot < options.MinPivot)
        {
            return NotDeterminate(warnings, FormattableString.Invariant(
                $"the samples do not separate the zero from the gain (smallest normalized pivot {pivot:F4}, ")
                + FormattableString.Invariant($"need {options.MinPivot:F2})"));
        }

        double gain = solution.Coefficients[0];
        double intercept = solution.Coefficients[1];
        if (!(Math.Abs(gain) > 1e-6))
        {
            return NotDeterminate(warnings, FormattableString.Invariant(
                $"the gain came out at {gain:F4}, so the zero −b/g is not defined"));
        }

        double sumSquares = 0.0;
        for (int i = 0; i < samples.Count; i++)
        {
            var sample = samples[i];
            if (!IsUsable(sample)) continue;

            double predicted = (gain * sample.MeasuredSteerAngleDegrees) + intercept;
            double residual = TrueSteerAngleDegrees(
                    wheelbaseMeters, sample.YawRateRadiansPerSecond, sample.SpeedMetersPerSecond)
                - predicted;
            sumSquares += residual * residual;
        }

        int degreesOfFreedom = usable - UnknownCount;
        double zero = -intercept / gain;
        double gainStdDev = solution.StandardError(0, sumSquares, degreesOfFreedom);
        double zeroVariance = ZeroVariance(solution, gain, intercept, sumSquares, degreesOfFreedom);

        if (gain <= 0.0)
        {
            warnings.Add("the gain came out negative — check that the yaw rate and the steer angle share a sign");
        }

        var fit = new WasCalibrationFit
        {
            MeasuredSteerAngleOffsetDegrees = zero,
            Gain = gain,
            GainObservable = true,
            ZeroStdDevDegrees = double.IsNaN(zeroVariance) ? double.NaN : Math.Sqrt(zeroVariance),
            GainStdDev = gainStdDev,
            RmsDegrees = Math.Sqrt(sumSquares / usable),
            SampleCount = usable,
            SteerRangeDegrees = steerRange,
        };

        return new WasCalibrationReport
        {
            IsDeterminate = true,
            IsAccepted = warnings.Count == 0,
            Fit = fit,
            Warnings = warnings,
            Message = FormattableString.Invariant(
                    $"WAS zero {zero:F2}°, gain {gain:F3} from {usable} samples ({steerRange:F1}° of steering)")
                + WarningSuffix(warnings),
        };
    }

    /// <summary>
    /// Variance of the zero <c>−b/g</c> from the covariance of the two coefficients, by the
    /// delta method. <c>NaN</c> when it cannot be formed (no residual degrees of freedom, or
    /// the linearized combination comes out negative).
    /// </summary>
    private static double ZeroVariance(
        LeastSquaresSolution solution,
        double gain,
        double intercept,
        double residualSumSquares,
        int degreesOfFreedom)
    {
        double varianceGain = solution.Variance(0, residualSumSquares, degreesOfFreedom);
        double varianceIntercept = solution.Variance(1, residualSumSquares, degreesOfFreedom);
        double covariance = solution.Covariance(0, 1, residualSumSquares, degreesOfFreedom);
        if (double.IsNaN(varianceGain) || double.IsNaN(varianceIntercept) || double.IsNaN(covariance))
        {
            return double.NaN;
        }

        // zero = −b/g  →  ∂/∂g = b/g², ∂/∂b = −1/g.
        double dGain = intercept / (gain * gain);
        double dIntercept = -1.0 / gain;
        double variance = (dGain * dGain * varianceGain)
            + (dIntercept * dIntercept * varianceIntercept)
            + (2.0 * dGain * dIntercept * covariance);

        return variance < 0.0 ? double.NaN : variance;
    }

    private static WasCalibrationReport NotDeterminate(List<string> warnings, string reason)
    {
        // The reason always goes in, after any "not enough samples" gate, so the panel says
        // both what is missing and why the solver could not proceed.
        warnings.Add(reason);
        return new WasCalibrationReport
        {
            IsDeterminate = false,
            IsAccepted = false,
            Fit = null,
            Warnings = warnings,
            Message = string.Join("; ", warnings),
        };
    }

    private static string WarningSuffix(IReadOnlyList<string> warnings)
        => warnings.Count == 0 ? string.Empty : " — " + string.Join("; ", warnings);


}
