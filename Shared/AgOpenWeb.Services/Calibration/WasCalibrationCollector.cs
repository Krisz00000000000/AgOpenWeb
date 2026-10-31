// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;
using System.Collections.Generic;

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// Records the samples a WAS zero / gain calibration needs, applying the plan's
/// measurement filters on the way in, and fits them on request
/// (<see cref="WasZeroGainCalibrator"/> does the math).
///
/// Stateful by design — it holds the recording buffer — but dependency-free: no DI, no
/// I/O, and it never writes a configuration value. Fitting is explicit so the least-squares
/// pass never runs on the GPS hot path.
///
/// Expected use: on flat, empty ground the operator weaves left and right at 1.5–3 m/s
/// (±10–15°) and the cycle hands every GPS sample's measured steer angle, IMU yaw rate,
/// speed and roll here.
/// </summary>
public sealed class WasCalibrationCollector
{
    private readonly object _lock = new();
    private readonly List<WasCalibrationSample> _samples = new();
    private readonly WasCalibrationOptions _options;

    private double _minSteer = double.PositiveInfinity;
    private double _maxSteer = double.NegativeInfinity;
    private bool _isCollecting;

    public WasCalibrationCollector(WasCalibrationOptions? options = null)
    {
        _options = options ?? WasCalibrationOptions.Default;
    }

    /// <summary>The filters and fit thresholds in force for this collector.</summary>
    public WasCalibrationOptions Options => _options;

    /// <summary>True between <see cref="Start"/> and <see cref="Stop"/>.</summary>
    public bool IsCollecting
    {
        get { lock (_lock) return _isCollecting; }
    }

    /// <summary>Samples recorded so far.</summary>
    public int Count
    {
        get { lock (_lock) return _samples.Count; }
    }

    /// <summary>Peak-to-peak steer angle covered so far, degrees (0 until two samples differ).</summary>
    public double SteerRangeDegrees
    {
        get
        {
            lock (_lock)
            {
                return _samples.Count == 0 ? 0.0 : _maxSteer - _minSteer;
            }
        }
    }

    /// <summary>Begin recording. Existing samples are kept.</summary>
    public void Start()
    {
        lock (_lock) _isCollecting = true;
    }

    /// <summary>Pause recording. Existing samples and counters are kept.</summary>
    public void Stop()
    {
        lock (_lock) _isCollecting = false;
    }

    /// <summary>Drop every sample, leaving the collector stopped.</summary>
    public void Reset()
    {
        lock (_lock)
        {
            _samples.Clear();
            _minSteer = double.PositiveInfinity;
            _maxSteer = double.NegativeInfinity;
            _isCollecting = false;
        }
    }

    /// <summary>
    /// Try to record one sample, applying the plan's filters (speed, roll, yaw rate and
    /// steer-angle limits). Nothing is recorded while the collector is stopped.
    /// </summary>
    /// <param name="measuredSteerAngleDegrees">Steer angle the module reports, positive = left.</param>
    /// <param name="yawRateRadPerSecond">Yaw rate from the IMU gyro, positive = turning left.</param>
    /// <param name="speedMps">Ground speed, m/s.</param>
    /// <param name="rollDegrees">IMU roll, degrees.</param>
    /// <param name="rejection">Reason the sample was dropped, or <c>null</c> when it was kept.</param>
    public bool TryAdd(
        double measuredSteerAngleDegrees,
        double yawRateRadPerSecond,
        double speedMps,
        double rollDegrees,
        out string? rejection)
    {
        if (!double.IsFinite(measuredSteerAngleDegrees) || !double.IsFinite(yawRateRadPerSecond)
            || !double.IsFinite(speedMps) || !double.IsFinite(rollDegrees))
        {
            rejection = "the sample carries a non-finite value";
            return false;
        }

        if (speedMps < _options.MinSpeedMps)
        {
            rejection = FormattableString.Invariant(
                $"speed {speedMps:F2} m/s is below {_options.MinSpeedMps:F2} m/s");
            return false;
        }

        if (Math.Abs(rollDegrees) > _options.MaxAbsRollDegrees)
        {
            rejection = FormattableString.Invariant(
                $"roll {rollDegrees:F2}° is past {_options.MaxAbsRollDegrees:F1}°, the model needs flat ground");
            return false;
        }

        if (Math.Abs(yawRateRadPerSecond) > _options.MaxAbsYawRateRadPerSecond)
        {
            rejection = FormattableString.Invariant(
                $"yaw rate {yawRateRadPerSecond:F3} rad/s is past {_options.MaxAbsYawRateRadPerSecond:F2} rad/s");
            return false;
        }

        if (Math.Abs(measuredSteerAngleDegrees) > _options.MaxAbsSteerAngleDegrees)
        {
            rejection = FormattableString.Invariant(
                $"steer angle {measuredSteerAngleDegrees:F1}° is past {_options.MaxAbsSteerAngleDegrees:F0}°");
            return false;
        }

        lock (_lock)
        {
            if (!_isCollecting)
            {
                rejection = "not recording";
                return false;
            }

            if (_samples.Count >= _options.MaxSamples)
            {
                rejection = FormattableString.Invariant($"the buffer is full ({_options.MaxSamples} samples)");
                return false;
            }

            _samples.Add(new WasCalibrationSample(
                measuredSteerAngleDegrees, yawRateRadPerSecond, speedMps, rollDegrees));

            if (measuredSteerAngleDegrees < _minSteer) _minSteer = measuredSteerAngleDegrees;
            if (measuredSteerAngleDegrees > _maxSteer) _maxSteer = measuredSteerAngleDegrees;
        }

        rejection = null;
        return true;
    }

    /// <summary>
    /// Fit everything recorded so far for the given wheelbase. Safe to call while
    /// recording; the sample snapshot is taken under the collector's lock and the math
    /// runs outside it.
    /// </summary>
    public WasCalibrationReport Fit(double wheelbaseMeters)
    {
        WasCalibrationSample[] snapshot;
        lock (_lock)
        {
            snapshot = _samples.ToArray();
        }

        return WasZeroGainCalibrator.Fit(snapshot, wheelbaseMeters, _options);
    }
}
