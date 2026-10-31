// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;
using System.Collections.Generic;

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// Records the samples an antenna lateral-offset / height calibration needs, applying
/// the measurement gates on the way in, and fits them on request
/// (<see cref="AntennaOffsetHeightCalibrator"/> does the math).
///
/// Stateful by design — it holds the recording buffer — but dependency-free: no DI,
/// no I/O, and it never writes a configuration value. Fitting is explicit so the
/// least-squares pass never runs on the GPS hot path; the operator (or a timer)
/// calls <see cref="Fit"/> when a recording is done.
///
/// Expected use: the operator drives one wheel track in both directions on ground
/// that rolls (part slope, part flat) with autosteer following the line; the pipeline
/// hands every cycle's raw (uncorrected) antenna deviation here.
/// </summary>
public sealed class AntennaCalibrationCollector
{
    private readonly object _lock = new();
    private readonly List<AntennaCalibrationSample> _samples = new();
    private readonly AntennaCalibrationOptions _options;

    private int _forwardCount;
    private int _reverseCount;
    private double _minRoll = double.PositiveInfinity;
    private double _maxRoll = double.NegativeInfinity;
    private bool _isCollecting;

    public AntennaCalibrationCollector(AntennaCalibrationOptions? options = null)
    {
        _options = options ?? AntennaCalibrationOptions.Default;
    }

    /// <summary>The gates and fit thresholds in force for this collector.</summary>
    public AntennaCalibrationOptions Options => _options;

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

    /// <summary>Samples recorded driving A→B.</summary>
    public int ForwardCount
    {
        get { lock (_lock) return _forwardCount; }
    }

    /// <summary>Samples recorded driving B→A.</summary>
    public int ReverseCount
    {
        get { lock (_lock) return _reverseCount; }
    }

    /// <summary>Peak-to-peak roll covered so far, degrees (0 until two samples differ).</summary>
    public double RollRangeDegrees
    {
        get
        {
            lock (_lock)
            {
                return _samples.Count == 0 ? 0.0 : _maxRoll - _minRoll;
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

    /// <summary>Drop every sample and counter, leaving the collector stopped.</summary>
    public void Reset()
    {
        lock (_lock)
        {
            _samples.Clear();
            _forwardCount = 0;
            _reverseCount = 0;
            _minRoll = double.PositiveInfinity;
            _maxRoll = double.NegativeInfinity;
            _isCollecting = false;
        }
    }

    /// <summary>
    /// Offer one cycle's observation. Returns false (with a reason) when the sample
    /// fails a gate — not collecting, not RTK Fixed, too slow, too far off the line,
    /// an invalid direction sign, a non-finite value, or the buffer is full.
    /// </summary>
    /// <param name="directionSign">+1 driving the track's A→B order, −1 driving it B→A.</param>
    /// <param name="rollDegrees">IMU roll in degrees, positive = vehicle right side down.</param>
    /// <param name="lateralDeviationMeters">Raw antenna deviation from the line, positive = right of A→B
    /// (line-relative — see <see cref="AntennaCalibrationSample"/>).</param>
    /// <param name="fixQuality">GGA fix quality of the fix; only RTK Fixed (4) is accepted.</param>
    /// <param name="speedMps">Ground speed in m/s.</param>
    /// <param name="rejection">Reason the sample was dropped, or <c>null</c> when it was kept.</param>
    public bool TryAdd(
        double directionSign,
        double rollDegrees,
        double lateralDeviationMeters,
        int fixQuality,
        double speedMps,
        out string? rejection)
    {
        if (!double.IsFinite(directionSign) || !double.IsFinite(rollDegrees)
            || !double.IsFinite(lateralDeviationMeters) || !double.IsFinite(speedMps))
        {
            rejection = "the sample carries a non-finite value";
            return false;
        }

        if (Math.Abs(Math.Abs(directionSign) - 1.0) > 1e-9)
        {
            rejection = FormattableString.Invariant(
                $"direction sign must be +1 or -1, got {directionSign:F2}");
            return false;
        }

        if (fixQuality != _options.RtkFixedFixQuality)
        {
            rejection = FormattableString.Invariant(
                $"fix quality {fixQuality} is not RTK Fixed ({_options.RtkFixedFixQuality})");
            return false;
        }

        if (speedMps < _options.MinSpeedMps)
        {
            rejection = FormattableString.Invariant(
                $"speed {speedMps:F2} m/s is below {_options.MinSpeedMps:F2} m/s");
            return false;
        }

        if (Math.Abs(lateralDeviationMeters) > _options.MaxAbsDeviationMeters)
        {
            rejection = FormattableString.Invariant(
                $"the antenna is {lateralDeviationMeters:F2} m off the line, more than ")
                + FormattableString.Invariant($"{_options.MaxAbsDeviationMeters:F2} m");
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

            _samples.Add(new AntennaCalibrationSample(directionSign, rollDegrees, lateralDeviationMeters));
            if (directionSign > 0) _forwardCount++;
            else _reverseCount++;

            if (rollDegrees < _minRoll) _minRoll = rollDegrees;
            if (rollDegrees > _maxRoll) _maxRoll = rollDegrees;
        }

        rejection = null;
        return true;
    }

    /// <summary>
    /// Fit everything recorded so far. Safe to call while recording; the sample
    /// snapshot is taken under the collector's lock and the math runs outside it.
    /// </summary>
    public AntennaCalibrationReport Fit()
    {
        AntennaCalibrationSample[] snapshot;
        lock (_lock)
        {
            snapshot = _samples.ToArray();
        }

        return AntennaOffsetHeightCalibrator.Fit(snapshot, _options);
    }
}
