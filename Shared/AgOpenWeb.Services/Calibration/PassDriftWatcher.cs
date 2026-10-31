// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;
using System.Collections.Generic;

namespace AgOpenWeb.Services.Calibration;

/// <summary>
/// The pass-drift watcher of algorithm 5. It corrects nothing: it smooths the signed
/// deviation of each pass from the seam where the neighbouring, earlier pass should have
/// joined, and raises an indicator when the smoothed value stays past a threshold — the
/// signal that the antenna/roll calibration (algorithm 1) or the WAS zero (algorithm 2)
/// has started to drift and should be run again.
///
/// <code>d̄_k = d̄_{k−1} + α·(d_k − d̄_{k−1});  alarm when |d̄| &gt; T for n passes in a row</code>
///
/// One <see cref="Add"/> call per pass, not per cycle, so it never touches the hot path.
/// Stateful but dependency-free: no DI, no I/O, no config writes. The per-pass history is
/// kept for a per-field log, in AgOpenWeb's own shape rather than AgOpenGPS's.
/// </summary>
public sealed class PassDriftWatcher
{
    private readonly object _lock = new();
    private readonly PassDriftOptions _options;
    private readonly List<PassDriftRecord> _history = new();

    private double _smoothed;
    private bool _hasSmoothed;
    private int _consecutiveOverThreshold;
    private int _passCount;
    private double? _lastDeviation;
    private double? _lastDirectionSign;

    public PassDriftWatcher(PassDriftOptions? options = null)
    {
        _options = options ?? PassDriftOptions.Default;
    }

    /// <summary>The thresholds in force for this watcher.</summary>
    public PassDriftOptions Options => _options;

    /// <summary>Passes recorded since the last <see cref="Reset"/>.</summary>
    public int PassCount
    {
        get { lock (_lock) return _passCount; }
    }

    /// <summary>The smoothed deviation, metres (0 before the first pass).</summary>
    public double SmoothedMeters
    {
        get { lock (_lock) return _smoothed; }
    }

    /// <summary>The current traffic light.</summary>
    public PassDriftStatus Status
    {
        get
        {
            lock (_lock)
            {
                if (!_hasSmoothed) return PassDriftStatus.Idle;
                if (_consecutiveOverThreshold >= _options.ConsecutiveUpdatesToAlarm) return PassDriftStatus.Alarm;
                return Math.Abs(_smoothed) >= _options.WatchFraction * _options.AlarmThresholdMeters
                    ? PassDriftStatus.Watch
                    : PassDriftStatus.Ok;
            }
        }
    }

    /// <summary>The per-pass log, oldest first.</summary>
    public IReadOnlyList<PassDriftRecord> History
    {
        get { lock (_lock) return _history.ToArray(); }
    }

    /// <summary>Start over — a new field, or after the operator has re-run a calibration.</summary>
    public void Reset()
    {
        lock (_lock)
        {
            _history.Clear();
            _smoothed = 0.0;
            _hasSmoothed = false;
            _consecutiveOverThreshold = 0;
            _passCount = 0;
            _lastDeviation = null;
            _lastDirectionSign = null;
        }
    }

    /// <summary>
    /// Record one pass and return the indicator.
    /// </summary>
    /// <param name="deviationMeters">The pass's signed deviation from the neighbouring seam:
    /// echo positive when the seam was left short (a gap), negative when the passes overlap.
    /// Which of the two is positive only has to be decided once, consistently.</param>
    /// <param name="directionSign">+1 driving the track's A→B order, −1 driving it B→A.</param>
    /// <exception cref="ArgumentOutOfRangeException">A non-finite deviation or a direction
    /// sign other than ±1.</exception>
    public PassDriftUpdate Add(double deviationMeters, double directionSign = 1.0)
    {
        if (!double.IsFinite(deviationMeters))
        {
            throw new ArgumentOutOfRangeException(
                nameof(deviationMeters), deviationMeters, "The deviation must be finite.");
        }

        if (Math.Abs(Math.Abs(directionSign) - 1.0) > 1e-9)
        {
            throw new ArgumentOutOfRangeException(
                nameof(directionSign), directionSign, "The direction sign must be +1 or −1.");
        }

        lock (_lock)
        {
            _smoothed = _hasSmoothed
                ? _smoothed + (_options.SmoothingAlpha * (deviationMeters - _smoothed))
                : deviationMeters;
            _hasSmoothed = true;

            _consecutiveOverThreshold = Math.Abs(_smoothed) > _options.AlarmThresholdMeters
                ? _consecutiveOverThreshold + 1
                : 0;

            bool sameSignAcrossDirections = _lastDeviation.HasValue
                && _lastDirectionSign.HasValue
                && Math.Abs(directionSign - _lastDirectionSign.Value) > 1e-9
                && Math.Sign(deviationMeters) == Math.Sign(_lastDeviation.Value)
                && deviationMeters != 0.0;

            _lastDeviation = deviationMeters;
            _lastDirectionSign = directionSign;
            _passCount++;

            var status = _consecutiveOverThreshold >= _options.ConsecutiveUpdatesToAlarm
                ? PassDriftStatus.Alarm
                : Math.Abs(_smoothed) >= _options.WatchFraction * _options.AlarmThresholdMeters
                    ? PassDriftStatus.Watch
                    : PassDriftStatus.Ok;

            _history.Add(new PassDriftRecord(_passCount, directionSign, deviationMeters, _smoothed, status));
            if (_history.Count > _options.MaxHistory) _history.RemoveAt(0);

            return new PassDriftUpdate
            {
                Status = status,
                SmoothedMeters = _smoothed,
                DeviationMeters = deviationMeters,
                ConsecutiveOverThreshold = _consecutiveOverThreshold,
                SameSignAcrossDirections = sameSignAcrossDirections,
            };
        }
    }
}
