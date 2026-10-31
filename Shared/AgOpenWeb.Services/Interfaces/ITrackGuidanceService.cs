// AgOpenWeb
// Copyright (C) 2024-2025 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using AgOpenWeb.Models.Track;
using AgOpenWeb.Services.Calibration;

namespace AgOpenWeb.Services.Interfaces;

/// <summary>
/// Unified track guidance service interface.
/// Handles both Pure Pursuit and Stanley algorithms.
/// Works with both AB lines (2 points) and curves (N points).
/// </summary>
public interface ITrackGuidanceService
{
    /// <summary>
    /// Calculate steering guidance for a track.
    /// </summary>
    /// <param name="input">Guidance input parameters including track, position, and configuration</param>
    /// <returns>Guidance output with steering angle, cross-track error, and updated state</returns>
    TrackGuidanceOutput CalculateGuidance(TrackGuidanceInput input);

    /// <summary>
    /// Attach (or, with <c>null</c>, detach) the cross-track integrator of algorithm 4 of the
    /// automatic calibration plan. While one is attached the steering laws steer on the
    /// corrected error <c>e + c</c>; with none attached the behaviour is bit-for-bit what it
    /// was before the feature existed. It is only <b>read</b> on the control path — the
    /// integrating step runs once per GPS cycle on the host — so attach or detach it while
    /// disengaged.
    /// </summary>
    void SetXteIntegrator(XteIntegrator? integrator);
}
