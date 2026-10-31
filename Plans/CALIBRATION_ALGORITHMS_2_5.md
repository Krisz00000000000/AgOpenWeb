# Automatic calibration — algorithms 2–5: calculators, tests and the UI wiring

**Status:** all four algorithms are implemented **and wired**: 2 (WAS zero + gain) records from
the GPS cycle and publishes the module proposal its Apply writes, 4 (cross-track integrator) is
switchable and steers on the corrected error, 5 (pass-drift watcher) records one deviation per
pass in observer mode. Entry 1 stays as it was (guided wizard). **Not wired on purpose:** 3
(steering response) — it is the only algorithm that moves the steering and the plan requires a
reviewed, gated test path first — and 6 (EKF/LQR/MPC), which the plan defers until 1–5 work on
real data. Nothing writes a setting without the operator: every Apply goes through
`config.set` / `autosteer.sendSave`, the existing paths.

Source: *„AgOpenWeb – automatikus kalibrációs algoritmusok és AI-ügynöki megvalósítási
terv"* (Hungarian, 2026-10-31), sections 2–5 and *Bevezetési sorrend*. This document covers the
plan's step 1 ("write the calculators and their NUnit tests; gate: all green") for those four
algorithms, and records what the simulations actually measured.

## What landed

| Algorithm | File | What it is |
|---|---|---|
| shared | `Calibration/LinearLeastSquares.cs` + `LeastSquaresSolution.cs` | Column-normalized normal equations solved by Gauss–Jordan with partial pivoting; the smallest pivot is the determinacy gate, the normalized inverse gives the parameter uncertainties. Used by 2 and 3; algorithm 1 keeps its own copy (migrating it is a follow-up, not a behaviour change). |
| 2 WAS | `WasCalibrationSample.cs` | One observation (measured angle, yaw rate, speed, roll) carrying the sign conventions. |
| 2 WAS | `WasCalibrationOptions.cs` | The plan's filters (1 m/s, 3° roll, 0.5 rad/s yaw rate, 35° steer) and gates (300 samples, 8° of steering, pivot). |
| 2 WAS | `WasCalibrationReport.cs` | `WasCalibrationFit` (zero, gain, standard errors, RMS, counts) and the report (`IsDeterminate`, `IsAccepted`, warnings, notes, message). |
| 2 WAS | `WasZeroGainCalibrator.cs` | The fit itself: `δ_true = atan(L·ω/v) = g·x + b`, zero `−b/g`, the zero-only path when the steering never moved, delta-method uncertainty of the zero. Pure math. |
| 2 WAS | `WasCalibrationCollector.cs` | Recording buffer with the filters and `Fit(wheelbase)` on request. No DI, no I/O, no config writes. |
| 3 steering | `SteerResponseSample.cs` | `SteerResponseSample` (time, commanded, measured) and `SteeringBacklashPoint` (command, measured, approach sign). |
| 3 steering | `SteerResponseOptions.cs` | The plan's grids (θ 0–0.8 s / 41, τ 0.03–3 s / 40, two refinement passes), the sample and step gates, the rate-limit thresholds, the backlash gate. |
| 3 steering | `SteerStepResponseReport.cs`, `SteeringBacklashReport.cs` | The two fits and their reports. |
| 3 steering | `SteerResponseIdentifier.cs` | `FitStepResponse` (largest commanded jump, dead time + time constant grid, closed-form gain, rate-limit detection) and `EstimateBacklash` (three unknowns `[command, 1, approach]`, play `−2e`). Never sends a command. |
| 4 integrator | `XteIntegratorOptions.cs`, `XteIntegratorInput.cs`, `XteIntegrator.cs` | `c ← clamp(c·(1 − λΔt) + k_i·e·Δt, ±c_max)` with the plan's gates, the conditional anti-windup rule and a naive mode for comparison. Off by default; `Update` is a no-op while disabled. |
| 5 drift | `PassDriftOptions.cs`, `PassDriftStatus.cs`, `PassDriftWatcher.cs` | Per-pass EMA plus consecutive-threshold alarm, the green/amber/red indicator, the "same sign across directions = suspected side-slip" hint and a bounded per-pass log. |
| tests | `Tests/AgOpenWeb.Services.Tests/Calibration/` | `WasZeroGainCalibrationTests`, `SteerResponseIdentifierTests`, `XteIntegratorTests`, `PassDriftWatcherTests`. |

All of these live under `Shared/AgOpenWeb.Services/Calibration/`.

Run them:

```
dotnet test Tests/AgOpenWeb.Services.Tests/ -c Debug --filter FullyQualifiedName~Calibration
```

The diagnostic dumps (`WasZeroGainCalibrationTests.Report`, `SteerResponseIdentifierTests.Report`,
`XteIntegratorTests.Report`) are `[Explicit]`; select them by name to print the margins:

```
dotnet test Tests/AgOpenWeb.Services.Tests/ -c Debug --filter "FullyQualifiedName~Calibration&Name=Report"
```

## What the simulations measured

Injected truth and measured error over five seeds (white **and** AR(1) noise, as the plan
demands), against the plan's own acceptance gates:

| Algorithm | Gate (plan) | Measured |
|---|---|---|
| 2 WAS zero | 0.15° | ≤ 0.030° (worst seed), std error ≈ 0.010° |
| 2 WAS gain | 0.02 | ≤ 0.003 |
| 3 dead time | 30 ms | ≤ 1 ms |
| 3 time constant | 40 ms | ≤ 2 ms |
| 3 gain | 0.02 | ≤ 0.002 |
| 3 play | — (the plan asks for 1.2° back) | 1.2° recovered exactly; ≤ 0.1° with noise |
| 4 steady error with slip | < 1.5 cm | 0.00 cm (12.04 cm without the integrator) |
| 4 saturated 3 m start | < 2 cm | 0.00 cm |
| 5 noise | no false alarm | none in 200 passes at ±1 cm |
| 5 growing drift | alarm within a few passes | at the 9th pass (1 cm/pass, α = 0.3) |

## Findings that change the plan

1. **The WAS gain is the *inverse* sensor scale.** The plan says "gain g, ≈ 1 when the scale
   is right", and a 10 % scale error therefore shows up as `g ≈ 0.909`, not `1.1`. The sample
   type documents the physical model (`x = s·δ_true + z`), so the fitted number cannot be read
   the other way round.
2. **The plan's rate-limit ratio alone would miss a rate-limited servo.** Peak rate divided by
   the fitted `K·Δ/τ` converges towards 1 as the fit adapts `τ` to the ramp, so a ramp can pass
   the 0.7 gate. A shape ratio was added next to it — `(t95 − t10)/(t63 − t10)`, independent of
   the dead time: **3.25** for a first-order response, **1.6** for a ramp, gated at 2.0. On the
   simulated rate-limited servo the plan's ratio reads 0.45 and the shape 1.60, so both fire.
   Both thresholds stay starting values to tune on a real servo, exactly as the plan says.
3. **The plan's anti-windup expectation was not reproduced.** The plan expects conditional
   integration to overshoot less than a clamp-only integrator. Measured on the closed loop in
   `XteIntegratorTests`: **identical** (0.965 m) from a 3 m start, and **6 mm more** on a small
   saturated start. Two reasons: the plan's own 0.5 m error gate already stops the integration
   during a large approach, and the integrator state here is an error bias rather than an
   actuator command, so the classic windup mechanism is weak. The rule is implemented and stays
   (it costs nothing, and it still protects a long saturation that happens while |e| < 0.5 m),
   but the effective protection is the error gate plus the ±c_max clamp. The plan's headline
   numbers do hold: 12.04 cm → 0.00 cm under constant side-slip, and under 2 cm after a
   saturated start.
4. **The shape check is a shape statement, not a noise statement.** The crossing times
   (t10/t63/t95) are read from the measured signal with linear interpolation, so it needs the
   step to be large against the noise — the 1° step gate plus the 10-sample pre-step noise
   estimate make it reliable (0.02° of noise on a 5° step moved the ratio between 3.10 and 3.31).

## The wiring

| Algorithm | Where | What happens |
|---|---|---|
| shared | `Services/Calibration/WasZeroGainApplication.cs` | Turns a WAS fit into the two module settings (`wasOffset` in counts, `countsPerDegree / gain`) and refuses what may not be written (non-positive gain, offset past ±3900). Pure arithmetic; the VM publishes its result as the proposal, so the panel only ever sends what it shows. |
| 2 WAS | `MainViewModel.Calibration.cs` (`ObserveWasCalibration`, `PublishWasCalibration`), `CalibrationState.Was*`, Status frame, `#cal2` panel | Every GPS cycle while recording appends one observation — the module's PGN 253 `ActualSteerAngle`, the yaw rate, the speed (m/s) and the roll — through the collector's filters. Every 25th sample (and on start/stop) it fits with `Vehicle.Wheelbase` and publishes zero, gain, standard errors and the proposal. **Apply** writes `autosteer.wasOffset` + `autosteer.countsPerDegree` through `config.set` and pushes them with `autosteer.sendSave`; **Revert** puts the two old values back. |
| 4 integrator | `ITrackGuidanceService.SetXteIntegrator`, `TrackGuidanceService`, `MainViewModel.Calibration.cs` (`ObserveXteIntegrator`), `#cal4` | Switching it on builds an `XteIntegrator` (`Enabled = true`), attaches it to the guidance, and steps it **once per GPS fix** from the line-relative cross-track error, with the RTK / engaged / speed / error / saturation gates. The steering laws only *read* the correction (Pure Pursuit shifts the goal point, Stanley the offset line), so the 100 Hz control path's cost is unchanged. Reset on a disengage and on a U-turn completion; switching off drops it (nothing is persisted). |
| 5 drift | `SwathSeamProbe` + `PassDriftWatcher` + `MainViewModel.Calibration.cs` (`ObservePassDrift`), `#cal5` panel | Armed while working: every 5th cycle the seam to the neighbouring worked pass is probed against the live coverage map; a pass's probes are averaged and handed to the watcher when the guidance pass offset changes. Observer only — it corrects nothing. |

Calibration tests: `dotnet test Tests/AgOpenWeb.Services.Tests/ -c Debug --filter FullyQualifiedName~Calibration`
(plus `CalibrationPanelTests` in the ViewModels suite for the wiring).

### Decisions and findings from the wiring

1. **There was no yaw-rate source in the app.** `GpsData.ImuYawRate` is never filled from a
   sentence and `AutoSteerService`'s `_state.YawRate` is always 0 (`NmeaEpochAssembler` sets it
   to 0), while `GpsPipelineService` deliberately passes `yawRateRadPerSec = 0` to the estimator
   because a board's gyro field is firmware-dependent and was seen sending −103.8 °/s parked.
   `PositionEstimator` therefore *derives* the rate from the heading change and EMA-filters it —
   that derived rate is now exposed as `GpsCycleResult.YawRateRadPerSecond`, which is what
   algorithm 2 fits against. Nothing else consumes it yet.
2. **The sign of that rate.** AgOpenWeb's heading is the compass heading, so the derived rate is
   **positive when turning right**, while the collector's convention (the plan's table) is
   positive = turning left — the VM negates it. The fit measures the sign anyway, which is why a
   fitted gain at or below zero is refused with "check the WAS direction and the yaw-rate sign"
   rather than applied.
3. **The integrator steps on the GPS cycle, not inside `CalculateGuidance`.** Two callers run the
   steering laws (the pipeline's per-cycle call and the 100 Hz control loop), so integrating
   inside them would advance the same correction several times per fix. The law reads
   `XteIntegrator.Correction`; the host owns the single step. No new fields were needed on
   `TrackGuidanceInput` after all, so that model is unchanged.
4. **Algorithm 2's zero lives in the steer module**, not in a guidance setting: Apply is
   `AutoSteer.WasOffset`/`CountsPerDegree` + the module push (PGN 251/252). The zero formula is
   AgOpenGPS's own Zero WAS (`WasCalibration.ZeroedOffset`), applied with the **old**
   countsPerDegree — the raw offset is scale-independent, and the pair then reads zero under the
   new scale too.
5. **The seam probe measured the wrong distance.** `SwathSeamProbe.MeasureSide` returned the
   *first uncovered step* for the gap case, i.e. ≈ 0.05 m for every gap, and its 4 m reach could
   never see an overlap (that branch needs first-uncovered ≥ 0.9 × width). It now returns the
   distance at which the neighbour's coverage resumes, and the default look-out is 8 m so a wide
   tool's overlap is inside the probe. Both cases are pinned in `SwathSeamProbeTests`.
6. **Two different direction frames on purpose.** Algorithm 1's deviation is line-relative
   (direction sign folded in) because the model is about the antenna geometry; algorithm 5 is fed
   the **vehicle-relative** seam, which is the frame its "same sign in both directions = suspected
   side-slip" hint is written for. Algorithm 4's error is line-relative again (the
   `CrossTrackError` the panel shows is flipped for the reverse direction, so the VM un-flips it).

## Deliberately not done yet

- **Algorithm 3 is not wired.** Menu entries 3 and 6 stay disabled. 3 is the only algorithm that
  moves the steering and the plan requires a separately reviewed, gated path: held button, ±10°,
  ≤ 1 m/s, watchdog, and the normal autosteer not active at the same time. The obvious reuse is
  the Steer Wizard's already-reviewed free-drive steps (`AutoMotorCalibrationStepViewModel`,
  `MaxSteeringAngleStepViewModel`), with `SteerResponseIdentifier` reading the samples — but that
  is a safety review, not a wiring task, so it stays open.
- **The PGN 253 feedback rate is still unmeasured.** It sets the dead-time resolution; the plan
  notes that 10 Hz feedback cannot resolve a 0.15 s dead time (it gates algorithm 3).
- **Algorithm 4 and the Pure Pursuit integral.** `GuidanceConfig.PurePursuitIntegralGain` is a
  second integral in the same loop and the plan says the two must not both be on. Nothing stops
  that yet — the panel says nothing about it, so a warning or a guard is the follow-up.
- **The drift probe's cost is unmeasured on a real field.** Two probe walks of 8 m at 5 cm are
  ≈320 coverage lookups every 5th cycle (≈64/cycle amortised). The HotPath budgets pass because
  that suite never arms algorithm 5; a budget case for an armed watcher is the follow-up.
- **Algorithm 5 has no per-field log yet.** `PassDriftWatcher.History` is kept in memory; writing
  it beside the field (like the coverage) is a follow-up.
- **No ground truth.** Every number here comes from synthetic cycles; 2's zero and 5's threshold
  have not been compared with a tape measure or a real seam.
- **Algorithm 6** (EKF, LQR/MPC, implement offset): untouched, as the plan asks.

## Next steps (each gated on the one before)

1. Operator review of this document: the two Apply paths (vehicle fields for 1, module
   `wasOffset`/`countsPerDegree` for 2), the integrator's sign and reset points, and the drift
   thresholds.
2. Bench / VirtualModules run: 2 against a simulated weave, 4 switched on against a side-slip, 5
   armed on a two-pass strip. The signs are the point of these runs.
3. Then the field procedure in the plan's own order: 1 (with 4 off), 2, 4, 5.
4. Measure the real rate of PGN 253 — it gates algorithm 3 and the dead-time resolution.
5. Algorithm 3 last, after the safety review of its gated test path.