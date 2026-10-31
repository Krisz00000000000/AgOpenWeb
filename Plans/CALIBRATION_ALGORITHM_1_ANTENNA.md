# Automatic calibration — algorithm 1: antenna lateral offset and height

**Status:** the calculator layer and its unit tests landed; all tests green.
**Not yet wired** into the GPS pipeline or the web UI, and nothing writes a setting —
that wiring is the plan's next gated step (an operator reviews the signs and the field
names first). Nothing outside `Shared/AgOpenWeb.Services/Calibration/` and the new test
file was modified.

Source: *„AgOpenWeb – automatikus kalibrációs algoritmusok és AI-ügynöki megvalósítási
terv"* (Hungarian, 2026-10-31), sections *Közös alapok* and *1. Antenna oldaleltolás és
magasság*. This document covers that algorithm plus the reconnaissance report the plan
asks for as the mandatory first step.

## What landed

| File | What it is |
|---|---|
| `Shared/AgOpenWeb.Services/Calibration/AntennaCalibrationSample.cs` | One observation: direction sign, roll, raw antenna deviation from the line. Carries the sign conventions. |
| `Shared/AgOpenWeb.Services/Calibration/AntennaCalibrationOptions.cs` | The gates (samples per direction, roll spread, pivot, RTK Fixed, speed, deviation) shared by collector and fitter. |
| `Shared/AgOpenWeb.Services/Calibration/AntennaCalibrationReport.cs` | `AntennaCalibrationFit` (a, h, c + standard errors + RMS + counts) and the report (`IsDeterminate`, `IsAccepted`, warnings, message). |
| `Shared/AgOpenWeb.Services/Calibration/AntennaOffsetHeightCalibrator.cs` | The fit itself: column-normalized normal equations, Gauss–Jordan with partial pivoting, pivot gate, residuals and parameter uncertainties. Pure math, no DI, no I/O. |
| `Shared/AgOpenWeb.Services/Calibration/AntennaCalibrationCollector.cs` | Recording buffer with the measurement gates, counters (per direction, roll range) and `Fit()` on request. No DI, no I/O, no config writes. |
| `Tests/AgOpenWeb.Services.Tests/Calibration/AntennaOffsetHeightCalibrationTests.cs` | 25 test cases: known parameters, 5 seeds, white **and** AR(1) noise, the negative cases, collector gates, locale safety, plus an explicit `Report()`. |

Run them:

```
dotnet test Tests/AgOpenWeb.Services.Tests/ -c Debug --filter FullyQualifiedName~AntennaOffsetHeightCalibrationTests
```

## Reconnaissance report

### 1. Where roll, the antenna offsets and the heading are computed

- **Roll from the receiver.** `NmeaParserServiceFast.cs` reads roll from `$PANDA`
  (field 13, `state.Roll = rawRoll * 0.1`), `$KSXT` (field 6, RTK-Fixed gated) and HPR;
  each passes through `ApplyAhrsRollCalibration(ref state.Roll, configStore)`, so
  `AhrsConfiguration.RollZero` / `IsRollInvert` / `RollFilter` are already applied.
  It surfaces as `GpsData.ImuRoll` (degrees).
- **Roll as the pipeline uses it.** `GpsPipelineService.ProcessCycle` lines 811–815:
  `_roll = _roll * rollFilter + data.ImuRoll * (1 - rollFilter)`, with
  `rollFilter = Clamp(ConfigStore.Ahrs.RollFilter, 0, 0.99)`.
- **Raw antenna position.** Line 819: `antennaE = posEasting + driftE` (local metres,
  uncorrected).
- **The one antenna→pivot transform.** Lines 821–830 call
  `Shared/AgOpenWeb.Services/Gps/AntennaToPivotTransform.cs`,
  `Apply(ref easting, ref northing, headingRad, ConfigStore.Vehicle, _roll)`. It applies,
  in this order: `AntennaPivot` (fore/aft), `AntennaOffset` (lateral) and
  `AntennaHeight × sin(roll)` (roll correction). **This is the code algorithm 1 is
  calibrating** — it is the only place those three values are used.
- **Heading.** `GpsHeadingFusionService.FuseHeading` (dual-antenna → fix-to-fix → IMU
  blend) replaces the raw NMEA heading; `PositionEstimator` dead-reckons between fixes.
- Consequence for the wiring: samples must be taken from the **raw antenna** position,
  before line 825 — after the transform the correction is already removed.


### 2. `IAutoSteerService` — inputs, outputs, cycle time

- Interface: `Shared/AgOpenWeb.Services/Interfaces/IAutoSteerService.cs`; implementation
  `Shared/AgOpenWeb.Services/AutoSteer/AutoSteerService.cs` (owns `VehicleState`,
  coordinates GPS → parse → guidance → PGN).
- Inputs: GPS datagrams (`ProcessGpsBuffer`, `ProcessSimulatedPosition`), guidance results
  (`UpdateGuidanceResults(steerAngle, crossTrackError)`), section bits
  (`SetSectionBits`), engage/disengage, free-drive (`EnableFreeDrive`, `SetFreeDriveAngle`).
- Outputs: PGN 254 + 239 every control tick (`SendPgnsForControlTick`), PGN 100 corrected
  position, PGN 214 GPS, machine config PGNs.
- Feedback in: PGN 253 steer data → `_state.ActualSteerAngle` (line 396); this is also
  where the existing `SmartWasCalibrationService` is fed (line 404).
- Cycle time: the host control loop is 100 Hz (CLAUDE.md line 123, and the hot-path tests
  drive "the real GPS cycle and the 100 Hz control tick"); GPS arrives at ~10 Hz, and the
  steer module's PGN 253 feedback rate is **not documented in `Docs/PGN.md`** — the plan
  asks for it to be measured before algorithm 3 (its resolution limit is what a step
  response can resolve). Still open.
- Nothing in algorithm 1 needs autosteer state; the plan's rule that no algorithm writes a
  setting while autosteer is engaged applies to the apply step.

### 3. PGNs — commanded vs measured steer angle

- `Docs/PGN.md` is the reference (`CLAUDE.md`'s link to `PGN.md` at the repo root is stale).
  Steer module section: **254** = guidance + status out, **253** = module feedback in
  (`LastSteerData.ActualSteerAngle`, decoded in `AutoSteer/PgnBuilder.cs` around line 776,
  `angleRaw / 100.0`), **250** = pressure/current sensor.
- So both the commanded angle (254) and the measured angle (253) are available; the missing
  piece for algorithm 3 is the **rate** of 253, which nothing in the repo records.

### 4. `ConfigurationStore.Vehicle`

`Shared/AgOpenWeb.Models/Configuration/VehicleConfig.cs`:

| Field | Default | Meaning (from the source) |
|---|---|---|
| `Wheelbase` | 2.5 m | axle distance (algorithm 2 needs it) |
| `TrackWidth` | 1.8 m | — |
| `AntennaHeight` | 3.0 m | height used by the roll correction → `h` |
| `AntennaPivot` | 0.0 m | positive = antenna ahead of the pivot |
| `AntennaOffset` | 0.0 m | negative = left of centreline, positive = right → `a` |
| `ImuPivot` / `ImuHeight` / `ImuOffset` | 0.0 | sent in PGN 247, not part of this model |
| `HitchLength`, `HitchType` | 1.8 m / 0 | trailing tools only |
| `MaxSteerAngle` | 35° | steer saturation, used by algorithms 3 and 4 |
| `MaxAngularVelocity` | 35°/s | — |

`MinTurningRadius = Wheelbase / tan(MaxSteerAngle)`.

### 5. Sign of the track cross-track error

- `TrackGuidanceService.CalculateCrossTrackError` (line 785) is `((dz·E) − (dx·N) + …)/|AB|`
  with `(dx, dz) = B − A`: for a north-bound line a point to the east is **positive**, i.e.
  positive = right of the A→B direction.

### 6. Coverage (needed by algorithm 5, not by this one)

Present as `Shared/AgOpenWeb.Services/Coverage/CoverageMapService.cs` (namespace
`AgOpenWeb.Services.Coverage`), consumed through `IsPointCovered` and the offset-pass cache
in `GpsPipelineService`. Where the edge of the previously recorded pass can be read out for
a per-pass deviation is **not yet investigated** — that belongs to algorithm 5's own step.

### 7. Licence and the header rule

Apache-2.0, confirmed in `LICENSE.md`, `README.md` (License section) and `CONTRIBUTING.md`
("Licence headers"): every new `.cs`/`.js`/`.py`/`.sh` file starts with the three-line
header. The new files carry it. The plan's open question about the README naming a second
licence is settled: the repository is Apache-2.0 throughout.

## Sign conventions (verified, not assumed)

The plan warns that signs are the most common failure. Everything below was read out of
the source, not from memory:

| Quantity | AgOpenWeb convention | Verified at |
|---|---|---|
| Roll | positive = the vehicle tilts to its **right** (right side down) | `VehicleState` roll doc, `AntennaToPivotTransform` header, `ActivityTransform` use of `sin(roll)·AntennaHeight` |
| `AntennaOffset` (`a`) | positive = antenna **right** of the centreline | `VehicleConfig` comment, wizard text "From centerline (+ = right)" |
| `AntennaHeight` (`h`) | height above the roll axis, metres; the transform applies `h·sin(roll)` | `AntennaToPivotTransform` lines 51–61 |
| Direction sign (`s`) | `+1` = driving the track's recorded A→B order, `−1` = B→A | pipeline `IsHeadingSameWay` |
| Antenna deviation | positive = right of the **A→B** direction (line-relative) | `CalculateCrossTrackError`; corrected for the published XTE as in §5 |

The model is algebraically the plan's `dev = s(a + h·sin φ) + c`; only the *value* of `a`
differs from the plan's "left-positive" table (ours is right-positive, matching
`VehicleConfig.AntennaOffset` exactly). The transform's own roll sign was checked against
the formula: for roll > 0 the correction moves the position left, i.e. it removes exactly
the rightward `h·sin(roll)` displacement the fit assumes.

## The model, the gates and the math

- Model: `deviation = s·(a + h·sin(roll)) + c`, regressors `[s, s·sin(roll), 1]`, linear
  least squares; the normal equations are **column-normalized**, then solved by
  Gauss–Jordan with partial pivoting. If the smallest pivot drops below
  `AntennaCalibrationOptions.MinPivot` (0.01) the result is *not determinate*.
- Measurement gates (applied per sample by the collector): RTK Fixed only, ≥ 0.5 m/s,
  |deviation| < 1 m, direction sign ±1, buffer cap 20 000 samples.
- Acceptance gates on the fit: ≥ 100 usable samples **per direction**, roll peak-to-peak
  ≥ 3°, and a determinate solve. `IsAccepted` is exactly "determinate and no gate warnings";
  every non-determinate outcome carries a message that names what is missing (one direction
  only / roll never varied / unknown pivot), which is what the panel will show.
- Uncertainties come out of the same solve: `Var(b) = σ²·(AᵀA)⁻¹` with the column
  normalization folded back in, `σ²` from the residual degrees of freedom.

## Test results

`dotnet test ... --filter FullyQualifiedName~AntennaOffsetHeightCalibrationTests` →
**25 passed, 0 failed** (25 test cases in ~65 ms, `net10.0`, Debug).

The explicit diagnostic (`--filter FullyQualifiedName=...Report`) prints the fits for the
noisy runs; the injected truth is a = 0.070 m, h = 2.850 m, c = 0.030 m, with 3 mm white
plus 5 mm AR(1) (ρ = 0.9) noise, 300 samples per direction:

| seed | offset error | height error | RSS std error (offset/height) | RMS |
|---|---|---|---|---|
| 1 | −0.2 mm | 6 mm | 0.2 / 3.0 mm | 5.5 mm |
| 7 | −0.4 mm | 32 mm | 0.2 / 2.8 mm | 5.1 mm |
| 42 | −0.2 mm | 11 mm | 0.2 / 2.9 mm | 5.3 mm |
| 1234 | −1.6 mm | 13 mm | 0.2 / 3.2 mm | 5.7 mm |
| 20261031 | −0.2 mm | 1 mm | 0.2 / 3.3 mm | 6.0 mm |

Both plan acceptance gates are met with margin (offset < 10 mm, height < 80 mm). The RSS
standard error understates the real height spread — as the plan warns ("the real scatter
will be larger than measured with white noise"), because AR(1) noise inflates the true
variance by roughly (1+ρ)/(1−ρ) ≈ 19× and least squares cannot see that. Acceptance
therefore rests on the fixed gates, not on the reported standard error.

Negative cases, all covered by tests: one direction only → not determinate ("both driving
directions are needed"); both directions but constant tilt → not determinate ("the roll
never varied"); too few samples or too little roll → determinate but not accepted, with the
gate named in the warning.

## Deliberately not done yet

- No wiring: nothing in `GpsPipelineService`, no `ICalibrationService`, no `config.set`
  path, no web panel, no i18n strings. The plan's own gate puts a human review of the signs
  and field names before the pipeline is touched.
- No ground-truth measurement: the numbers above are simulation. The plan's field test
  (empty area, low speed, forward/backward on the same wheel track) has not run.
- The fit does not model side-slip. It is the plan's known limit: on a slope the tractor
  can slide, and that offset is not direction-dependent, so it biases `h`. Measure slowly.
- `h` is the height above the **roll axis** as the transform assumes it (which it treats as
  ground level). If the real roll axis is above the ground, the applied correction is the
  same number but means something slightly different; the plan's 8 cm tolerance absorbs it.

## Next steps (each gated on the one before)

1. Operator review of this document: the sign table in §"Sign conventions", the field
   names, and that `h` maps to `VehicleConfig.AntennaHeight` and `a` to
   `VehicleConfig.AntennaOffset`.
2. Wire the collector into `GpsPipelineService` at the raw antenna point (line 819, before
   the transform), using line-relative deviation × direction sign, and an
   `ICalibrationService` that only produces a proposal.
3. VirtualModules run (drive the simulated vehicle both ways with a tilt) to confirm the
   **sign** of the estimate end-to-end, then a bench test, then an empty-area measurement.
4. Only then the web panel (start/stop, sample counters per direction, roll range, result,
   Apply + Revert) with `tr(...)` strings through `Tools/i18n-extract.py`.
5. Algorithm 2 (WAS zero/gain) afterwards — its yaw-rate source and the measured angle
   source are partly identified here (§2, §3), but the module-side vs config-side location
   of the WAS zero still has to be read out of the source.

- **But** the published value is negated when driving against the track
  (`output.DistanceFromLinePivot *= -1.0` for `!isHeadingSameWay`, lines 283–286 and
  355–359, then `CrossTrackError = DistanceFromLinePivot`). `CrossTrackError` is therefore
  **vehicle**-relative (positive = right of the direction the vehicle is travelling), not
  line-relative. The fit needs the line-relative value: multiply by the direction sign
  (`+1` when `IsHeadingSameWay`, `−1` otherwise), which is exactly what
  `AntennaCalibrationSample` documents.
- `IsHeadingSameWay` is the pipeline's "A→B / B→A" flag and is frozen while actively
  steering (`TrackGuidanceState.IsHeadingSameWay`, #422), so it is a stable direction sign
  to record.

