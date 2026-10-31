# Plans

Work being done or agreed, one file per plan. Each starts with a **Status** line.

| Plan | Status |
|---|---|
| [CALIBRATION_ALGORITHM_1_ANTENNA.md](CALIBRATION_ALGORITHM_1_ANTENNA.md) | Calculator + tests landed; pipeline/UI wiring, gate: sign review |
| [CALIBRATION_ALGORITHMS_2_5.md](CALIBRATION_ALGORITHMS_2_5.md) | 2, 4 and 5 wired (panels + sampling); 3 waits for its safety review, 6 not started; gate: sign/threshold review |
| [GPS_RECEIVER_SENTENCES_PLAN.md](GPS_RECEIVER_SENTENCES_PLAN.md) | In progress: receivers straight to the app |
| [ISOBUS_VT_TRANSPORTS.md](ISOBUS_VT_TRANSPORTS.md) | In progress: how the VT reaches the implement bus |
| [JOB_FINISH_LOCK_IMPLEMENT_PLAN.md](JOB_FINISH_LOCK_IMPLEMENT_PLAN.md) | Agreed, not started: implement on the job, Finish locks it |
| [RTCM_FORWARDING_PLAN.md](RTCM_FORWARDING_PLAN.md) | Built; open bench checks |

- **[Ideas/](Ideas/)**: written up but not started or on hold. Nothing there is agreed.
- **[Completed/](Completed/)**: done, decided, or overtaken (much of it from the Avalonia UI). Code
  comments still point there for the reasoning behind a design.
- Reference documents (architecture, threat model, performance, ISOBUS mapping) are in
  [Docs/](../Docs/).

When a plan is done, move it to `Completed/` with `git mv` and update the links to it.
