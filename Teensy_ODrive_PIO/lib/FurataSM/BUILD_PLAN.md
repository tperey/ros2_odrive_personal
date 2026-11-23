# FurataStateMachine — Build Plan

Fill in the TODOs in this order. Each stage ends with something you can **test on hardware**,
and the motor can't produce torque until everything feeding it has been checked.

**Sensing → estimation → logic → torque.** Don't skip ahead of Stage 3.

Build: `pio run -e furata_balance`  ·  Upload: `pio run -e furata_balance -t upload`

---

## Stage 1 — Small helpers (no hardware needed) ✅ DONE

File: `lib/FurataSM/FurataStateMachine.cpp`

- [x] `_saturate` — clamp into `[-lim, lim]`
- [x] `_wrapPi` — wrap into `(-pi, pi]`
- [x] `_post` / `_has` — event bitmask (consider a `_bit(Event)` helper)
- [x] `_timeInStateMs` — `millis() - _stateEntryMs`
- [x] `_motorArmed` — true for `SWING_UP`, `BALANCE`

**Test:** optional desktop tests in `host_tests/`. Remove each `(void)x;` as you fill in a function.

---

## Stage 2 — Wrapper sensing and telemetry ✅ DONE

File: `lib/OdriveCANWrapper/OdriveCANWrapper.cpp`

- [x] `onHeartbeat` — set `last_heartbeat_ms = millis()`
- [x] `command_odrv0_torque` — wrap `poll_telemetry()` in `if (poll)`
- [x] `convert_current_odrv0_feedback` — turns → rad, flip sign, ZOH, the **only** place that clears `received_feedback`
- [x] `read_current_odrv0_health` — axis error / state / ms since heartbeat
- [x] `request_motor_idle_nonblocking` — one `setState(IDLE)`, no wait, no `clearErrors()`
- [x] `sendTelemetry_furata` — 45-byte packet: state, sm_fault, tau_cmd, x_hat[4], Iq set/meas, tau target/est
- [x] **PC parser** — `logger_teensy_node.py --full` (45-byte packet)

**Test:** builds. Existing envs still build (`pio run`).

---

## Stage 3 — Read-only state machine, motor never armed

File: `lib/FurataSM/FurataStateMachine.cpp`

- [ ] `begin` — `initialize_singleODCW`, anticog, `_enc.write(0)`, seed filter, enter IDLE
- [ ] `tick` — the fixed skeleton
- [ ] `_sense` — t1 from ODrive, t2 from encoder (raw, not wrapped)
- [ ] `_estimate` — `kf.step(_tauPrev, t1, t2)`, `kf.getState(_x)`
- [ ] `_seedFilterFromMeasurements` — x0 from measurements, P0 from `SMConfig::P0_DIAG`
- [ ] `_actuate` — saturate, gate with `_motorArmed`, store `_tauPrev`
- [ ] `_sendTelemetry` — fill `FurataTelemetry`, call `sendTelemetry_furata`
- [ ] **Temporary:** call `stop_motor_single()` at the end of `begin()` so the axis isn't left in
      closed loop (`_entry` is still a stub). Remove in Stage 4.

**Test — the big one. First time the UKF runs on hardware.** Move the arm and pendulum by hand,
plot `x_hat` (run the logger with `--full`). Raw `t1_meas` / `t2_meas` were dropped from the
packet. Consider adding them back temporarily: without them you can't see filter lag or noise
against the measurement.

- [ ] `ENC_SIGN` correct (+t2 matches the model)
- [ ] t1 units (rad) and sign correct
- [ ] dt2 wraps cleanly near upright (±pi)
- [ ] velocities look sane, no lag / blow-up
- [ ] tick fits in 1 ms (UKF + CAN polls + telemetry)
- [ ] tune `Q`, `R` in `src/furata_balance/furata_balance.cpp`

---

## Stage 4 — Events and transitions, all torques zero

Files: `lib/FurataSM/FurataStateMachine.cpp`, `src/furata_balance/furata_balance.cpp`

- [ ] `_readSerialCommands` — non-blocking line parser → `CMD_START/STOP/RESET`
- [ ] `_collectEvents` — faults (set `_fault` here, first cause only), thresholds, timer
- [ ] `_checkTransitions` — ordered `if`s with early `return` = priority (faults → stop → per-state)
- [ ] `_transitionTo` — `exit(old)` → switch → `entry(new)`
- [ ] `_entry` / `_exit` — remove the Stage 3 temporary stop
- [ ] `_during` — return `0` in **every** state for now
- [ ] `main` overrun policy — entry actions block ~150 ms; don't fire a burst of catch-up ticks

**Test:** walk the whole machine by hand, motor armed but zero torque. Watch `state` / `fault` in telemetry.

- [ ] `start` → SWING_UP (rejected if pendulum not hanging still)
- [ ] lift pendulum up → CAUGHT → BALANCE
- [ ] let go → FELL → IDLE
- [ ] wait in SWING_UP → SWINGUP_TIMEOUT → IDLE
- [ ] `stop` from SWING_UP and BALANCE → IDLE
- [ ] turn arm past `T1_LIMIT` → FAULT, `reset` → IDLE
- [ ] unplug CAN → heartbeat fault → FAULT (and the SM doesn't hang)

---

## Stage 5 — LQR, catching by hand

- [ ] `_lqrTorque` — `tau = -(K · x)` with `LQRConfig::K`
- [ ] `_during(BALANCE)` returns it; `_swingUpTorque` stays `return 0.0f`

**Test:** small `TAU_MAX` first. Hold the pendulum near upright, send `start`, lift into the catch
window, and **check the torque pushes the right way before letting go**. Wrong direction is almost
always a sign mismatch between the encoder / t1 frame and the frame the LQR was designed in.

- [ ] torque direction correct
- [ ] balances with hand-catch
- [ ] raise `TAU_MAX`, tune `CATCH_ANGLE`, `CATCH_RATE`, `FALL_ANGLE`

---

## Stage 6 — Swing-up

- [ ] `_swingUpTorque` — energy pumping (formula in the TODO)
- [ ] `_during(SWING_UP)` returns it

**Test:** tiny `SWINGUP_GAIN` first to check the sign (energy should grow), then tune.

- [ ] energy grows, not shrinks
- [ ] arm stays inside `T1_LIMIT`
- [ ] catch → BALANCE works from a real swing-up
- [ ] tune `SWINGUP_GAIN`, `SWINGUP_TIMEOUT_MS`
