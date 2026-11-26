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

## Stage 2.5 — Friction comp + UKF on hardware, outside the state machine

Standalone test functions in `lib/OdriveCANWrapper/OdriveCANWrapper.cpp`, each run from its own
main + `platformio.ini` env, like the SID mains (`perform_*` pattern: own 1 kHz loop, `check_for_shutdown_msg()`).
Log with `logger_teensy_node.py --full` via `sendTelemetry_furata` (`state = 0`, `sm_fault = 0`).

**Why here:** the state machine assumes friction is compensated and the UKF works. Check both
without any state machine logic in the way.

> **Recommended before 2.5c:** add raw `t1_meas` / `t2_meas` back into the packet (+8 bytes, update
> `FURATA_TEL_PKCT_SIZE` and the logger). Tuning Q/R without the measurements is guesswork, and
> logging `(tau, t1, t2)` lets you replay the UKF offline in Python instead of reflashing.

### 2.5a — Implement friction comp in `command_odrv0_torque` ✅ DONE

Implemented as `fric_comp(tau_cmd, vel_revs)` in `OdriveCANWrapper.cpp`, called from
`command_odrv0_torque` when `doAntifriction`. Returns the compensation only (added like the anticog
term). Config: `load_friction_comp(FrictionCompParams)`, values in
`lib/config/m8325s_furata/friction_comp.hpp` (`fs` from `Arm1FsConfig::fs1`). The old friction
arguments of `load_anticogging_config` are disabled.

Two regimes, chosen by arm velocity `v` (ODrive `Vel_Estimate` → telemetry frame rad/s) and the
requested torque `tau_cmd` (`tau_setpoint`, before anticog):

```
fs = pct * fs1                      // recomputed every call, so live changes take effect

MOVING  (entered |v| > V_TH_HI, left |v| < V_TH_LO):
    comp = copysignf(fs, v)         // cancel Coulomb friction, which opposes motion
STATIC:
    if |tau_cmd| > TAU_TH:  comp = copysignf(fs, tau_cmd)   // breakaway help
    else:                   comp = 0                        // settle; no creep, no buzz
```

- [x] Implement the two regimes above
- [x] Hysteresis on `V_TH` (`V_TH_HI` / `V_TH_LO`, function-local `static FrictionState`)
- [x] Velocity in the telemetry frame, rad/s (`-kTwoPi * Vel_Estimate`); signs verified through to `setTorque`
- [x] Config struct + `load_friction_comp`, copied into a file-static; `fs` derived per call
- [x] Placeholder values in `friction_comp.hpp`: `PCT = 0.7`, `V_TH_HI = 0.05`, `V_TH_LO = 0.02`,
      `TAU_TH = 0.01` (tuned in 2.5b / Stage 5)
- [x] KF/UKF is fed the **pre-compensation** tau (the model assumes friction is cancelled)

### 2.5b — Test friction comp at tau = 0

New function, e.g. `perform_zero_torque_test()`: closed loop, `command_odrv0_torque(0.0f)` with
anticog + friction comp, log every tick.

- [ ] **Flick test:** spin the arm by hand and let go. With good comp the velocity decays
      exponentially from viscous damping only: time constant ≈ `J1zz_hat / b1` = 0.0022 / 0.0005 ≈ **4.4 s**.
      Compare against comp off (should stop much sooner, roughly linearly = Coulomb friction).
- [ ] **No creep:** at rest, the arm doesn't drift or buzz on its own
- [ ] **Not overcompensated:** the arm doesn't speed up after release or settle into a limit cycle
- [ ] Set `V_TH_HI` / `V_TH_LO` just above the velocity noise floor at rest (log `Vel_Estimate` while still)
- [ ] Tune `PCT` until the flick decay matches ~4.4 s (with comp on, logged `Torque_Target` should have the
      same sign as t1d while coasting)
- [ ] Note: at `tau_cmd = 0` the STATIC branch never adds torque (below `TAU_TH`), so this tests
      the MOVING regime only, and "no creep" holds by design. Breakaway (`TAU_TH`) gets
      exercised in 2.5d / Stage 5

### 2.5c — Add the UKF, tau = 0

Same loop as 2.5b plus the UKF: read the encoder, `ukf.step(tau_prev, t1, t2)`, send `x_hat`.
(Function takes `FurataKFBase*` + `Encoder*`, like `perform_sid_with_pend_*` took `AccelKF*`.
Alternative: put this loop in the test main instead, so the wrapper stays KF-free. Both work.)

- [ ] Signs and units: `ENC_SIGN`, t1 in rad, dt2 wraps cleanly near ±pi
- [ ] **Hands-off free swing:** raise pendulum ~90°, release, don't touch. Velocities should
      match the differentiated measurement with little lag (this is the real filter check)
- [ ] Moving things by hand: positions track; velocities lag while you push (expected, unmodeled torque)
- [ ] Loop time: UKF + CAN polls + telemetry fit in 1 ms (time it with `micros()`)
- [ ] Tune `Q`, `R`

### 2.5d — UKF with a sinusoidal torque input

Same as 2.5c with `tau = A sin(2 pi f t)` (reuse `OdriveSinusoidTorqueSID`), still feeding the
pre-compensation tau to the UKF.

- [ ] Small amplitude first (below your step-SID 0.08 N-m), add a timeout or cycle limit
- [ ] A few frequencies, including near the pendulum's natural frequency
      (`sqrt(m2 g l2 / J2zz_hat)` ≈ 9.4 rad/s ≈ **1.5 Hz**). At resonance the swing grows every
      cycle and nothing in a standalone test stops it, so use a cycle limit. Going over the top is
      fine for the UKF (dt2 wraps at hanging, which normal swings already cross); and the rig has slip
      rings, so a deliberate over-the-top run is a good full-range test of the nonlinear model.
- [ ] Estimates stay smooth and track the measurement; no growing bias in t1d / t2d
- [ ] Arm stays within a safe range (keep the amplitude low or add a t1 limit that stops the motor)

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

**Test:** UKF and friction comp were already checked in Stage 2.5, so this checks that the SM
reproduces those results through `begin()` / `tick()`. Should look like 2.5c. Move the arm and pendulum by hand,
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
