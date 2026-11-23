#include "FurataStateMachine.hpp"

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include <OdriveCANWrapper.hpp>
#include <m8325s_furata/sm_config.hpp>
#include <m8325s_furata/lqr.hpp>
#include <m8325s_furata/cogging_config.hpp>

/*
 * TEMPLATE: every body below is a TODO. The stubs compile and return safe
 * defaults (tau = 0, no transitions) so you can fill in one piece at a time.
 * See the header for the call diagram and state table.
 */

static constexpr float kPi = 3.14159265358979f;

FurataStateMachine::FurataStateMachine(Encoder& enc, FurataKFBase& kf)
    : _enc(enc), _kf(kf) {
    // Nothing else: hardware isn't ready at static-init time. Do setup in begin().
}

/* ===================================================================== */
/*  PUBLIC                                                               */
/* ===================================================================== */

void FurataStateMachine::begin() {
    // TODO: one-time startup. Called ONCE from setup().
    //   1. initialize_singleODCW();     // blocks: CAN, heartbeat, vbus, abs homing,
    //                                   //   closed loop, waits for PC 0x01 byte
    //   2. load_anticogging_config(CoggingConfig::cogging_map, SMConfig::ANTICOG_PCT);
    //   3. _enc.write(0);               // pendulum must be hanging still -> t2 = 0
    //   4. _sense();                    // get first t1, t2
    //      _seedFilterFromMeasurements();
    //   5. _state = State::IDLE;  _entry(_state);   // the ONLY _entry not via _transitionTo
    //      (IDLE entry disarms the motor that initialize_singleODCW armed)
}

void FurataStateMachine::tick() {
    // TODO: the fixed skeleton. Same in every state.
    //   _events = 0;                               // events live for one tick
    //   _sense();
    //   _estimate();
    //   _collectEvents();
    //   _transitionTo(_checkTransitions(_state));  // no-op if unchanged
    //   _tauCmd = _during(_state);
    //   _actuate(_tauCmd);
    //   _sendTelemetry();                          // always, last
}

/* ===================================================================== */
/*  tick() STEPS                                                         */
/* ===================================================================== */

void FurataStateMachine::_sense() {
    // TODO:
    //   t1: poll_telemetry();                      // fresh ODrive feedback (once per tick)
    //       OdriveFeedback fb; get_odrv0_feedback(fb);
    //       _t1Meas = fb.pos_rad;                  // ZOH if !fb.fresh
    //   t2: _t2Meas = SMConfig::ENC_SIGN * _enc.read() * SMConfig::RAD_PER_PULSE;
    //       (RAW: 0 = hanging. Do NOT wrap -- the KF does the dt2 conversion.)
}

void FurataStateMachine::_estimate() {
    // TODO:
    //   _kf.step(_tauPrev, _t1Meas, _t2Meas);
    //   _kf.getState(_x);
    //
    //   _tauPrev is the torque from the PREVIOUS tick: that's what acted over the
    //   interval we're predicting across. Pass the SM's tau (before anticogging) --
    //   the model assumes cogging/static friction are already compensated.
}

void FurataStateMachine::_collectEvents() {
    // TODO: turn the world into event bits. Post everything that's true;
    // _checkTransitions decides what matters in the current state.
    //
    //   _readSerialCommands();                                   // CMD_START/STOP/RESET
    //
    //   ODrive health:
    //     OdriveHealth h; get_odrv0_health(h);
    //     if (h.axis_error != 0)                                  -> _fault = ODRIVE_AXIS_ERROR, post ODRIVE_FAULT
    //     if (h.ms_since_heartbeat > SMConfig::HEARTBEAT_TIMEOUT_MS) -> _fault = ODRIVE_HEARTBEAT_LOST, post ODRIVE_FAULT
    //   Estimate sanity:
    //     any !isfinite(_x[i])                                    -> _fault = ESTIMATE_INVALID, post ODRIVE_FAULT
    //   Arm range:
    //     fabsf(_x[0]) > SMConfig::T1_LIMIT                       -> post T1_LIMIT
    //   Pendulum (use dt2 = _x[1], t2d = _x[3]):
    //     |dt2| < CATCH_ANGLE && |t2d| < CATCH_RATE               -> post CAUGHT
    //     |dt2| > FALL_ANGLE                                      -> post FELL
    //   Timer:
    //     _timeInStateMs() > SWINGUP_TIMEOUT_MS                   -> post SWINGUP_TIMEOUT
    //
    // Only set _fault where you post the event that leads to FAULT, so the
    // code reflects the first reason. Careful not to overwrite it while in FAULT.
}

State FurataStateMachine::_checkTransitions(State s) {
    // TODO: PURE DECISION -- return the state we should be in. No side effects.
    //
    // All events in _events happened "this tick" -- treat them as a SET, decided
    // by PRIORITY, not arrival order. Return on the first rule that fires; at most
    // one transition per tick; events that don't apply in s are simply dropped
    // (_events is cleared next tick, so a stale "start" can't fire later).
    //
    //   GLOBAL (any state, checked first, in priority order):
    //     if s != FAULT && (_has(ODRIVE_FAULT) || _has(T1_LIMIT)) -> return FAULT
    //       (T1_LIMIT: set _fault = FaultCode::T1_LIMIT if not already set --
    //        acceptable exception to "no side effects", or set it in _collectEvents)
    //     if s != FAULT && s != IDLE && _has(CMD_STOP)            -> return IDLE
    //
    //   PER STATE:
    //     IDLE:     CMD_START and pendulum hanging & still
    //                 (|_x[1]| near pi, i.e. |t2| < START_MAX_T2, and |_x[3]| < START_MAX_T2D)
    //                                                        -> SWING_UP
    //     SWING_UP: CAUGHT -> BALANCE;  SWINGUP_TIMEOUT -> IDLE
    //     BALANCE:  FELL -> IDLE
    //     FAULT:    CMD_RESET -> IDLE
    //
    //   T1_LIMIT in IDLE? Motor is off, so you may want to only fault on it in
    //   armed states. Your call.
    return s;
}

void FurataStateMachine::_transitionTo(State next) {
    // TODO:
    //   if (next == _state) return;
    //   _exit(_state);
    //   _state = next;
    //   _entry(_state);
    (void)next;
}

void FurataStateMachine::_entry(State s) {
    // TODO: once, on arrival. Always: _stateEntryMs = millis();
    //
    //   IDLE:     stop_motor_single();          // BLOCKS ~150 ms+, waits for heartbeat
    //   SWING_UP: start_motor_single();         // BLOCKS ~150 ms+
    //             _sense(); _seedFilterFromMeasurements();
    //               -> the KF just "missed" 150 ms of real time; the pendulum is
    //                  hanging still so reseeding from measurement is exact enough.
    //             _stateEntryMs = millis();     // after the block, so the timeout is fair
    //   BALANCE:  optional: reseed KF / reset any integrators. Probably nothing.
    //   FAULT:    request_motor_idle_nonblocking();   // NOT stop_motor_single():
    //               it loops forever if the ODrive is dead (heartbeat-lost fault).
    //
    // Note: stop_motor_single()/start_motor_single() also call clearErrors(),
    // which will hide axis errors. Fine for IDLE/SWING_UP, which is why FAULT doesn't use them.
    (void)s;
}

float FurataStateMachine::_during(State s) {
    // TODO: every tick while in s. Return tau. NEVER change _state here.
    //   IDLE:     return 0
    //   SWING_UP: return _swingUpTorque(_x)
    //   BALANCE:  return _lqrTorque(_x)
    //   FAULT:    return 0
    (void)s;
    return 0.0f;
}

void FurataStateMachine::_exit(State s) {
    // TODO: once, on leaving. Undo what entry set up. Likely mostly empty.
    //   FAULT: _fault = FaultCode::NONE;   // cleared on the way out via RESET
    (void)s;
}

void FurataStateMachine::_actuate(float tau) {
    // TODO:
    //   _tauSent = _saturate(tau, SMConfig::TAU_MAX);
    //   if (_motorArmed(_state)) command_odrv0_torque(_tauSent, false);   // false: _sense() already polled
    //   else                     _tauSent = 0.0f;                         // axis idle -> no torque acts
    //   _tauPrev = _tauSent;     // what really acted -> next tick's KF predict
}

void FurataStateMachine::_sendTelemetry() {
    // TODO: fill a FurataTelemetry and hand it to the wrapper.
    //   FurataTelemetry tel;
    //   tel.state    = (uint8_t)_state;
    //   tel.fault    = (uint8_t)_fault;
    //   tel.tau_cmd  = _tauCmd;
    //   tel.tau_sent = _tauSent;
    //   tel.t1_meas  = _t1Meas;
    //   tel.t2_meas  = _t2Meas;
    //   memcpy(tel.x_hat, _x, sizeof(_x));
    //   sendTelemetry_furata(tel);
}

/* ===================================================================== */
/*  EVENTS                                                               */
/* ===================================================================== */

void FurataStateMachine::_post(Event e) {
    // Set the _events bit corresponding to the event
    _events |= (1u << static_cast<uint8_t>(e));
}

bool FurataStateMachine::_has(Event e) const {
    // Check if bit correspoing to event is set in _events
    return (_events >> static_cast<uint8_t>(e)) & 1u;
}

void FurataStateMachine::_readSerialCommands() {
    // TODO: NON-BLOCKING line reader (don't use check_for_shutdown_msg(): it
    // calls stop_motor_single() itself, behind the SM's back).
    //
    //   while (Serial.available() > 0) {
    //     char c = Serial.read();
    //     if (c == '\n' || c == '\r') {
    //       if (_rxLen > 0) {
    //         _rxBuf[_rxLen] = '\0';
    //         if      (strcmp(_rxBuf, "start") == 0) _post(Event::CMD_START);
    //         else if (strcmp(_rxBuf, "stop")  == 0) _post(Event::CMD_STOP);
    //         else if (strcmp(_rxBuf, "reset") == 0) _post(Event::CMD_RESET);
    //         _rxLen = 0;
    //       }
    //     } else if (_rxLen < RX_BUF_LEN - 1) {
    //       _rxBuf[_rxLen++] = c;
    //     } else {
    //       _rxLen = 0;   // overflow: drop the line
    //     }
    //   }
}

/* ===================================================================== */
/*  HELPERS                                                              */
/* ===================================================================== */

void FurataStateMachine::_seedFilterFromMeasurements() {
    // TODO:
    //   x0 = [ _t1Meas, _wrapPi(_t2Meas - kPi), 0, 0 ]   // assumes at rest
    //   P0 = diag(SMConfig::P0_DIAG)
    //   _kf.reset(x0, P0);
    //   _kf.getState(_x);      // so _x is valid before the first step
}

bool FurataStateMachine::_motorArmed(State s) const {
    // return true for states that command torque (SWING_UP, BALANCE).
    // Note motor will ignore cmds when not in closed loop control.
    // Intent is mostly to ensure _tauSent reflects this.

    // Commandable states are {SWING_UP, BALANCE}
    return (s == State::SWING_UP) || (s == State::BALANCE);
}

uint32_t FurataStateMachine::_timeInStateMs() const {
    // unsigned subtraction handles wrap
    // Times tracked in ms.
    return millis() - _stateEntryMs;
}

/* ===================================================================== */
/*  CONTROL LAWS                                                         */
/* ===================================================================== */

float FurataStateMachine::_lqrTorque(const float x[NX]) const {
    // TODO: tau = -(K . x), K from LQRConfig::K. x is already [t1, dt2, t1d, t2d],
    // dt2 already wrapped by the filter -- matches lqr.hpp ordering.
    (void)x;
    return 0.0f;
}

float FurataStateMachine::_swingUpTorque(const float x[NX]) const {
    // TODO: energy-based swing-up (or return 0 to catch by hand for now).
    //   Pendulum energy relative to upright (0 at upright, rest, < 0 below):
    //     E = 0.5 * J2 * t2d^2 + m2 g l2 (cos(t2) - 1)  ... with t2 = dt2 + pi,
    //       so cos(t2) = -cos(dt2):  E = 0.5 J2 t2d^2 - m2 g l2 (1 + cos(dt2))
    //   Pump law (Astrom-Furuta style):
    //     tau = SWINGUP_GAIN * (0 - E) * sign(t2d * cos(t2))
    //   Then clamp, and consider a term that keeps t1 from drifting toward T1_LIMIT.
    //   Verify the sign on hardware with a tiny gain first!
    (void)x;
    return 0.0f;
}

/* ===================================================================== */
/*  MATH HELPER                                                          */
/* ===================================================================== */

float FurataStateMachine::_saturate(float v, float lim) {
    if (v < -lim) {
        v = -lim;
    } else if (v > lim) {
        v = lim;
    }

    return v;
}

float FurataStateMachine::_wrapPi(float a) {
    // Wrap into (-pi, pi]. Uses atan2 of sin/cos, which maps angle into this ragne
    return atan2f(sinf(a), cosf(a));
}
