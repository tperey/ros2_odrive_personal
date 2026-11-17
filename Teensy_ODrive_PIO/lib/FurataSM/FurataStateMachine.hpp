#ifndef FURATA_STATE_MACHINE_HPP
#define FURATA_STATE_MACHINE_HPP

#include <cstdint>
#include <Encoder.h>
#include <FurataKFBase.hpp>

/*
 * FurataStateMachine -- drives the ODrive Furata pendulum
 * -------------------------------------------------------
 * TEMPLATE: function bodies are TODOs to fill in (see FurataStateMachine.cpp).
 *
 * OWNERSHIP
 *   main creates the Encoder and the filter (KF / EKF / UKF) and passes them in
 *   by reference. The SM is their only user. The ODrive is the file-static
 *   module in OdriveCANWrapper -- the SM just calls its free functions.
 *   The KF and the ODrive wrapper never talk to each other; the SM moves floats.
 *
 * WHO CALLS WHAT
 *   main.setup() --> begin()   once: hardware init, seed filter, _entry(IDLE)
 *   main.loop()  --> tick()    every DT, forever
 *                      |-- _transitionTo(next)   only if state changes
 *                      |      |-- _exit(old)
 *                      |      '-- _entry(new)
 *                      '-- _during(state)        every tick
 *
 * tick() ORDER (same skeleton in every state)
 *   1. _sense()              t1 from ODrive, t2 from encoder
 *   2. _estimate()           kf.step(tau_prev, t1, t2), x_hat
 *   3. _collectEvents()      serial cmds, thresholds, faults -> event bits
 *   4. _checkTransitions()   global rules first, then per-state. Pure decision.
 *      _transitionTo()       exit(old) -> switch -> entry(new). At most ONE per tick.
 *   5. _during(state)        returns tau for the (possibly new) state
 *   6. _actuate(tau)         saturate, send (only if motor armed), store tau_prev
 *   7. _sendTelemetry()      ALWAYS, last
 *
 * RULES
 *   - Only _transitionTo() calls _exit/_entry (plus the one _entry in begin()).
 *   - _during() returns tau and NEVER changes _state.
 *
 * STATES
 *   state     | entry                     | during          | out
 *   ----------+---------------------------+-----------------+------------------------------------
 *   IDLE      | stop_motor_single()       | tau = 0         | START (+ hanging-still guard) -> SWING_UP
 *   SWING_UP  | start_motor_single(),     | _swingUpTorque  | CAUGHT -> BALANCE
 *             | reseed KF (blocked 150ms) |                 | SWINGUP_TIMEOUT -> IDLE
 *   BALANCE   | (optional) kf reset       | _lqrTorque      | FELL -> IDLE
 *   FAULT     | non-blocking idle request | tau = 0         | RESET -> IDLE
 *   GLOBAL (checked first, any state): ODRIVE_FAULT / T1_LIMIT -> FAULT,  STOP -> IDLE
 */

enum class State : uint8_t {
    IDLE     = 0,
    SWING_UP = 1,
    BALANCE  = 2,
    FAULT    = 3,
};

/*
 * EVENTS -- how they flow through one tick
 * ----------------------------------------
 *   COLLECT  _collectEvents() posts EVERY event that is true right now (serial
 *            commands, thresholds, faults) into _events. It doesn't care what
 *            state we're in -- it just reports the world.
 *   DECIDE   _checkTransitions(s) reads the whole set and returns ONE next state.
 *            It is a pure decision; _transitionTo() does the exit/entry.
 *   DROP     _events is cleared at the start of the next tick. Anything not
 *            acted on is discarded.
 *
 * WHY A SET (bitmask), NOT AN ORDERED QUEUE
 *   Everything posted in one tick happened "at the same time" at 1 ms
 *   resolution. Their order is just the order of the ifs in _collectEvents(),
 *   not real timing -- so order carries no information worth storing.
 *
 * MULTIPLE EVENTS IN ONE TICK -> PRIORITY, NOT ARRIVAL ORDER
 *   _checkTransitions checks rules from most to least important and returns
 *   on the first that fires:
 *       1. ODRIVE_FAULT / T1_LIMIT  -> FAULT       (global)
 *       2. CMD_STOP                 -> IDLE        (global)
 *       3. per-state events         (e.g. CAUGHT, FELL, CMD_START)
 *   e.g. CMD_STOP and CAUGHT in the same tick while SWING_UP -> IDLE, not BALANCE.
 *
 * AT MOST ONE TRANSITION PER TICK
 *   We do NOT transition on the first event and keep processing the rest in the
 *   new state. That would chain (SWING_UP -> BALANCE -> IDLE in one tick) and run
 *   entry actions -- some block ~150 ms -- for states we never really occupy.
 *
 * WHY DROPPING IS SAFE
 *   - Condition events (CAUGHT, FELL, T1_LIMIT, faults) are re-posted every tick
 *     while true, so nothing real is lost.
 *   - Command events (CMD_*) fire once. If they don't apply in the current state
 *     they're ignored on purpose: a "start" sent during BALANCE must not fire
 *     later by surprise.
 *   A real queue would only be needed for one-shot events that arrive BETWEEN
 *   ticks and must survive until a later state can use them. None here do.
 *
 * ADDING AN EVENT
 *   1. Add it to the enum below (before COUNT, max 32 total).
 *   2. Post it in _collectEvents().
 *   3. Handle it in _checkTransitions() at the right priority, in the states
 *      where it matters.
 */
enum class Event : uint8_t {
    CMD_START = 0,     // serial "start"
    CMD_STOP,          // serial "stop"
    CMD_RESET,         // serial "reset"
    CAUGHT,            // near upright and slow enough to catch
    FELL,              // too far from upright
    SWINGUP_TIMEOUT,   // swing-up took too long
    T1_LIMIT,          // arm angle out of range
    ODRIVE_FAULT,      // axis error or heartbeat lost
    COUNT              // keep last (must be <= 32)
};

// Why we are in FAULT. Sent in telemetry (not printed: Serial carries binary packets).
enum class FaultCode : uint8_t {
    NONE                  = 0,
    T1_LIMIT              = 1,
    ODRIVE_AXIS_ERROR     = 2,
    ODRIVE_HEARTBEAT_LOST = 3,
    ESTIMATE_INVALID      = 4,   // NaN / inf in x_hat
};

class FurataStateMachine {
public:
    static constexpr int NX = FurataKFBase::NX;

    FurataStateMachine(Encoder& enc, FurataKFBase& kf);

    void begin();   // once, from setup()
    void tick();    // every DT, from loop()

    State     state() const { return _state; }
    FaultCode fault() const { return _fault; }

private:
    // ---- Injected (owned by main) ----
    Encoder&      _enc;
    FurataKFBase& _kf;

    // ---- Machine state ----
    State     _state        = State::IDLE;
    FaultCode _fault        = FaultCode::NONE;
    uint32_t  _stateEntryMs = 0;   // millis() at last _entry()

    // ---- Per-tick data ----
    float    _t1Meas  = 0.0f;  // [rad] from ODrive
    float    _t2Meas  = 0.0f;  // [rad] RAW pendulum angle, from encoder
    float    _x[NX]   = {};    // x_hat = [t1, dt2, t1d, t2d]
    float    _tauCmd  = 0.0f;  // what _during asked for
    float    _tauSent = 0.0f;  // after saturation / arming gate
    float    _tauPrev = 0.0f;  // fed to kf.step() next tick
    uint32_t _events  = 0;     // Event bitmask, this tick only

    // ---- Serial command line buffer ----
    static constexpr uint8_t RX_BUF_LEN = 16;
    char    _rxBuf[RX_BUF_LEN] = {};
    uint8_t _rxLen = 0;

    // ---- tick() steps ----
    void  _sense();
    void  _estimate();
    void  _collectEvents();
    State _checkTransitions(State s);
    void  _transitionTo(State next);
    void  _entry(State s);
    float _during(State s);
    void  _exit(State s);
    void  _actuate(float tau);
    void  _sendTelemetry();

    // ---- Events ----
    void _post(Event e);
    bool _has(Event e) const;
    void _readSerialCommands();   // non-blocking line parser -> CMD_* events

    // ---- Helpers ----
    void     _seedFilterFromMeasurements();  // kf.reset(x0 from _t1Meas/_t2Meas, P0)
    bool     _motorArmed(State s) const;     // does this state send torque?
    uint32_t _timeInStateMs() const;

    // ---- Control laws (pure math on x_hat) ----
    float _lqrTorque(const float x[NX]) const;
    float _swingUpTorque(const float x[NX]) const;

    // ---- Math ----
    static float _saturate(float v, float lim);
    static float _wrapPi(float a);   // into (-pi, pi]
};

#endif
