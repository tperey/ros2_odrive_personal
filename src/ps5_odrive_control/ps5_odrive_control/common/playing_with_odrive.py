import odrive
from odrive.enums import * # Get them all
from odrive.utils import request_state
import math
import time

# For examples
import numpy as np

LEAD_SCREW_PITCH = 5.0 # mm/rev of lead screw
# To go 75 mm, need 15 rev
# 15 rev/sec = Full travel in 1 sec

""" ODRIVE INIT FUNCS """
def get_Odrive_init(Kp = 20.0, Kv = 0.167, KvI = 0.333, Vmax =4.0, shouldErase = True, shouldReconfig = True, shouldCalibrate = True, ConType = ControlMode.POSITION_CONTROL, InType = InputMode.PASSTHROUGH):
    """
    config_Odrive_init

    Runs Odrive configuration with initial parameters
    Copied from
    
    """
    # Connect
    print("Connecting to Odrive...")
    odrv = odrive.find_sync(timeout = 30.0)
    print("Connected!")
    print("~Clearing errors!")
    odrv.clear_errors()

    # *** CONFIGURE ***

    # Erase old config
    if shouldErase:
        print("--Erasing old config...")

        try: 
            odrv.erase_configuration()
        except odrive.libodrive.DeviceLostException:
            pass
        
        print("--Rebooting...")
        try: 
            odrv.reboot()
            time.sleep(5)
        except odrive.libodrive.DeviceLostException:
            pass

        odrv = odrive.find_any()
        print("--Reconnected!")

    if shouldReconfig:
        print("----Re-configuring...")

        # Power
        odrv.config.dc_bus_overvoltage_trip_level = 50
        odrv.config.dc_bus_undervoltage_trip_level = 10.5
        odrv.config.dc_max_positive_current = 11.5
        odrv.config.dc_max_negative_current = -5
        odrv.config.brake_resistor0.enable = False

        # Motor and Thermistor
        odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
        odrv.axis0.config.motor.pole_pairs = 20
        odrv.axis0.config.motor.torque_constant = 0.0827
        odrv.axis0.config.motor.current_soft_max = 50
        odrv.axis0.config.motor.current_hard_max = 70
        odrv.axis0.config.motor.calibration_current = 10
        odrv.axis0.config.motor.resistance_calib_max_voltage = 2
        odrv.axis0.config.calibration_lockin.current = 10
        odrv.axis0.motor.motor_thermistor.config.enabled = True
        odrv.axis0.motor.motor_thermistor.config.r_ref = 10000
        odrv.axis0.motor.motor_thermistor.config.beta = 3435
        odrv.axis0.motor.motor_thermistor.config.temp_limit_lower = 110
        odrv.axis0.motor.motor_thermistor.config.temp_limit_upper = 130

        # CONTROLLER
        odrv.axis0.controller.config.control_mode = ConType
        odrv.axis0.controller.config.input_mode = InType
        odrv.axis0.controller.config.vel_limit = Vmax
        odrv.axis0.controller.config.vel_limit_tolerance = 0.3333333333333333*Vmax
        odrv.axis0.config.torque_soft_min = -math.inf
        odrv.axis0.config.torque_soft_max = math.inf

        # HOMING
        odrv.config.gpio5_mode = GpioMode.DIGITAL # Set up 5 and 6 for digital 
        #odrv.config.gpio6_mode = GpioMode.DIGITAL 
        odrv.config.gpio7_mode = GpioMode.DIGITAL # 5 = TOP, 7 = BTM

        # Gains
        odrv.axis0.controller.config.pos_gain = Kp
        odrv.axis0.controller.config.vel_gain = Kv
        odrv.axis0.controller.config.vel_integrator_gain = KvI

        # Comms and Encoders
        odrv.can.config.protocol = Protocol.NONE
        odrv.axis0.config.enable_watchdog = False
        odrv.axis0.config.load_encoder = EncoderId.ONBOARD_ENCODER0
        odrv.axis0.config.commutation_encoder = EncoderId.ONBOARD_ENCODER0
        odrv.config.enable_uart_a = False

        print("----Done reconfiguring!!!!")
        print("----Saving new config...")
        try:
            odrv.save_configuration()
        except odrive.libodrive.DeviceLostException:
            pass

        print("----Rebooting...")
        try:
            odrv.reboot()
            time.sleep(5)
        except odrive.libodrive.DeviceLostException:
            pass
        
        odrv = odrive.find_any()
        print("----Saved and reconnected!")
    
    if shouldCalibrate:
        print("------Calibrating...")
        request_state(odrv.axis0, AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
        
        while odrv.axis0.current_state != AxisState.IDLE:
            time.sleep(0.1) # Wait til calibration done

        print("------Done calibrating!!!!!!")
        print("------Saving new config...")
        try:
            odrv.save_configuration()
        except odrive.libodrive.DeviceLostException:
            pass

        print("------Rebooting...")
        try:
            odrv.reboot()
            time.sleep(5)
        except odrive.libodrive.DeviceLostException:
            pass
        
        odrv = odrive.find_any()
        print("------Saved and reconnected!")
    
    ### FINAL PRINTS ###
    print("")
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    print("~~~~~~~~~~~~~O-DRIVE READY~~~~~~~~~~~~~~~~")
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    print("")

    return odrv 

# def home_Odrive(odrv, Vmax_home = -1.0, backoff = 5.0, bo_precision = 0.01, Vmin_home = -0.4):
#     """
#     Function for homing an Odrive, assuming a single touch-off 
#     """
#     # REQUIREMENT - Moving UP is negative, moving DOWN is positive

#     orig_Vmax = odrv.axis0.controller.config.vel_limit # Save for restoring
#     orig_Vtol = odrv.axis0.controller.config.vel_limit_tolerance

#     # Put odrv in single speed mode
#     print("@@@Starting homing...")
#     request_state(odrv.axis0, AxisState.IDLE) # Put in idle state for changing stuff
#     odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL

#     request_state(odrv.axis0, AxisState.CLOSED_LOOP_CONTROL) # Put in control state
#     odrv.axis0.controller.input_vel = Vmax_home # Command constant speed

#     try: # Wait until switch triggered
#         while True:
#             gpio_vals = odrv.get_gpio_states()
#             cur_top_switch = bool(gpio_vals & (1 << 5)) # Isolate gpio_5 and store in bool

#             if not cur_top_switch: # BREAK when switch low
#                 print("Hit top switch!")
#                 break

#     except KeyboardInterrupt:
#         print("QUITTING homing bc ctrl+C")
#         request_state(odrv.axis0, AxisState.IDLE)
#         return
    
#     odrv.axis0.set_abs_pos(0.0) # Zero at current position

#     # Switch was triggered. Back off (input in mm, to precision in revs)
#     request_state(odrv.axis0, AxisState.IDLE) # Put in idle state for changing stuff
#     odrv.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
#     request_state(odrv.axis0, AxisState.CLOSED_LOOP_CONTROL) # Put in control state
#     odrv.axis0.controller.config.vel_limit = 2.0*abs(Vmin_home) # Command backoff position SLOWLY
#     odrv.axis0.controller.config.vel_limit_tolerance = 200.0*abs(Vmin_home)
#     target_backoff = backoff/LEAD_SCREW_PITCH
#     odrv.axis0.controller.input_pos = target_backoff

#     print(f"@@@Backing off to: {abs(odrv.axis0.pos_estimate - target_backoff)}")
#     try: 
#         while ( abs(odrv.axis0.pos_estimate - target_backoff) > bo_precision ): # Wait until finish backoff
#             #print(f"Still backing off: {abs(odrv.axis0.pos_estimate - target_backoff)}")
#             pass
#     except KeyboardInterrupt:
#         print("QUITTING homing bc ctrl+C")
#         request_state(odrv.axis0, AxisState.IDLE)
#         return
    
#     print("Done backing off")
#     time.sleep(0.5) # Wait a little before touching off again

#     # Now, do slow touchoff
#     print("@@@Doing SLOW touchoff...")
#     request_state(odrv.axis0, AxisState.IDLE) # Put in idle state for changing stuff
#     odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
#     request_state(odrv.axis0, AxisState.CLOSED_LOOP_CONTROL) # Put in control state
#     odrv.axis0.controller.input_vel = Vmin_home # Command constant speed

#     try: # Wait until switch triggered
#         while True:
#             gpio_vals = odrv.get_gpio_states()
#             cur_top_switch = bool(gpio_vals & (1 << 5)) # Isolate gpio_5 and store in bool

#             if not cur_top_switch: # BREAK when switch low
#                 print("Hit top switch!")
#                 break

#     except KeyboardInterrupt:
#         print("QUITTING homing bc ctrl+C")
#         request_state(odrv.axis0, AxisState.IDLE)
#         return
    
#     # Switch was triggered. Repeat back off so not stuck
#     request_state(odrv.axis0, AxisState.IDLE) # Put in idle state for changing stuff
#     odrv.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
#     request_state(odrv.axis0, AxisState.CLOSED_LOOP_CONTROL) # Put in control state
#     odrv.axis0.controller.config.vel_limit = 2.0*abs(Vmin_home) # Command backoff position SLOWLY
#     odrv.axis0.controller.config.vel_limit_tolerance = 200.0*abs(Vmin_home)
#     target_backoff = backoff/LEAD_SCREW_PITCH
#     odrv.axis0.controller.input_pos = target_backoff

#     print(f"@@@Backing off to: {abs(odrv.axis0.pos_estimate - target_backoff)}")
#     try: 
#         while ( abs(odrv.axis0.pos_estimate - target_backoff) > bo_precision ): # Wait until finish backoff
#             #print(f"Still backing off: {abs(odrv.axis0.pos_estimate - target_backoff)}")
#             pass
#     except KeyboardInterrupt:
#         print("QUITTING homing bc ctrl+C")
#         request_state(odrv.axis0, AxisState.IDLE)
#         return
    
#     print("Done backing off")
#     time.sleep(0.5) # Wait a little before touching off again


#     # Clean up
#     odrv.axis0.controller.input_vel = 0.0
#     odrv.axis0.set_abs_pos(0.0) # Zero at current position
#     request_state(odrv.axis0, AxisState.IDLE) # Return to idle
#     odrv.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
#     odrv.axis0.controller.config.vel_limit = orig_Vmax # Allow high speeds again
#     odrv.axis0.controller.config.vel_limit_tolerance = orig_Vtol

#     print("@@@ DONE Homing. Motor idle")


""" TEST FUNCS """
# Simple sine example WITH MY FUNCTIONS
def simple_sinusoid():
    odrv0 = get_Odrive_init(Vmax = 4.0)

    while True:
        try:
            request_state(odrv0.axis0, AxisState.CLOSED_LOOP_CONTROL)
            print("Axis is now in CLOSED_LOOP_CONTROL!")
            break
        except TimeoutError:
            print("Timeout, retrying...")
            time.sleep(0.5)

    p0 = odrv0.axis0.controller.input_pos # Position sinuosoid
    t0 = time.monotonic()
    try:
        while odrv0.axis0.current_state == AxisState.CLOSED_LOOP_CONTROL:
            setpoint = p0 + 0.5 * math.sin((time.monotonic() - t0) * 2) # [rev]
            print(f"goto {setpoint}")
            print(f"Actually at {odrv0.axis0.pos_estimate}")
            odrv0.axis0.controller.input_pos = setpoint
            #time.sleep(0.1)
    except KeyboardInterrupt as e:
        print(f"Keyboard interrupt")
        request_state(odrv0.axis0, AxisState.IDLE)

if __name__ == "__main__":
    simple_sinusoid()
