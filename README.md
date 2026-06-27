NOTES
--Code generally expects Teensy on ACM1 and Odrive on ACM0
--Cogging
1. Tried using velocity info, assuming its constant (no acceleration). But problems
----At low velocity setpoints, acceleration is absolutely not negligible.
----At high velocity setpoints, dynamics filter out cogging in the position output (can't identify it from the data), AND/OR (cogging in space)*(velcoity) > Nyquist (can't detect the spatial frequency in the sampled time). 
2. Tried bringin in accel info, but other challenges
----You have to numerically DIFFERENTIATE to bring in acceleration, so its noisy. Hard to know how much to filter velocity, acceleration, and torque (if at all)
----Again, at high speeds, can't resolve the cogging component
----But at low speeds, acceleration is caused by (and therefore included in) the COGGING, not the actual motor torque. So the linear regressino says there's is basically no inertia
----So, estiamte is no better or worse
3. I used the onboard cogging. Seemed pretty good, but should evaluate


SSH INFO
Via wifi: If your laptop is connected to same WiFi as pi, `ssh tperey-desktop.local` or `ssh tperey@tperey-desktop.local` should work.
Via Ethernet: When connected via Ethernet, at least to your Macbook, `ssh tperey@192.168.50.2` should work

NOTE: if it doesn't, and nothing else seems to work, try forcing reconnect
--On Mac
sudo ifconfig en9 down
sleep 2
sudo ifconfig en9 up
--On Linux
sudo ip link set eth0 down
sleep 2
sudo ip link set eth0 up

TEENSY CODE
`read_furata_encoder` = basic furata pendulum encoder reader, no Kalman Filter
`teensy_furata_encoder` = pendulum encoder reader, uses extended Kalman Filter based on UNFORCED dynamics (doesn't account for motor's affects, but rather just gravity physics); only outputs VELOCITY
`teensy_KF_accel` = pendulum encoder reader, uses Kalman Filter assuming constant acceleration btw timesteps (with high uncertainty in that assumption). Outputs velocity AND ACCELERATION

SRC/PS5_ODRIVE_CONTROL/PS5_ODRIVE_CONTROL
`acckf_encoder_node` = reads `teensy_KF_accel`
`encoder_node` = reads `teensy_furata_encoder`
`furata_controller` = does the actual LQR control to balance the pendulum
`integrated_furata` = `encoder_node` and `furata_controller` built into one (I think?)
`odrive_profiler` = I don't really know, maybe the hanging weight stuff?
`ps5_controller_node` = read PS5 controller?
`pst_odrive_node` = move Odrive with PS5 controller?
`velocity_furata` = velocity loop control to balance the pendulum

SYS_ID
`friction_sine_waves` = sinusoidal torque input for sys ID, WITH friction comp
`friction_tester` = not srue
`torque_sine_waves` = sinusoidal torque input for sys ID of JUST THE MOTOR AND ITS ARM
`full_sysid_torquewaves` = like `torque_sine_waves` but also reads and includes pendulum encoder info
`full_sysid_oop` = like `full_sysid_torquewaves` but includes a class to make it easy to do multi-frequency sinusoidal inputs
`linear_sysid_torque` = LINEAR torque input for sys ID; typically ran with JUST THE MOTOR (no other coponents at all!)
`postprocess_sys_id` = do the fit on JUST THE MOTOR AND ITS ARM based on output of `torque_sine_waves`
`linear_sysid_position` = LINEAR position input for sys ID (i.e. cogging comp); typically ran with JUST THE MOTOR
`cogging_cal` = runs and logs onboard cogging calibration