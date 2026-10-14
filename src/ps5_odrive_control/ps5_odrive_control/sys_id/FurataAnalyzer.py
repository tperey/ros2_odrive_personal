"""
Fit outstanding furata pendulum parameters

See paper for equations.

First, computes input torque as tau = tau_input + [cogging] - [static_friction]
to address nonlinearities
Note the cogging map was the current to hold position, i.e. OVERCOME cogging.
That means it is the NEGATIVE of cogging torque.
So it is SUBTRACTED to get the overall RHS above.

Includes options for actual torque filtering as well.

Then, Two independent methods are implemented so you
can cross-check them against each other:

  Method 1 (derivative-based, LINEAR):

  Method 2 (forward-simulation, NONLINEAR output-error):

  Other?

Both methods pool ALL N runs into one residual vector so a single param set
must explain every run simultaneously -- this is what keeps 2 parameters
from overfitting to noise in any one run.

Usage: fill in `runs` near the bottom with your real (t, theta) arrays per
run, or run as-is to see it validated on synthetic data with known J, b.

For reference:
Watch out for weirdness with Odrive sign convention.
Command may be flipped to actual commanded torque on occasion (esp without cogging comp)

Here is the log dict structure:
{
    'time': [],
    'pos': [],
    'vel': [],
    'iq_set': [],
    'iq_act': [],
    'tau_set': [],
    'tau_act': [],
    'cmd': [],
    'pend_pos': [],
    'pend_vel': [],
    'pend_acc': [],
}
"""

import json
import pickle
import numpy as np
from scipy.optimize import least_squares
from scipy.signal import savgol_filter, butter, filtfilt
from pathlib import Path
import matplotlib.pyplot as plt
from CoggingAnalyzer import COUNTS_PER_REV
from pathlib import Path

class FurataAnalyzer:

    RESID_FEATURE_NAMES = ["1", "tau", "t1d", "t2d", "t1d*|t1d|", "t2d*|t2d|", "tanh(50*t1d)", "tanh(50*t2d)",
                           "sin(t2)", "cos(t2)", "t1d^2*sin(2*t2)", "t1d*t2d*sin(2*t2)",
                           "strib_pos_t1d", "strib_neg_t1d", "strib_pos_t2d", "strib_neg_t2d"]

    def __init__(self, cog_file, tau_filter_bw = 100.0):
        # Infrastructure to combine several runs
        self.log_dict = {} # to be dict of dicts

        self.tau_filter_bw = tau_filter_bw
        self.linear_filter_bw = 0.0

        with open(cog_file, "r") as f:
            self.cog_map = np.asanyarray(json.load(f))

        # Convert everything to numpy arrays, and parse out all runs (periods between motor stops)
        self.runs_parsed = False
        self.start_dict = {}
        self.end_dict = {}
        self.runs_dict = {}

        # Config storage
        self.arm1_json = None
        self.arm2_json = None
        self.config_folder = None
        self._withFs = None

        # Learned residual correction
        self.resid_coef = None
        self.resid_feat_scale = None
        self.resid_dec = 1   # integration step (ms) the residual model was fit with

    def add_all_logs(self, top_path, identifier = "motoronly"):
        """
        Recursively search root for all folders whose name contains `identifier`.
        Add all
        """
        root = Path(top_path)
        path_list = [p for p in root.rglob("*") if p.is_dir() and identifier in p.name]
        for path in path_list:
            self.add_log(next(path.glob("*pkl")), path.name)

    def add_log(self, log_path, log_name):
        # Handle paths
        self.log_path = Path(log_path)
        with open(self.log_path, "rb") as f:
            log = pickle.load(f)

        cur_log_dict = {k: np.asarray(v) for k, v in log.items()}

        # De-cog here
        self._postprocess(cur_log_dict, log_name)
        
        self.log_dict[log_name] = cur_log_dict

    def plot_raw_all(self):
        """Plot original data from a log"""
        # For ref, here is the log dict structure
        for (name, log) in self.log_dict.items():
            self._plot_raw(name,log)

    def _plot_raw(self, name, log, allowStartStops = True):
            # Parse cmd properly
            if "cogd" in name:
                cmd = log["cmd"]
            else:
                cmd = -log["cmd"]

            # Pos/vel
            fig, axes = plt.subplots(2, 1, figsize=(12, 7.5), sharex=True)

            axes[0].plot(log["time"], log["pos"], linewidth=1, marker = ".", color = "cyan", label = "Position")
            if allowStartStops:
                if self.start_dict:
                    axes[0].scatter((log["time"])[self.start_dict[name]], (log["pos"])[self.start_dict[name]], color = "red", marker = "x", label = "Start points", s = 100)
                if self.end_dict:
                    axes[0].scatter((log["time"])[self.end_dict[name]], (log["pos"])[self.end_dict[name]], color = "purple", marker = "^", label = "End points", s = 100)
            axes[0].set_ylabel("Position (rad)")
            axes[0].set_title(f"{name} - Motor Pos/Vel Raw")
            axes[0].legend()
            axes[0].grid(True, alpha=0.3)

            axes[1].plot(log["time"], log["vel"], linewidth=1, marker = ".", color = "orange", label = "Velocity (from Odrive)")
            axes[1].set_ylabel("Velocity (rad/s)")
            axes[1].set_xlabel("Time (ms)")
            axes[1].legend()
            axes[1].grid(True, alpha=0.3)

            # Current/accel
            fig, axes = plt.subplots(2, 1, figsize=(12, 7.5), sharex=True)

            axes[0].plot(log["time"], log["iq_set"], linewidth=1, marker = ".", color = "blue", label = "Iq setpoint")
            axes[0].plot(log["time"], log["iq_act"], linewidth=1, marker = ".", color = "green", label = "Iq actual")
            axes[0].set_ylabel("Current (A)")
            axes[0].set_title(f"{name} - Current/Torque Raw")
            axes[0].legend()
            axes[0].grid(True, alpha=0.3)

            axes[1].plot(log["time"], log["tau_set"], linewidth=1, marker = ".", color = "orange", label = "Torque setpoint")
            axes[1].plot(log["time"], log["tau_set_filt"], linewidth=1, color = "red", label = "Tau set filtered")
            axes[1].plot(log["time"], log["tau_act"], linewidth=1, marker = ".", color = "gray", label = "Torque actual")
            axes[1].plot(log["time"], log["tau_act_filt"], linewidth=1, color = "black", label = "Tau actual filtered")
            axes[1].plot(log["time"], cmd, linewidth=1, color = "purple", label = "Command")
            axes[1].set_ylabel("Torque (N-m)")
            axes[1].set_xlabel("Time (ms)")
            axes[1].legend()
            axes[1].grid(True, alpha=0.3)

            # Torque, with modifications
            fig, axes = plt.subplots(3, 1, figsize=(12, 7.5), sharex=True)

            axes[0].plot(log["time"], log["accel"], linewidth=1, marker = ".", color = "red", label = "Acceleration (from filtered np.gradient)")
            axes[0].set_ylabel("Acceleration (rad/s^2)")
            axes[0].set_title(f"{name} - Acceleration and Compensated torque")
            axes[0].legend()
            axes[0].grid(True, alpha=0.3)

            axes[1].plot(log["time"], log["tau_set_compensated"], linewidth=1, marker = ".", color = "orange", label = "Torque setpoint compensated")
            axes[1].plot(log["time"], log["tau_set_compensated_filt"], linewidth=1, color = "red", label = "Tau set compensated Filtered")
            axes[1].plot(log["time"], log["tau_act_compensated"], linewidth=1, marker = ".", color = "gray", label = "Torque actual compensated")
            axes[1].plot(log["time"], log["tau_act_compensated_filt"], linewidth=1, color = "black", label = "Tau actual compensated Filtered")
            axes[1].plot(log["time"], cmd, linewidth=1, color = "purple", label = "Command")
            axes[1].set_ylabel("Torque (N-m)")
            axes[1].set_xlabel("Time (ms)")
            axes[1].legend()
            axes[1].grid(True, alpha=0.3)

            axes[2].plot(log["time"], (log["tau_set"] - cmd), label = "Delta (set vs cmd)", color = "blue")
            axes[2].plot(log["time"], (log["tau_set_compensated"] - cmd), label = "Delta (set vs cmd) AFTER decog", color = "green")
            axes[2].set_ylabel("Torque (N-m)")
            axes[2].set_xlabel("Time (ms)")
            axes[2].legend()
            axes[2].grid(True, alpha=0.3)

            # Pendulum
            fig, axes = plt.subplots(3, 1, figsize=(12, 7.5), sharex=True)

            axes[0].plot(log["time"], log["pend_pos"], linewidth=1, marker = ".", color = "blue", label = "Pendulum position (rad???)")
            axes[0].set_ylabel("Position (rad???)")
            axes[0].set_xlabel("Time (ms)")
            axes[0].legend()
            axes[0].grid(True, alpha=0.3)

            axes[1].plot(log["time"], log["pend_vel"], linewidth=1, marker = ".", color = "green", label = "Pendulum Velocity (rad/s???)")
            axes[1].set_ylabel("Velocity (rad/s???)")
            axes[1].set_xlabel("Time (ms)")
            axes[1].legend()
            axes[1].grid(True, alpha=0.3)

            axes[2].plot(log["time"], log["pend_accel"], linewidth=1, marker = ".", color = "red", label = "Pendulum Velocity (rad/s???)")
            axes[2].set_ylabel("Acceleration (rad/s^2???)")
            axes[2].set_xlabel("Time (ms)")
            axes[2].legend()
            axes[2].grid(True, alpha=0.3)
            
            fig.tight_layout()

            plt.show()

    def parse_runs(self, time_key="time", command_key="cmd", dt_thresh=3, nt_thresh = 10, zero_tol=1e-3, doPlot = True, trim = 100):
        """
        Split the logs into individual runs, based on gaps in the time vector.

        A run boundary is detected whenever the timestep between successive
        samples exceeds dt_thresh. This produces segments that alternate between
        actual runs and inter-run pauses (where the command is ~zero); pause
        segments are filtered out.

        Parameters
        ----------
        log : dict of str -> np.ndarray
            All arrays must be the same length, one entry per timestep.
        time_key : str
            Key for the time array in `log`.
        command_key : str
            Key for the commanded value used to detect zero-command pauses.
        dt_thresh : float
            Timestep gap (in same units as time_key) above which a split occurs.
        zero_tol : float
            Command magnitude below which a segment is considered "zero command"
            (and therefore a pause, not a run).

        Returns
        -------
        list of dict
            One dict per run, same keys as `log`, each value sliced to that run.
        """
        for (name, log) in self.log_dict.items():
            time = np.asarray(log[time_key])
            dt = np.diff(time)
            split_points = np.where(dt > dt_thresh)[0] + 1
            boundaries = np.concatenate(([0], split_points, [len(time)]))

            runs = []
            starts = []
            ends = []
            counter = 0
            for start, end in zip(boundaries[:-1], boundaries[1:]):
                end -= 1
                segment_cmd = np.asarray(log[command_key])[start:end]

                if (np.mean(np.abs(segment_cmd)) < zero_tol) or (len(segment_cmd) < nt_thresh):
                    continue

                # Trim `trim` samples off each end of the run to drop boundary artifacts
                trimmed_start = start + trim
                trimmed_end = end - trim
                if trimmed_end - trimmed_start < nt_thresh:
                    # Run too short after trimming; skip it rather than produce a near-empty run
                    continue

                run_dict = {k: np.asarray(v)[trimmed_start:trimmed_end] for k, v in log.items()}
                starts.append(trimmed_start)
                ends.append(trimmed_end)
                runs.append(run_dict)

                if doPlot:
                    self._plot_raw(f"{name}_{counter}", run_dict, False)
                    counter += 1

            self.start_dict[name] = starts
            self.end_dict[name] = ends
            self.runs_dict[name] = runs

        self.runs_parsed = True

    def filter_runs(self, debug = False, tau_source = "act", filter_bw = 25.0):
        """
        Uses numerical diff and filtering to generate velocities and alphas for every run
        """

        if not self.runs_parsed:
            self.parse_runs()

        for run_list in self.runs_dict.values():
            for run in run_list:

                """First, parse DATA """
                time = np.array(run["time"])
                if tau_source == "act":
                    tau_raw = np.array(run["tau_act_compensated"])
                elif tau_source == "set":
                    tau_raw = np.array(run["tau_set_compensated"])
                else:
                    raise ValueError("This is not a valid tau_source")
                t1 = np.array(run["pos"])
                t1d_raw = np.array(run["vel"])
                t2 = np.array(run["pend_pos"])
                t2d_raw = self._gradient_padded(t2, 0.001)#np.gradient(t2, 0.001, edge_order=2)  # Don't use the Kalman filter

                dt = np.mean(np.diff(time))
                if not np.allclose(np.diff(time), dt, rtol=3.0):
                    raise ValueError("Method 1 requires (approximately) uniform sampling per run; "
                                    "resample/interpolate onto a uniform grid first.")
                t1dd_raw = self._gradient_padded(t1d_raw, 0.001)#np.gradient(t1d_raw, 0.001, edge_order=2)
                t2dd_raw = self._gradient_padded(t2d_raw, 0.001)#np.gradient(t2d_raw, 0.001, edge_order=2)
            
                # Filter
                self.linear_filter_bw = filter_bw
                t1d = self._implement_filtfilt(t1d_raw, cutoff_hz=filter_bw)
                t2d = self._implement_filtfilt(t2d_raw, cutoff_hz=filter_bw)
                t1dd = self._implement_filtfilt(t1dd_raw, cutoff_hz=filter_bw)
                t2dd = self._implement_filtfilt(t2dd_raw, cutoff_hz=filter_bw)
                tau_hat = self._implement_filtfilt(tau_raw, cutoff_hz=filter_bw)

                run["vel_filt"] = t1d  # Save for eval
                run["accel_filt"] = t1dd
                run["pend_vel_filt"] = t2d
                run["pend_accel_filt"] = t2dd
                run["used_tau_filt"] = tau_hat

    def load_params_from_configs(self, config_path, withFs = False):
        config_folder = Path(config_path)
        if not withFs:
            arm1_json = config_folder / "arm1.json"
            arm2_json = config_folder / "arm2.json"
        else:
            arm1_json = config_folder / "arm1_fs.json"
            arm2_json = config_folder / "arm2_fs.json"

        self.arm1_params = json.loads(arm1_json.read_text())
        self.arm2_params = json.loads(arm2_json.read_text())

        self.arm1_json = arm1_json  # For future logging
        self.arm2_json = arm2_json
        self.config_folder = config_folder
        self._withFs = withFs

    def store_params_to_configs(self, header_path):
        if not self.arm1_json or not self.arm2_json:
            print("Must load params to store them")
            return
               
        # JSON (for Python scripts)
        self.arm1_json.write_text(json.dumps(self.arm1_params, indent=2))
        self.arm2_json.write_text(json.dumps(self.arm2_params, indent=2))

        # # HEADERS (for CPP scripts)
        self.header_path = Path(header_path)
        self.header_path.mkdir(parents=True, exist_ok=True)
        if self._withFs:
            header_file = self.header_path / "arm1_fs.hpp"
            with open(header_file, "w") as f:
                f.write("#pragma once\n\n")
                f.write("namespace Arm1Config {\n\n")
                f.write(f"constexpr float J1zz_hat = {self.arm1_params["J1zz_hat"]:.4f};\n\n")
                f.write(f"constexpr float b1 = {self.arm1_params["b1"]:.4f}; \n\n")
                f.write(f"constexpr float fs1 = {self.arm1_params["fs1"]:.4f};\n\n")
                f.write(f"constexpr float L_1 = {self.arm1_params["L_1"]:.4f};\n\n")
                f.write("} // namespace Arm1Config\n")
            header_file = self.header_path / "arm2_fs.hpp"
            with open(header_file, "w") as f:
                f.write("#pragma once\n\n")
                f.write("namespace Arm2Config {\n\n")
                f.write(f"constexpr float J2zz_hat = {self.arm2_params["J2zz_hat"]:.4f};\n\n")
                f.write(f"constexpr float J2yy_hat = {self.arm2_params["J2yy_hat"]:.4f};\n\n")
                f.write(f"constexpr float J2xx = {self.arm2_params["J2xx"]:.4f};\n\n")
                f.write(f"constexpr float b2 = {self.arm2_params["b2"]:.4f}; \n\n")
                f.write(f"constexpr float m2 = {self.arm2_params["m2"]:.4f};\n\n")
                f.write(f"constexpr float l2 = {self.arm2_params["l2"]:.4f};\n\n")
                f.write(f"constexpr float L_2 = {self.arm2_params["L_2"]:.4f};\n\n")
                f.write(f"constexpr float fs2 = {self.arm2_params["fs2"]:.4f};\n\n")
                f.write("} // namespace Arm2Config\n")
        else:
            header_file = self.header_path / "arm1.hpp"
            with open(header_file, "w") as f:
                f.write("#pragma once\n\n")
                f.write("namespace Arm1Config {\n\n")
                f.write(f"constexpr float J1zz_hat = {self.arm1_params["J1zz_hat"]:.4f};\n\n")
                f.write(f"constexpr float b1 = {self.arm1_params["b1"]:.4f}; \n\n")
                f.write(f"constexpr float fs1 = {self.arm1_params["fs1"]:.4f};\n\n")
                f.write(f"constexpr float L_1 = {self.arm1_params["L_1"]:.4f};\n\n")
                f.write("} // namespace Arm1Config\n")
            header_file = self.header_path / "arm2.hpp"
            with open(header_file, "w") as f:
                f.write("#pragma once\n\n")
                f.write("namespace Arm2Config {\n\n")
                f.write(f"constexpr float J2zz_hat = {self.arm2_params["J2zz_hat"]:.4f};\n\n")
                f.write(f"constexpr float J2yy_hat = {self.arm2_params["J2yy_hat"]:.4f};\n\n")
                f.write(f"constexpr float J2xx = {self.arm2_params["J2xx"]:.4f};\n\n")
                f.write(f"constexpr float b2 = {self.arm2_params["b2"]:.4f}; \n\n")
                f.write(f"constexpr float m2 = {self.arm2_params["m2"]:.4f};\n\n")
                f.write(f"constexpr float l2 = {self.arm2_params["l2"]:.4f};\n\n")
                f.write(f"constexpr float L_2 = {self.arm2_params["L_2"]:.4f};\n\n")
                f.write("} // namespace Arm2Config\n")

        print(f"Wrote {self.header_path}")

    # ---------------------------------------------------------------------------
    # Filtering and other helpers
    # ---------------------------------------------------------------------------
    
    def _postprocess(self, log, name):
        log["time"] -= log["time"][0]
        if "cogd" in name:
            print("---Shifting setpoint bwd by 1! Bc setpoint sent AFTER polling telemetry. So actual setpoint at N is logged at N+1")
            log["tau_set"] = np.roll(np.array(log["tau_set"]), -1)

        # Decog
        encoder_indices = ((np.round(np.array(log["pos"])*COUNTS_PER_REV) % COUNTS_PER_REV)).astype(int)
        log["tau_act_compensated"] = log["tau_act"] - self.cog_map[encoder_indices] - self._evaluate_motor_friction(log["vel"], log["tau_act"])
        log["tau_set_compensated"] = log["tau_set"] - self.cog_map[encoder_indices] - self._evaluate_motor_friction(log["vel"], log["tau_set"])

        # Filter
        filter_list = ["tau_act", "tau_set", "tau_act_compensated", "tau_set_compensated"]
        for filter in filter_list:
            log[f"{filter}_filt"] = self._implement_filtfilt(log[filter], cutoff_hz=self.tau_filter_bw)

        # Store initial accel for starting basic evals
        self._generate_accel(log)

        # Only AFTER deco (so as not to mess up map), convert units
        log["pos"] *= 2.0*np.pi
        log["vel"] *= 2.0*np.pi

    def _generate_accel(self, log):
        t = np.array(log["time"])
        t1d_raw = np.array(log["vel"])
        t2d_raw = np.array(log["pend_vel"])
        t1dd_raw = np.gradient(t1d_raw, 0.001, edge_order=2)
        t2dd_raw = np.gradient(t2d_raw, 0.001, edge_order=2)
    
        # Filter
        log["accel"] = self._implement_filtfilt(t1dd_raw, cutoff_hz=self.tau_filter_bw)
        log["pend_accel"] = self._implement_filtfilt(t2dd_raw, cutoff_hz=self.tau_filter_bw)

    def _evaluate_motor_friction(self, w, tau, eps=0.1):
        # Moving: Coulomb friction opposes velocity
        friction_moving = self.arm1_params["fs1"] * np.sign(w)

        # Near zero speed: friction opposes input torque, capped at +/- f_s
        # Not it opposes NET, so this is an approximation, but probably ok
        friction_slow = np.clip(tau, -self.arm1_params["fs1"], self.arm1_params["fs1"])

        return np.where(np.abs(w) >= eps, friction_moving, friction_slow)

    def _implement_filtfilt(self, signal, order = 2, cutoff_hz = 100, fs = 1000):
        nyquist = fs / 2.0
        normal_cutoff = cutoff_hz / nyquist
        b, a = butter(order, normal_cutoff, btype='low')
        return filtfilt(b, a, signal)

    # def cut_run(self, name_to_cut, i_to_cut):
    #     for (name, run_list) in self.runs_dict.items():
    #         if name == name_to_cut:
    #             cropped_run_list = []
    #             for i, run in enumerate(run_list):
    #                 if i != i_to_cut:
    #                     cropped_run_list.append(run)
    #                 else:
    #                     print(f"Actually tried to cut a run")
    #             self.runs_dict[name] = cropped_run_list

    # ---------------------------------------------------------------------------
    # Simulation
    # ---------------------------------------------------------------------------
    
    # """ SIMULATION """
    def _gradient_padded(self, x, dt, pad=10):
        """
        np.gradient with reflect-padding so every real sample gets a proper
        central-difference estimate -- no real sample is ever treated as
        a true array boundary.
        """
        x_padded = np.pad(x, pad, mode='reflect')
        dx_padded = np.gradient(x_padded, dt, edge_order=2)
        return dx_padded[pad:-pad]   # back to original length, no fake values kept


    def take_state_deriv(self, state, tau_hat, J2yy_hat, J2xx, J1zz_hat):
        """ Deriv of state """
        # Unpack
        t1, t1d, t2, t2d = state

        #J1zz_hat = self.arm1_params["J1zz_hat"]
        m2 = self.arm2_params["m2"]
        L1 = self.arm1_params["L_1"]
        l2 = self.arm2_params["l2"]
        b1 = self.arm1_params["b1"]
        b2 = self.arm2_params["b2"]
        J2zz_hat = self.arm2_params["J2zz_hat"]
        g = 9.81  # [m/s^2]

        fs2 = 0.0
        if 'fs2' in self.arm2_params.keys():
            fs2 = self.arm2_params['fs2']

        # Derivative
        gamma_1 = J1zz_hat + m2*(L1**2) + J2yy_hat*(np.sin(t2)**2) + J2xx*(np.cos(t2)**2)
        gamma_2 = m2*L1*l2*np.cos(t2)
        Gamma = np.array([[gamma_1, gamma_2],
                          [gamma_2, J2zz_hat]])

        phi_1 = tau_hat + m2*L1*l2*np.sin(t2)*(t2d**2) - t1d*t2d*np.sin(2*t2)*(J2yy_hat - J2xx) - b1*t1d
        phi_2 = 0.5*(t1d**2)*np.sin(2*t2)*(J2yy_hat - J2xx) - b2*t2d - g*m2*l2*np.sin(t2) - fs2*np.tanh(100*t2d)
        Phi = np.array([phi_1, phi_2])

        accel = np.linalg.solve(Gamma, Phi)

        return np.array([t1d, accel[0], t2d, accel[1]])

    def rk4_furata_step(self, state, dt, tau_hat, J2yy_hat, J2xx, J1zz_hat):
        k1 = self.take_state_deriv(state, tau_hat, J2yy_hat, J2xx, J1zz_hat)
        k2 = self.take_state_deriv(state + 0.5 * dt * k1, tau_hat, J2yy_hat, J2xx, J1zz_hat)
        k3 = self.take_state_deriv(state + 0.5 * dt * k2, tau_hat, J2yy_hat, J2xx, J1zz_hat)
        k4 = self.take_state_deriv(state + dt * k3, tau_hat, J2yy_hat, J2xx, J1zz_hat)
        return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    def simulate_furata(self, t, state0, tau_hat, J2yy_hat, J2xx, J1zz_hat):
        """RK4-integrate the diff equ at the exact (possibly non-uniform) sample times in t."""
        state = np.array(state0, dtype=float)
        out = np.empty((len(t), 4))
        out[0,:] = state
        for i in range(len(t) - 1):
            dt = t[i + 1] - t[i]
            state = self.rk4_furata_step(state, dt, tau_hat[i], J2yy_hat, J2xx, J1zz_hat)
            out[i+1, :] = state
        return out

    def simulate_runs(self):
        """ For each run, copmare actual to forward simulation """
        J2yy_hat = self.arm2_params["J2yy_hat"]
        J2xx = self.arm2_params["J2xx"]
        J1zz_hat = self.arm1_params["J1zz_hat"]
        if not self.runs_parsed:
            self.parse_runs()

        for (name, run_list) in self.runs_dict.items():
            for (i, run) in enumerate(run_list):
                # Parse
                t = np.array(run["time"])/1000.0
                tau = np.array(run["used_tau_filt"])

                t1 = np.array(run["pos"])
                t2 = np.array(run["pend_pos"])
                t1d = np.array(run["vel_filt"])
                t2d = np.array(run["pend_vel_filt"])

                state0 = np.array([t1[0], t1d[0], t2[0], t2d[0]])

                # Forward simulation
                t1dd = []
                t2dd = []
                cur_state = state0
                for i in range(len(t)):
                    cur_state = np.array([t1[i], t1d[i], t2[i], t2d[i]])
                    cur_deriv = self.take_state_deriv(cur_state, tau[i], J2yy_hat, J2xx, J1zz_hat)
                    t1dd.append(cur_deriv[1])
                    t2dd.append(cur_deriv[3])
                t1dd_sim = np.array(t1dd)
                t2dd_sim = np.array(t2dd)
                t1dd = np.array(run["accel_filt"])
                t2dd = np.array(run["pend_accel_filt"])

                out_sim = self.simulate_furata(t, state0, tau, J2yy_hat, J2xx, J1zz_hat)
                t1_sim = out_sim[:, 0]
                t1d_sim = out_sim[:, 1]
                t2_sim = out_sim[:, 2]
                t2d_sim = out_sim[:, 3]

                # Compare to actual
                residuals_t1 = (t1_sim - t1)**2
                residuals_t2 = (t2_sim - t2)**2
                residuals_t1d = (t1d_sim - t1d)**2
                residuals_t2d = (t2d_sim - t2d)**2

                # Plot MOTOR
                fig, axs = plt.subplots(5,1, figsize=(12,7.5), sharex = True)
                axs[0].plot(t, tau, label = "Tau input (compensated, filtered, used)", color = "red")
                axs[0].set_ylabel("Torque (N-m)")
                axs[0].set_title(f"Forward Simulation of {name}_{i} MOTOR")
                axs[0].legend()
                axs[0].grid(True, alpha=0.3)
                
                axs[1].plot(t, t1, label = "Measured")
                axs[1].plot(t, t1_sim, label = "Predicted")
                axs[1].set_ylabel("Position (rad)")
                axs[1].legend()
                axs[1].grid(True, alpha=0.3)

                axs[2].plot(t, t1d, label = "Measured")
                axs[2].plot(t, t1d_sim, label = "Predicted")
                axs[2].set_ylabel("Velocity (rad/s)")
                axs[2].legend()
                axs[2].grid(True, alpha=0.3)

                axs[3].plot(t, t1dd, label = "Measured")
                axs[3].plot(t, t1dd_sim, label = "Predicted")
                axs[3].set_ylabel("Accleration (rad/s^2)")
                axs[3].legend()
                axs[3].grid(True, alpha=0.3)
        
                axs[4].plot(t, residuals_t1, linewidth=1, color = "cyan", label = "Position Residual (rev^2)")
                axs[4].plot(t, residuals_t1d, linewidth=1, color = "green", label = "Velocity Residual((rev/s)^2)")
                axs[4].set_ylabel("Residual")
                axs[4].set_xlabel("Time (ms)")
                axs[4].set_title("Residuals")
                axs[4].legend()
                axs[4].grid(True, alpha=0.3)
                fig.tight_layout()

                # Plot PENDULUM
                fig, axs = plt.subplots(5,1, figsize=(12,7.5), sharex = True)
                axs[0].plot(t, tau, label = "Tau input (compensated, filtered, used)", color = "red")
                axs[0].set_ylabel("Torque (N-m)")
                axs[0].set_title(f"Forward Simulation of {name}_{i} PENDULUM")
                axs[0].legend()
                axs[0].grid(True, alpha=0.3)
                
                axs[1].plot(t, t2, marker = '.', label = "Measured")
                axs[1].plot(t, t2_sim, marker = '.', label = "Predicted")
                axs[1].set_ylabel("Position (rev)")
                axs[1].legend()
                axs[1].grid(True, alpha=0.3)

                axs[2].plot(t, t2d, marker = '.', label = "Measured")
                axs[2].plot(t, t2d_sim, marker = '.', label = "Predicted")
                axs[2].set_ylabel("Velocity (rev/s)")
                axs[2].legend()
                axs[2].grid(True, alpha=0.3)

                axs[3].plot(t, t2dd, marker = '.', label = "Measured")
                axs[3].plot(t, t2dd_sim, marker = '.', label = "Predicted")
                axs[3].set_ylabel("Accleration (rad/^2)")
                axs[3].legend()
                axs[3].grid(True, alpha=0.3)
        
                axs[4].plot(t, residuals_t2, linewidth=1, color = "cyan", label = "Position Residual (rad^2)")
                axs[4].plot(t, residuals_t2d, linewidth=1, color = "green", label = "Velocity Residual((rad/s)^2)")
                axs[4].set_ylabel("Residual")
                axs[4].set_xlabel("Time (ms)")
                axs[4].set_title("Residuals")
                axs[4].legend()
                axs[4].grid(True, alpha=0.3)
                fig.tight_layout()

                plt.show()

    # ---------------------------------------------------------------------------
    # Method 1: derivative-based, linear least squares (with bounds via least_squares)
    # ---------------------------------------------------------------------------
    def fit_linear_noJ1(self, debug = True, tau_source = "act", filter_bw = 100.0):
        """
        Uses numerical diff and filtering to get alpha.
        Returns: OptimizeResult from least_squares (result.x = [J, b, f_s])
        """

        A_blocks, y_blocks = [], []

        if not self.runs_parsed:
            self.parse_runs()

        # Get known params
        J1zz_hat = self.arm1_params["J1zz_hat"]
        m2 = self.arm2_params["m2"]
        L1 = self.arm1_params["L_1"]
        l2 = self.arm2_params["l2"]
        b1 = self.arm1_params["b1"]
        b2 = self.arm2_params["b2"]
        J2zz_hat = self.arm2_params["J2zz_hat"]
        g = 9.81  # [m/s^2]

        fs2 = 0.0
        if 'fs2' in self.arm2_params.keys():
            fs2 = self.arm2_params['fs2']

        for run_list in self.runs_dict.values():
            for run in run_list:

                """First, parse DATA """
                time = np.array(run["time"])
                if tau_source == "act":
                    tau_raw = np.array(run["tau_act_compensated"])
                elif tau_source == "set":
                    tau_raw = np.array(run["tau_set_compensated"])
                else:
                    raise ValueError("This is not a valid tau_source")
                t1 = np.array(run["pos"])
                t1d_raw = np.array(run["vel"])
                t2 = np.array(run["pend_pos"])
                t2d_raw = self._gradient_padded(t2, 0.001)#np.gradient(t2, 0.001, edge_order=2)  # Don't use the Kalman filter

                dt = np.mean(np.diff(time))
                if not np.allclose(np.diff(time), dt, rtol=3.0):
                    raise ValueError("Method 1 requires (approximately) uniform sampling per run; "
                                    "resample/interpolate onto a uniform grid first.")
                t1dd_raw = self._gradient_padded(t1d_raw, 0.001)#np.gradient(t1d_raw, 0.001, edge_order=2)
                t2dd_raw = self._gradient_padded(t2d_raw, 0.001)#np.gradient(t2d_raw, 0.001, edge_order=2)
            
                # Filter
                self.linear_filter_bw = filter_bw
                t1d = self._implement_filtfilt(t1d_raw, cutoff_hz=filter_bw)
                t2d = self._implement_filtfilt(t2d_raw, cutoff_hz=filter_bw)
                t1dd = self._implement_filtfilt(t1dd_raw, cutoff_hz=filter_bw)
                t2dd = self._implement_filtfilt(t2dd_raw, cutoff_hz=filter_bw)
                tau_hat = self._implement_filtfilt(tau_raw, cutoff_hz=filter_bw)

                run["vel_filt"] = t1d  # Save for eval
                run["accel_filt"] = t1dd
                run["pend_vel_filt"] = t2d
                run["pend_accel_filt"] = t2dd
                run["used_tau_filt"] = tau_hat

                """ Build up A """
                s2, c2, sin2 = np.sin(t2)**2, np.cos(t2)**2, np.sin(2*t2)
                a11 = t1dd*s2 + t1d*t2d*sin2
                a12 = t1dd*c2 - t1d*t2d*sin2
                a21 = -0.5*(t1d**2)*sin2
                a22 = -a21

                A_blocks.append(np.vstack(( np.column_stack([a11, a12]), np.column_stack([a21, a22]) )))

                """ Build up B """
                P0p = J1zz_hat + m2*(L1**2)
                C = m2*L1*l2
                G = g*m2*l2

                y1 = tau_hat - t1dd*P0p - t2dd*C*np.cos(t2) + C*np.sin(t2)*(t2d**2) - b1*t1d
                y2 = -t1dd*C*np.cos(t2) - t2dd*J2zz_hat - b2*t2d - G*np.sin(t2) - fs2*np.tanh(100*t2d)
                y_blocks.append(np.concatenate([y1, y2]))                            # RHS

                if debug:
                    fig, axs = plt.subplots(2, 1, figsize = (10,6), sharex = True)
                    axs[0].plot(time, t1, label = "Theta")
                    axs[0].plot(time, t1d, label = "Omega")
                    axs[0].plot(time, t1dd, label = "Alpha")
                    axs[0].grid(True, alpha=0.3)
                    axs[0].legend()
                    axs[1].plot(time, tau_hat, label = "Tau", color="black")
                    axs[1].grid(True, alpha=0.3)
                    axs[1].legend()
                    
                    plt.show()

        A = np.vstack(A_blocks)
        y = np.concatenate(y_blocks)

        # Check conditioning
        A_norm = A / np.linalg.norm(A, axis=0)
        print(f"A condition number: {np.linalg.cond(A_norm)}")

        def residuals(params):
            J2yy_hat, J2xx = params
            return A @ np.array([J2yy_hat, J2xx]) - y

        def jac(params):
            return A  # linear model -> constant Jacobian

        x0 = [J2zz_hat, J2zz_hat]
        print(f"Initial guess x0 = {x0}")
        bounds = ([m2*(l2**2), 0], [np.inf, np.inf])
        result = least_squares(residuals, x0, jac=jac, bounds=bounds)

        # Save result
        self.arm2_params["J2yy_hat"] = result.x[0]
        self.arm2_params["J2xx"] = result.x[1]
        print(result)
        return result

    def fit_linear_withJ1(self, debug = True, tau_source = "act", filter_bw = 100.0):
        """
        Uses numerical diff and filtering to get alpha.
        Returns: OptimizeResult from least_squares (result.x = [J, b, f_s])
        """

        A_blocks, y_blocks = [], []

        if not self.runs_parsed:
            self.parse_runs()

        # Get known params
        J1zz_hat = self.arm1_params["J1zz_hat"]
        m2 = self.arm2_params["m2"]
        L1 = self.arm1_params["L_1"]
        l2 = self.arm2_params["l2"]
        b1 = self.arm1_params["b1"]
        b2 = self.arm2_params["b2"]
        J2zz_hat = self.arm2_params["J2zz_hat"]
        g = 9.81  # [m/s^2]

        fs2 = 0.0
        if 'fs2' in self.arm2_params.keys():
            fs2 = self.arm2_params['fs2']

        for run_list in self.runs_dict.values():
            for run in run_list:

                """First, parse DATA """
                time = np.array(run["time"])
                if tau_source == "act":
                    tau_raw = np.array(run["tau_act_compensated"])
                elif tau_source == "set":
                    tau_raw = np.array(run["tau_set_compensated"])
                else:
                    raise ValueError("This is not a valid tau_source")
                t1 = np.array(run["pos"])
                t1d_raw = np.array(run["vel"])
                t2 = np.array(run["pend_pos"])
                t2d_raw = self._gradient_padded(t2, 0.001)#np.gradient(t2, 0.001, edge_order=2)  # Don't use the Kalman filter

                dt = np.mean(np.diff(time))
                if not np.allclose(np.diff(time), dt, rtol=3.0):
                    raise ValueError("Method 1 requires (approximately) uniform sampling per run; "
                                    "resample/interpolate onto a uniform grid first.")
                t1dd_raw = self._gradient_padded(t1d_raw, 0.001)#np.gradient(t1d_raw, 0.001, edge_order=2)
                t2dd_raw = self._gradient_padded(t2d_raw, 0.001)#np.gradient(t2d_raw, 0.001, edge_order=2)
            
                # Filter
                self.linear_filter_bw = filter_bw
                t1d = self._implement_filtfilt(t1d_raw, cutoff_hz=filter_bw)
                t2d = self._implement_filtfilt(t2d_raw, cutoff_hz=filter_bw)
                t1dd = self._implement_filtfilt(t1dd_raw, cutoff_hz=filter_bw)
                t2dd = self._implement_filtfilt(t2dd_raw, cutoff_hz=filter_bw)
                tau_hat = self._implement_filtfilt(tau_raw, cutoff_hz=filter_bw)

                run["vel_filt"] = t1d  # Save for eval
                run["accel_filt"] = t1dd
                run["pend_vel_filt"] = t2d
                run["pend_accel_filt"] = t2dd
                run["used_tau_filt"] = tau_hat

                """ Build up A """
                s2, c2, sin2 = np.sin(t2)**2, np.cos(t2)**2, np.sin(2*t2)
                a11 = t1dd
                a12 = t1dd*s2 + t1d*t2d*sin2
                a13 = t1dd*c2 - t1d*t2d*sin2
                a21 = 0*s2 # To enforce proper length
                a22 = -0.5*(t1d**2)*sin2
                a23 = -a22

                A_blocks.append(np.vstack(( np.column_stack([a11, a12, a13]), np.column_stack([a21, a22, a23]) )))

                """ Build up B """
                P0p = m2*(L1**2)
                C = m2*L1*l2
                G = g*m2*l2

                y1 = tau_hat - t1dd*P0p - t2dd*C*np.cos(t2) + C*np.sin(t2)*(t2d**2) - b1*t1d
                y2 = -t1dd*C*np.cos(t2) - t2dd*J2zz_hat - b2*t2d - G*np.sin(t2) - fs2*np.tanh(100*t2d)
                y_blocks.append(np.concatenate([y1, y2]))                            # RHS

                if debug:
                    fig, axs = plt.subplots(2, 1, figsize = (10,6), sharex = True)
                    axs[0].plot(time, t1, label = "Theta")
                    axs[0].plot(time, t1d, label = "Omega")
                    axs[0].plot(time, t1dd, label = "Alpha")
                    axs[0].grid(True, alpha=0.3)
                    axs[0].legend()
                    axs[1].plot(time, tau_hat, label = "Tau", color="black")
                    axs[1].grid(True, alpha=0.3)
                    axs[1].legend()
                    
                    plt.show()

        A = np.vstack(A_blocks)
        y = np.concatenate(y_blocks)

        # Check conditioning
        A_norm = A / np.linalg.norm(A, axis=0)
        print(f"A condition number: {np.linalg.cond(A_norm)}")

        def residuals(params):
            J1zz_hat, J2yy_hat, J2xx = params
            return A @ np.array([J1zz_hat, J2yy_hat, J2xx]) - y

        def jac(params):
            return A  # linear model -> constant Jacobian

        x0 = [J1zz_hat, J2zz_hat, J2zz_hat]
        print(f"Initial guess x0 = {x0}")
        bounds = ([0, m2*(l2**2), 0], [np.inf, np.inf, np.inf])
        result = least_squares(residuals, x0, jac=jac, bounds=bounds)

        # Save result
        self.arm1_params["J1zz_hat"] = result.x[0]  # Overwrite with new prediction
        self.arm2_params["J2yy_hat"] = result.x[1]
        self.arm2_params["J2xx"] = result.x[2]
        print(result)
        return result

    def fit_linear_combo(self, debug=True, tau_source="act", filter_bw=100.0):
        """
        Fit the two inertia combinations the data can actually distinguish:
            A = J1zz_hat + J2xx      (motor-side inertia with pendulum hanging)
            D = J2yy_hat - J2xx      (how that inertia changes with pendulum angle)
        J1zz_hat, J2yy_hat and J2xx individually are NOT identifiable from these runs.
        The current J2xx in the config is kept as-is and J1zz_hat / J2yy_hat are set
        so that A and D match the fit. Any J2xx gives identical dynamics.
        Returns: OptimizeResult from least_squares (result.x = [A, D])
        """
        A_blocks, y_blocks = [], []

        if not self.runs_parsed:
            self.parse_runs()

        # Known params
        m2 = self.arm2_params["m2"]
        L1 = self.arm1_params["L_1"]
        l2 = self.arm2_params["l2"]
        b1 = self.arm1_params["b1"]
        b2 = self.arm2_params["b2"]
        J2zz_hat = self.arm2_params["J2zz_hat"]
        J2xx = self.arm2_params["J2xx"]
        g = 9.81  # [m/s^2]
        fs2 = self.arm2_params.get("fs2", 0.0)

        for run_list in self.runs_dict.values():
            for run in run_list:

                """ First, parse DATA """
                time = np.array(run["time"])
                if tau_source == "act":
                    tau_raw = np.array(run["tau_act_compensated"])
                elif tau_source == "set":
                    tau_raw = np.array(run["tau_set_compensated"])
                else:
                    raise ValueError("This is not a valid tau_source")
                t1 = np.array(run["pos"])
                t1d_raw = np.array(run["vel"])
                t2 = np.array(run["pend_pos"])
                t2d_raw = self._gradient_padded(t2, 0.001)

                dt = np.mean(np.diff(time))
                if not np.allclose(np.diff(time), dt, rtol=3.0):
                    raise ValueError("Method 1 requires (approximately) uniform sampling per run; "
                                     "resample/interpolate onto a uniform grid first.")
                t1dd_raw = self._gradient_padded(t1d_raw, 0.001)
                t2dd_raw = self._gradient_padded(t2d_raw, 0.001)

                # Filter
                self.linear_filter_bw = filter_bw
                t1d = self._implement_filtfilt(t1d_raw, cutoff_hz=filter_bw)
                t2d = self._implement_filtfilt(t2d_raw, cutoff_hz=filter_bw)
                t1dd = self._implement_filtfilt(t1dd_raw, cutoff_hz=filter_bw)
                t2dd = self._implement_filtfilt(t2dd_raw, cutoff_hz=filter_bw)
                tau_hat = self._implement_filtfilt(tau_raw, cutoff_hz=filter_bw)

                run["vel_filt"] = t1d  # Save for eval
                run["accel_filt"] = t1dd
                run["pend_vel_filt"] = t2d
                run["pend_accel_filt"] = t2dd
                run["used_tau_filt"] = tau_hat

                """ Build up A (columns: [A, D]) """
                s2, sin2 = np.sin(t2)**2, np.sin(2*t2)
                a11 = t1dd                              # A: motor row
                a12 = t1dd*s2 + t1d*t2d*sin2            # D: motor row
                a21 = np.zeros_like(t1dd)               # A: not in pendulum row
                a22 = -0.5*(t1d**2)*sin2                # D: pendulum row

                A_blocks.append(np.vstack((np.column_stack([a11, a12]),
                                           np.column_stack([a21, a22]))))

                """ Build up y (everything that doesn't depend on A or D) """
                P0p = m2*(L1**2)
                C = m2*L1*l2
                G = g*m2*l2

                y1 = tau_hat - t1dd*P0p - t2dd*C*np.cos(t2) + C*np.sin(t2)*(t2d**2) - b1*t1d
                y2 = -t1dd*C*np.cos(t2) - t2dd*J2zz_hat - b2*t2d - G*np.sin(t2) - fs2*np.tanh(100*t2d)
                y_blocks.append(np.concatenate([y1, y2]))

                if debug:
                    fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
                    axs[0].plot(time, t1, label="Theta")
                    axs[0].plot(time, t1d, label="Omega")
                    axs[0].plot(time, t1dd, label="Alpha")
                    axs[0].grid(True, alpha=0.3)
                    axs[0].legend()
                    axs[1].plot(time, tau_hat, label="Tau", color="black")
                    axs[1].grid(True, alpha=0.3)
                    axs[1].legend()
                    plt.show()

        A = np.vstack(A_blocks)
        y = np.concatenate(y_blocks)

        # Check conditioning
        A_norm = A / np.linalg.norm(A, axis=0)
        print(f"A condition number: {np.linalg.cond(A_norm)}")

        def residuals(params):
            return A @ params - y

        def jac(params):
            return A  # linear model -> constant Jacobian

        x0 = [1e-3 + J2xx,
              1e-5 - J2xx]
        print(f"Initial guess [A, D] = {x0}")
        bounds = ([0, -np.inf], [np.inf, np.inf])   # A > 0; D left free
        result = least_squares(residuals, x0, jac=jac, bounds=bounds)
        A_fit, D_fit = result.x

        # Save the combos, and set individual params consistently (J2xx unchanged)
        self.arm1_params["A_J1_plus_J2xx"] = A_fit
        self.arm2_params["D_J2yy_minus_J2xx"] = D_fit
        self.arm1_params["J1zz_hat"] = A_fit - J2xx
        self.arm2_params["J2yy_hat"] = D_fit + J2xx
        self.arm2_params["J2xx"] = J2xx

        print("AFTER FIT")
        print(f"A = J1zz_hat + J2xx = {A_fit:.6g}")
        print(f"D = J2yy_hat - J2xx = {D_fit:.6g}")
        print(f"(Using J2xx = {J2xx:.6g} -> J1zz_hat = {A_fit - J2xx:.6g}, J2yy_hat = {D_fit + J2xx:.6g})")
        print(result)
        return result

    # # ---------------------------------------------------------------------------
    # # Method 2: forward-simulation (RK4) with coulombic friction + nonlinear least squares
    # # ---------------------------------------------------------------------------
    
    # """ FITTING """
    def fit_nonlinear(self, tau_source = "act", fit_source = "vel", filter_bw = 100.0, inclJ1 = True):
        """
        Fit to rk4 simulations
        """

        # Parse
        self.filter_runs()

        if inclJ1:
            print("~~~Including J1~~~")
            def residuals(params):
                J1zz_hat, J2yy_hat, J2xx = params
                res = []
                for run_list in self.runs_dict.values():
                    for run in run_list:
                        # Parse
                        t = np.array(run["time"])/1000.0
                        tau = np.array(run["used_tau_filt"])
        
                        t1 = np.array(run["pos"])
                        t2 = np.array(run["pend_pos"])
                        t1d = np.array(run["vel_filt"])
                        t2d = np.array(run["pend_vel_filt"])
        
                        state0 = np.array([t1[0], t1d[0], t2[0], t2d[0]])
        
                        # Forward simulation
                        out_sim = self.simulate_furata(t, state0, tau, J2yy_hat, J2xx, J1zz_hat)
                        t1_sim = out_sim[:, 0]
                        t1d_sim = out_sim[:, 1]
                        t2_sim = out_sim[:, 2]
                        t2d_sim = out_sim[:, 3]
        
                        # Compare to actual
                        residuals_t1 = (t1_sim - t1)
                        residuals_t2 = (t2_sim - t2)
                        residuals_t1d = (t1d_sim - t1d)
                        residuals_t2d = (t2d_sim - t2d)
        
                        # Compare to actual
                        if fit_source == "pos":
                            cur_res = residuals_t1
                        elif fit_source == "vel":
                            cur_res = residuals_t1d
                        elif fit_source == "pend_pos":
                            cur_res = residuals_t2
                        elif fit_source == "pend_vel":
                            cur_res = residuals_t2d
                        elif fit_source == "pend_all":
                            cur_res = np.concatenate((residuals_t2, residuals_t2d))
                        elif fit_source == "all":
                            cur_res = np.concatenate((residuals_t1, residuals_t1d, residuals_t2, residuals_t2d))
                        else:
                            raise ValueError("Invalid fit source")
                        res.append(cur_res)
                return np.concatenate(res)
        else:
            def residuals(params):
                J2yy_hat, J2xx = params
                res = []
                for run_list in self.runs_dict.values():
                    for run in run_list:
                        # Parse
                        t = np.array(run["time"])/1000.0
                        tau = np.array(run["used_tau_filt"])
        
                        t1 = np.array(run["pos"])
                        t2 = np.array(run["pend_pos"])
                        t1d = np.array(run["vel_filt"])
                        t2d = np.array(run["pend_vel_filt"])
        
                        state0 = np.array([t1[0], t1d[0], t2[0], t2d[0]])
        
                        # Forward simulation
                        out_sim = self.simulate_furata(t, state0, tau, J2yy_hat, J2xx, self.arm1_params['J1zz_hat'])
                        t1_sim = out_sim[:, 0]
                        t1d_sim = out_sim[:, 1]
                        t2_sim = out_sim[:, 2]
                        t2d_sim = out_sim[:, 3]
        
                        # Compare to actual
                        residuals_t1 = (t1_sim - t1)
                        residuals_t2 = (t2_sim - t2)
                        residuals_t1d = (t1d_sim - t1d)
                        residuals_t2d = (t2d_sim - t2d)
        
                        # Compare to actual
                        if fit_source == "pos":
                            cur_res = residuals_t1
                        elif fit_source == "vel":
                            cur_res = residuals_t1d
                        elif fit_source == "pend_pos":
                            cur_res = residuals_t2
                        elif fit_source == "pend_vel":
                            cur_res = residuals_t2d
                        elif fit_source == "pend_all":
                            cur_res = np.concatenate((residuals_t2, residuals_t2d))
                        elif fit_source == "all":
                            cur_res = np.concatenate((residuals_t1, residuals_t1d, residuals_t2, residuals_t2d))
                        else:
                            raise ValueError("Invalid fit source")
                        res.append(cur_res)
                return np.concatenate(res)

        print("Starting least_squares")
        if inclJ1:
            result = least_squares(
                residuals, x0=[self.arm1_params['J1zz_hat'], self.arm2_params['J2zz_hat'], self.arm2_params['J2zz_hat']],
                bounds=([0, self.arm2_params['m2']*(self.arm2_params['l2']**2), 0], [np.inf, np.inf, np.inf]),
                method="trf",
                loss="soft_l1",   # mild robustness to outlier samples/glitches; use 'linear' for plain LS
                x_scale=[max(self.arm1_params['J1zz_hat'], 1e-9), max(self.arm2_params['J2zz_hat'], 1e-9), max(self.arm2_params['J2zz_hat'], 1e-9)],  # helps scipy when params have very different magnitudes
                verbose=2,
            )
        else:
            result = least_squares(
                residuals, x0=[self.arm2_params['J2zz_hat'], self.arm2_params['J2zz_hat']],
                bounds=([self.arm2_params['m2']*(self.arm2_params['l2']**2), 0], [np.inf, np.inf]),
                method="trf",
                loss="soft_l1",   # mild robustness to outlier samples/glitches; use 'linear' for plain LS
                x_scale=[max(self.arm2_params['J2zz_hat'], 1e-9), max(self.arm2_params['J2zz_hat'], 1e-9)],  # helps scipy when params have very different magnitudes
                verbose=2,
            )

        # Save result
        if inclJ1:
            self.arm1_params["J1zz_hat"] = result.x[0]  # Overwrite with new prediction
            self.arm2_params["J2yy_hat"] = result.x[1]
            self.arm2_params["J2xx"] = result.x[2]
        else:
            self.arm2_params["J2yy_hat"] = result.x[0]
            self.arm2_params["J2xx"] = result.x[1]
        print(result)
        return result


    # # ---------------------------------------------------------------------------
    # # Analyzer residuals
    # # ---------------------------------------------------------------------------

    def _horizon_error_curve(self, run, min_horizon_guarantee=1000, stride=1000):
        t = np.array(run["time"]) / 1000.0
        tau = np.array(run["used_tau_filt"])
        t1, t2 = run["pos"], run["pend_pos"]
        t1d, t2d = run["vel_filt"], run["pend_vel_filt"]

        # One-step accel residual: model accel at MEASURED state vs measured accel
        t1dd = []
        t2dd = []
        for i in range(len(t)):
            cur_state = np.array([t1[i], t1d[i], t2[i], t2d[i]])
            cur_deriv = self.take_state_deriv(cur_state, tau[i],
                                            self.arm2_params['J2yy_hat'],
                                            self.arm2_params['J2xx'],
                                            self.arm1_params['J1zz_hat'])
            t1dd.append(cur_deriv[1])
            t2dd.append(cur_deriv[3])
        errors_t1dd = np.array(t1dd) - np.array(run["accel_filt"])
        errors_t2dd = np.array(t2dd) - np.array(run["pend_accel_filt"])

        # Horizon errors (unchanged)
        errors_t1 = {h: [] for h in range(1, len(t))}
        errors_t2 = {h: [] for h in range(1, len(t))}

        for start in range(0, max(len(t) - min_horizon_guarantee, 1), stride):
            print(start)
            state0 = np.array([t1[start], t1d[start], t2[start], t2d[start]])
            out = self.simulate_furata(t[start:], state0, tau[start:],
                                    self.arm2_params['J2yy_hat'],
                                    self.arm2_params['J2xx'],
                                    self.arm1_params['J1zz_hat'])
            max_h_this_start = len(out)
            for h in range(1, max_h_this_start):
                errors_t1[h].append(out[h, 0] - t1[start + h])
                errors_t2[h].append(out[h, 2] - t2[start + h])

        return errors_t1, errors_t2, errors_t1dd, errors_t2dd

    def plot_run_error_horizons(self, debug=False, save_dir=None):
        """
        Computes horizon error curves (position) and one-step residuals (accel)
        for every run, optionally plotting each run individually (debug=True),
        and always producing aggregate plots pooled across ALL runs.
          Fig 1: MOTOR     - position RMSE vs horizon, accel residual vs time since run start
          Fig 2: PENDULUM  - position RMSE vs horizon, accel residual vs time since run start
        """
        if not self.runs_parsed:
            self.parse_runs()
        self.filter_runs()

        # Accumulators, pooled across every run
        pooled_t1 = {}     # horizon -> list of position errors
        pooled_t2 = {}
        pooled_t1dd = {}   # sample index from run start -> list of accel residuals
        pooled_t2dd = {}

        for (name, run_list) in self.runs_dict.items():
            for (i, run) in enumerate(run_list):

                print(f"Generating horizon error curve for {name}_{i}...")
                errors_t1, errors_t2, errors_t1dd, errors_t2dd = self._horizon_error_curve(run)

                # Pool position errors by horizon
                for h, e in errors_t1.items():
                    if len(e) > 0:
                        pooled_t1.setdefault(h, []).extend(e)
                for h, e in errors_t2.items():
                    if len(e) > 0:
                        pooled_t2.setdefault(h, []).extend(e)

                # Pool accel residuals by sample index from run start
                for k, e in enumerate(errors_t1dd):
                    pooled_t1dd.setdefault(k, []).append(e)
                for k, e in enumerate(errors_t2dd):
                    pooled_t2dd.setdefault(k, []).append(e)

                if debug:
                    hed_t1 = {h: np.sqrt(np.mean(np.array(e)**2)) for h, e in errors_t1.items() if len(e) > 0}
                    hed_t2 = {h: np.sqrt(np.mean(np.array(e)**2)) for h, e in errors_t2.items() if len(e) > 0}
                    h1 = sorted(hed_t1.keys())
                    h2 = sorted(hed_t2.keys())
                    t_run = (np.array(run["time"]) - run["time"][0]) / 1000.0

                    fig, axs = plt.subplots(2, 2, figsize=(14, 7.5), sharex = True)
                    axs[0, 0].plot(np.array(h1) / 1000.0, [hed_t1[h] for h in h1], color="red")
                    axs[0, 0].set_title(f"{name}_{i} MOTOR position RMSE vs horizon")
                    axs[0, 0].set_xlabel("Horizon (s)")
                    axs[0, 0].set_ylabel("RMSE (rad)")

                    axs[0, 1].plot(t_run, errors_t1dd, color="red")
                    axs[0, 1].axhline(np.sqrt(np.mean(np.array(errors_t1dd)**2) ), color = "black", label = "RMSE")
                    axs[0, 1].set_title(f"{name}_{i} MOTOR accel residual")
                    axs[0, 1].set_xlabel("Time since run start (s)")
                    axs[0, 1].set_ylabel("Model - measured (rad/s^2)")

                    axs[1, 0].plot(np.array(h2) / 1000.0, [hed_t2[h] for h in h2], color="blue")
                    axs[1, 0].set_title(f"{name}_{i} PENDULUM position RMSE vs horizon")
                    axs[1, 0].set_xlabel("Horizon (s)")
                    axs[1, 0].set_ylabel("RMSE (rad)")

                    axs[1, 1].plot(t_run, errors_t2dd, color="blue")
                    axs[1, 1].axhline(np.sqrt(np.mean(np.array(errors_t2dd)**2) ), color = "purple", label = "RMSE")
                    axs[1, 1].set_title(f"{name}_{i} PENDULUM accel residual")
                    axs[1, 1].set_xlabel("Time since run start (s)")
                    axs[1, 1].set_ylabel("Model - measured (rad/s^2)")

                    for ax in axs.flat:
                        ax.grid(True, alpha=0.3)
                    fig.tight_layout()
                    plt.show()

        # --- Aggregates: position ---
        sorted_h1 = sorted(pooled_t1.keys())
        sorted_h2 = sorted(pooled_t2.keys())

        agg_horizons_t1 = np.array(sorted_h1) / 1000.0
        agg_rmse_t1 = np.array([np.sqrt(np.mean(np.array(pooled_t1[h])**2)) for h in sorted_h1])

        agg_horizons_t2 = np.array(sorted_h2) / 1000.0
        agg_rmse_t2 = np.array([np.sqrt(np.mean(np.array(pooled_t2[h])**2)) for h in sorted_h2])

        # --- Aggregates: accel (scatter of every run's point at each time, plus RMSE across runs) ---
        def _accel_agg(pooled):
            ks = sorted(pooled.keys())
            scatter_t = np.concatenate([np.full(len(pooled[k]), k / 1000.0) for k in ks])
            scatter_e = np.concatenate([np.array(pooled[k]) for k in ks])
            t_line = np.array(ks) / 1000.0
            rmse_line = np.array([np.sqrt(np.mean(np.array(pooled[k])**2)) for k in ks])
            return scatter_t, scatter_e, t_line, rmse_line

        sc_t1, sc_e1, t_line1, rmse_line1 = _accel_agg(pooled_t1dd)
        sc_t2, sc_e2, t_line2, rmse_line2 = _accel_agg(pooled_t2dd)
        print(f"Overall accel RMSE: motor {np.sqrt(np.mean(sc_e1**2)):.3f}, "
              f"pendulum {np.sqrt(np.mean(sc_e2**2)):.3f} (rad/s^2)")

        # --- Fig 1: MOTOR ---
        fig1, axs = plt.subplots(2, 1, figsize=(12, 7.5))
        axs[0].plot(agg_horizons_t1, agg_rmse_t1, color="red")
        axs[0].set_title("MOTOR position RMSE vs horizon (pooled across all runs)")
        axs[0].set_xlabel("Horizon (s)")
        axs[0].set_ylabel("RMSE (rad)")
        axs[0].grid(True, alpha=0.3)

        axs[1].scatter(sc_t1, sc_e1, s=1, color="red", alpha=0.2, label="Each run")
        axs[1].plot(t_line1, rmse_line1, color="black", linewidth=1, label="RMSE across runs")
        axs[1].set_title("MOTOR accel residual vs time since run start (all runs)")
        axs[1].set_xlabel("Time since run start (s)")
        axs[1].set_ylabel("Model - measured (rad/s^2)")
        axs[1].legend()
        axs[1].grid(True, alpha=0.3)
        fig1.tight_layout()

        # --- Fig 2: PENDULUM ---
        fig2, axs = plt.subplots(2, 1, figsize=(12, 7.5))
        axs[0].plot(agg_horizons_t2, agg_rmse_t2, color="blue")
        axs[0].set_title("PENDULUM position RMSE vs horizon (pooled across all runs)")
        axs[0].set_xlabel("Horizon (s)")
        axs[0].set_ylabel("RMSE (rad)")
        axs[0].grid(True, alpha=0.3)

        axs[1].scatter(sc_t2, sc_e2, s=1, color="blue", alpha=0.2, label="Each run")
        axs[1].plot(t_line2, rmse_line2, color="black", linewidth=1, label="RMSE across runs")
        axs[1].set_title("PENDULUM accel residual vs time since run start (all runs)")
        axs[1].set_xlabel("Time since run start (s)")
        axs[1].set_ylabel("Model - measured (rad/s^2)")
        axs[1].legend()
        axs[1].grid(True, alpha=0.3)
        fig2.tight_layout()

        if save_dir is not None:
            fig1.savefig(Path(save_dir) / "residuals_MOTOR.png", dpi=150)
            fig2.savefig(Path(save_dir) / "residuals_PENDULUM.png", dpi=150)
        plt.show()

        # --- Save to pkl ---
        if save_dir is not None:
            agg_dict = {
                'agg_horizons_t1': agg_horizons_t1,
                'agg_rmse_t1': agg_rmse_t1,
                'agg_horizons_t2': agg_horizons_t2,
                'agg_rmse_t2': agg_rmse_t2,
                'accel_t1dd': t_line1,
                'accel_rmse_t1dd': rmse_line1,
                'accel_t2dd': t_line2,
                'accel_rmse_t2dd': rmse_line2,
            }
            with open(Path(save_dir) / "horizon_errors.pkl", "wb") as f:
                pickle.dump(agg_dict, f)

    """ Learned residual correction (gray-box simulator) """
    @staticmethod
    def _stribeck_one_sided(w, eps = 0.1):
        decay = np.exp(-np.abs(w) / eps)
        t = np.tanh(100 * w)
        return decay * 0.5 * (1 + t), decay * 0.5 * (1 - t)

    @classmethod
    def _resid_features(cls, t1d, t2, t2d, tau):
        s2 = np.sin(2 * t2)
        cols = [
            np.ones_like(t1d), tau, t1d, t2d, t1d*np.abs(t1d), t2d*np.abs(t2d),
            np.tanh(50 * t1d), np.tanh(50 * t2d),
            np.sin(t2), np.cos(t2),
            t1d**2 * s2, t1d * t2d * s2,
        ]
        for w in (t1d, t2d):
            pos, neg = cls._stribeck_one_sided(w, 0.1)
            cols += [pos, neg]
        return np.column_stack(cols)

    def _resid_accel(self, t1d, t2, t2d, tau, coef):
        F = self._resid_features(t1d, t2, t2d, tau) / self.resid_feat_scale
        nf = F.shape[1]
        return F @ coef[:nf], F @ coef[nf:]

    def _deriv_batch(self, X, tau, coef):
        """Vectorized dynamics: X is (N, 4), tau is (N,)."""
        t1, t1d, t2, t2d = X.T
        J1 = self.arm1_params["J1zz_hat"]; L1 = self.arm1_params["L_1"]; b1 = self.arm1_params["b1"]
        J2yy = self.arm2_params["J2yy_hat"]; J2xx = self.arm2_params["J2xx"]; J2zz = self.arm2_params["J2zz_hat"]
        m2 = self.arm2_params["m2"]; l2 = self.arm2_params["l2"]; b2 = self.arm2_params["b2"]
        fs2 = self.arm2_params.get("fs2", 0.0); g = 9.81

        s, c, s2 = np.sin(t2), np.cos(t2), np.sin(2 * t2)
        g1 = J1 + m2 * L1**2 + J2yy * s**2 + J2xx * c**2
        g2 = m2 * L1 * l2 * c
        p1 = tau + m2 * L1 * l2 * s * t2d**2 - t1d * t2d * s2 * (J2yy - J2xx) - b1 * t1d
        p2 = 0.5 * t1d**2 * s2 * (J2yy - J2xx) - b2 * t2d - g * m2 * l2 * s - fs2 * np.tanh(100 * t2d)

        det = g1 * J2zz - g2**2
        a1 = (J2zz * p1 - g2 * p2) / det
        a2 = (g1 * p2 - g2 * p1) / det

        if coef is not None:
            r1, r2 = self._resid_accel(t1d, t2, t2d, tau, coef)
            a1 = a1 + r1
            a2 = a2 + r2
        return np.column_stack([t1d, a1, t2d, a2])

    def _rollout_batch(self, X0, TAU, coef, dt=0.001):
        """RK4 rollouts of N windows in parallel. X0: (N, 4), TAU: (N, W). Returns (N, W, 4)."""
        N, W = TAU.shape
        out = np.empty((N, W, 4))
        X = X0.copy()
        out[:, 0] = X
        for k in range(W - 1):
            u = TAU[:, k]
            k1 = self._deriv_batch(X, u, coef)
            k2 = self._deriv_batch(X + 0.5 * dt * k1, u, coef)
            k3 = self._deriv_batch(X + 0.5 * dt * k2, u, coef)
            k4 = self._deriv_batch(X + dt * k3, u, coef)
            X = X + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
            out[:, k + 1] = X
        return out

    # ---------------------------------------------------------------------------
    # Residual model: windows, fit, save/load, compare
    # ---------------------------------------------------------------------------
    def _build_windows(self, window_s, stride_s, dec=1):
        """
        Cut every run into fixed-length windows, sampled every `dec` ms.
        Torque uses zero-order hold: the sample at the start of each step is held
        for the whole step, so TAU[:, k] is the torque applied between state k and k+1.
        Returns X0 (N,4), TAU/T1/T2/T1D/T2D (N, W/dec).
        """
        W, S = int(window_s * 1000), int(stride_s * 1000)
        W -= W % dec                                     # window length divisible by dec
        X0, TAU, T1, T2, T1D, T2D = [], [], [], [], [], []
        for rl in self.runs_dict.values():
            for run in rl:
                tau = np.asarray(run["used_tau_filt"])
                t1, t2 = np.asarray(run["pos"]), np.asarray(run["pend_pos"])
                t1d, t2d = np.asarray(run["vel_filt"]), np.asarray(run["pend_vel_filt"])
                for s in range(0, len(tau) - W, S):
                    X0.append([t1[s], t1d[s], t2[s], t2d[s]])
                    TAU.append(tau[s:s+W:dec])                       # ZOH: sample at step start
                    T1.append(t1[s:s+W:dec]);   T2.append(t2[s:s+W:dec])
                    T1D.append(t1d[s:s+W:dec]); T2D.append(t2d[s:s+W:dec])
        return tuple(np.array(a) for a in (X0, TAU, T1, T2, T1D, T2D))

    def fit_residual_model(self, window_s=1.0, stride_s=0.25, lam=1e-4, error_weights=(1.0, 1.0),
                           ftol_lsq=1e-7, loss_lsq="soft_l1", use_vel=False, dec=1):
        """
        Learn a correction to the physics accelerations that minimizes multi-step
        rollout error. Physics params stay as-is; only resid_coef is fit.
        dec: integration step in ms (ZOH torque over each step). dec=5 is ~5x faster.
        """
        self.filter_runs()
        X0, TAU, T1, T2, T1D, T2D = self._build_windows(window_s, stride_s, dec=dec)
        dt = dec * 1e-3
        print(f"Fitting residual model on {len(X0)} windows of {window_s}s, dt = {dec} ms")

        # Feature normalization (from all samples) so coefficients are well-conditioned
        F_all = self._resid_features(T1D.ravel(), T2.ravel(), T2D.ravel(), TAU.ravel())
        self.resid_feat_scale = np.std(F_all, axis=0) + 1e-9
        self.resid_feat_scale[0] = 1.0   # bias column has zero std
        nf = F_all.shape[1]

        # Error normalization so motor and pendulum contribute comparably
        s1 = np.std(T1 - T1[:, [0]]) + 1e-6
        s2 = np.std(T2 - T2[:, [0]]) + 1e-6
        s1d, s2d = np.std(T1D) + 1e-6, np.std(T2D) + 1e-6

        best = {"cost": np.inf, "calls": 0}

        def residuals(coef):
            out = self._rollout_batch(X0, TAU, coef, dt=dt)
            m_err = error_weights[0] * (out[:, :, 0] - T1) / s1
            p_err = error_weights[1] * (out[:, :, 2] - T2) / s2

            # Progress print: WEIGHTED costs (what the optimizer balances), only on meaningful improvement
            best["calls"] += 1
            motor_cost, pend_cost = np.sum(m_err**2), np.sum(p_err**2)
            total = motor_cost + pend_cost
            if total < best["cost"] * (1 - 1e-6):
                best["cost"] = total
                print(f"         [call {best['calls']:4d}] weighted: motor {motor_cost:.4e}  pend {pend_cost:.4e}  "
                      f"(pend share {pend_cost / total:.1%})  |  unweighted pend {pend_cost / error_weights[1]**2:.4e}")

            r = [m_err.ravel(), p_err.ravel()]
            if use_vel:
                r += [(error_weights[0] * (out[:, :, 1] - T1D) / s1d).ravel(),
                      (error_weights[1] * (out[:, :, 3] - T2D) / s2d).ravel()]
            r.append(np.sqrt(lam * len(r[0])) * coef)   # L2 penalty on weights
            return np.concatenate(r)

        x0 = np.zeros(2 * nf) if self.resid_coef is None else self.resid_coef
        result = least_squares(residuals, x0, method="trf", loss=loss_lsq,
                               x_scale="jac", verbose=2, ftol=ftol_lsq)
        self.resid_coef = result.x
        self.resid_dec = dec

        # Checkpoint immediately so a later crash can't lose a long fit
        ckpt_dir = Path(self.config_folder) if self.config_folder else Path(".")
        with open(ckpt_dir / "resid_checkpoint.pkl", "wb") as f:
            pickle.dump({"resid_coef": self.resid_coef,
                         "resid_feat_scale": self.resid_feat_scale,
                         "dec": dec}, f)
        print(f"Checkpointed residual fit to {ckpt_dir / 'resid_checkpoint.pkl'}")
        return result

    def save_residual_model(self, save_dir, name="residual_model", header_path=None):
        """
        Saves the learned correction (pkl + json), along with the integration step
        and a snapshot of the physics params it was fit on. Optionally writes a C++ header.
        """
        if self.resid_coef is None:
            print("No residual model fit yet - nothing to save")
            return

        save_dir = Path(save_dir)
        save_dir.mkdir(parents=True, exist_ok=True)
        model = {
            "resid_coef": np.asarray(self.resid_coef).tolist(),
            "resid_feat_scale": np.asarray(self.resid_feat_scale).tolist(),
            "feature_names": self.RESID_FEATURE_NAMES,
            "dec": int(self.resid_dec),
            "arm1_params": {k: float(v) for k, v in self.arm1_params.items() if isinstance(v, (int, float))},
            "arm2_params": {k: float(v) for k, v in self.arm2_params.items() if isinstance(v, (int, float))},
        }
        with open(save_dir / f"{name}.pkl", "wb") as f:
            pickle.dump(model, f)
        (save_dir / f"{name}.json").write_text(json.dumps(model, indent=2))
        print(f"Saved residual model to {save_dir / name}.pkl/.json")

        if header_path is not None:
            header_path = Path(header_path)
            header_path.mkdir(parents=True, exist_ok=True)
            nf = len(self.RESID_FEATURE_NAMES)
            coef_str = ", ".join(f"{c:.8e}f" for c in model["resid_coef"])
            scale_str = ", ".join(f"{s:.8e}f" for s in model["resid_feat_scale"])
            with open(header_path / f"{name}.hpp", "w") as f:
                f.write("#pragma once\n\n")
                f.write("namespace ResidualModel {\n\n")
                f.write(f"// Feature order: {', '.join(self.RESID_FEATURE_NAMES)}\n")
                f.write(f"// accel_corr_motor = sum_j (feat_j / feat_scale[j]) * coef[j]\n")
                f.write(f"// accel_corr_pend  = sum_j (feat_j / feat_scale[j]) * coef[{nf} + j]\n")
                f.write(f"// Fit with integration step dt = {model['dec']} ms (ZOH torque)\n\n")
                f.write(f"constexpr int NF = {nf};\n\n")
                f.write(f"constexpr int FIT_DT_MS = {model['dec']};\n\n")
                f.write(f"constexpr float coef[{2 * nf}] = {{{coef_str}}};\n\n")
                f.write(f"constexpr float feat_scale[{nf}] = {{{scale_str}}};\n\n")
                f.write("} // namespace ResidualModel\n")
            print(f"Wrote {header_path / name}.hpp")

    def load_residual_model(self, path, check_params=True):
        """
        Loads a saved correction. Call AFTER load_params_from_configs so the
        physics-param check is meaningful: the correction is only valid on top of
        the physics params it was fit with.
        """
        with open(path, "rb") as f:
            model = pickle.load(f)
        self.resid_coef = np.asarray(model["resid_coef"])
        self.resid_feat_scale = np.asarray(model["resid_feat_scale"])
        self.resid_dec = int(model.get("dec", 1))

        if check_params and "arm1_params" in model:
            for arm_key, cur in (("arm1_params", self.arm1_params), ("arm2_params", self.arm2_params)):
                for k, v in model[arm_key].items():
                    if k in cur and not np.isclose(cur[k], v, rtol=1e-6):
                        print(f"WARNING: {k} = {cur[k]:.6g} now, but residual model was fit with {v:.6g}")
        print(f"Loaded residual model from {path} (fit with dt = {self.resid_dec} ms)")

    def compare_residual_model(self, sim_s=None, save_dir=None, dec=1, show=True):
        """
        For each run, forward-simulate from the measured initial state with:
          - physics only (the original fit)
          - physics + learned residual correction
        and plot both against the measured positions, with error underneath.

        sim_s: optional cap on simulated duration (s). None = whole run.
        dec:   integration step in ms (ZOH torque). Use 1 to check behaviour at the
               rate the real controller runs, or self.resid_dec to match training.
        show:  if False, figures are saved (if save_dir) and closed without blocking.
        """
        if self.resid_coef is None:
            raise ValueError("No residual model - fit or load one first")
        if not self.runs_parsed:
            self.parse_runs()
        self.filter_runs()
        dt = dec * 1e-3
        rms = lambda e: np.sqrt(np.mean(e**2))

        summary = []
        for (name, run_list) in self.runs_dict.items():
            for (i, run) in enumerate(run_list):
                tau = np.asarray(run["used_tau_filt"])
                t1, t2 = np.asarray(run["pos"]), np.asarray(run["pend_pos"])
                t1d, t2d = np.asarray(run["vel_filt"]), np.asarray(run["pend_vel_filt"])

                n = len(tau) if sim_s is None else min(len(tau), int(sim_s * 1000))
                n -= n % dec
                if n < 2 * dec:
                    continue
                t = (np.asarray(run["time"][:n:dec]) - run["time"][0]) / 1000.0
                m1, m2 = t1[:n:dec], t2[:n:dec]                               # measured, on sim grid

                X0 = np.array([[t1[0], t1d[0], t2[0], t2d[0]]])
                TAU = tau[:n:dec][None, :]                                     # ZOH: sample at step start
                out_phys = self._rollout_batch(X0, TAU, None, dt=dt)[0]
                out_resid = self._rollout_batch(X0, TAU, self.resid_coef, dt=dt)[0]

                err1_phys, err1_resid = out_phys[:, 0] - m1, out_resid[:, 0] - m1
                err2_phys, err2_resid = out_phys[:, 2] - m2, out_resid[:, 2] - m2
                summary.append((f"{name}_{i}", rms(err1_phys), rms(err1_resid),
                                rms(err2_phys), rms(err2_resid)))

                fig, axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True,
                                        gridspec_kw={"height_ratios": [2, 1, 2, 1]})

                axs[0].plot(t, m1, color="black", linewidth=1.5, label="Measured")
                axs[0].plot(t, out_phys[:, 0], color="tab:orange", linewidth=1, label="Physics only")
                axs[0].plot(t, out_resid[:, 0], color="tab:green", linewidth=1, label="Physics + residual")
                axs[0].set_ylabel("Motor pos (rad)")
                axs[0].set_title(f"{name}_{i}: forward simulation comparison (dt = {dec} ms)")
                axs[0].legend()

                axs[1].plot(t, err1_phys, color="tab:orange", linewidth=1,
                            label=f"Physics only (RMS {rms(err1_phys):.3f})")
                axs[1].plot(t, err1_resid, color="tab:green", linewidth=1,
                            label=f"Physics + residual (RMS {rms(err1_resid):.3f})")
                axs[1].axhline(0, color="black", linewidth=0.5)
                axs[1].set_ylabel("Motor err (rad)")
                axs[1].legend()

                axs[2].plot(t, m2, color="black", linewidth=1.5, label="Measured")
                axs[2].plot(t, out_phys[:, 2], color="tab:orange", linewidth=1, label="Physics only")
                axs[2].plot(t, out_resid[:, 2], color="tab:green", linewidth=1, label="Physics + residual")
                axs[2].set_ylabel("Pend pos (rad)")
                axs[2].legend()

                axs[3].plot(t, err2_phys, color="tab:orange", linewidth=1,
                            label=f"Physics only (RMS {rms(err2_phys):.3f})")
                axs[3].plot(t, err2_resid, color="tab:green", linewidth=1,
                            label=f"Physics + residual (RMS {rms(err2_resid):.3f})")
                axs[3].axhline(0, color="black", linewidth=0.5)
                axs[3].set_ylabel("Pend err (rad)")
                axs[3].set_xlabel("Time since run start (s)")
                axs[3].legend()

                for ax in axs:
                    ax.grid(True, alpha=0.3)
                fig.tight_layout()

                if save_dir is not None:
                    fig.savefig(Path(save_dir) / f"resid_compare_{name}_{i}_dt{dec}.png", dpi=150)
                if show:
                    plt.show()
                else:
                    plt.close(fig)

        # Summary table
        print(f"\n{'Run':<40}{'Motor phys':>12}{'Motor +res':>12}{'Pend phys':>12}{'Pend +res':>12}")
        for row in summary:
            print(f"{row[0]:<40}{row[1]:>12.4f}{row[2]:>12.4f}{row[3]:>12.4f}{row[4]:>12.4f}")
        if summary:
            arr = np.array([r[1:] for r in summary])
            print(f"{'MEAN':<40}{arr[:,0].mean():>12.4f}{arr[:,1].mean():>12.4f}"
                  f"{arr[:,2].mean():>12.4f}{arr[:,3].mean():>12.4f}")
        return summary


# ---------------------------------------------------------------------------
# Fit dat data
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    base_path = "/Users/trevorperey/Desktop/PersonalProjects/ros2_odrive_personal/src/ps5_odrive_control/ps5_odrive_control/logs/"
    cog_path = "/Users/trevorperey/Desktop/PersonalProjects/ros2_odrive_personal/config/m8325s_furata/cogging_map.json"
    config_path = "/Users/trevorperey/Desktop/PersonalProjects/ros2_odrive_personal/config/m8325s_furata"
    header_path = "/Users/trevorperey/Desktop/PersonalProjects/ros2_odrive_personal/Teensy_ODrive_PIO/lib/config/m8325s_furata/"
    one_example = base_path + "sinetau_nocog_motoronly_001/logs_20260823_211700.pkl"

    analyzer = FurataAnalyzer(cog_path, tau_filter_bw=25)
    analyzer.load_params_from_configs(config_path=config_path, withFs=True)
    analyzer.add_all_logs(base_path, identifier="all")
    analyzer.parse_runs(doPlot=False, trim = 10)
    #analyzer.plot_raw_all()

    ### LINEAR FIT ###
    print("-----Linear (with J1)-----")
    result = analyzer.fit_linear_combo(debug=False, filter_bw = 25)
    analyzer.store_params_to_configs(header_path=header_path)
    #analyzer.simulate_runs()

    # # Check residuals
    # analyzer.plot_run_error_horizons(debug=False, save_dir=config_path)

    # # Fit and test residual model
    # print(f"fit with stride_s = 0.5 = {analyzer.fit_residual_model(window_s=0.5, stride_s=0.5, use_vel=False)}")
    # analyzer.save_residual_model(config_path, header_path=header_path)
    # print(f"fit with stride_s = 1.5 = {analyzer.fit_residual_model(window_s=1.5, stride_s=1.5, use_vel=False)}")
    # analyzer.save_residual_model(config_path, header_path=header_path)
    # analyzer.compare_residual_model()

    # # TEST
    # print("~~~Moving to tester")
    # tester = FurataAnalyzer(cog_path, tau_filter_bw=25)
    # tester.load_params_from_configs(config_path=config_path, withFs=True)
    # tester.add_all_logs(base_path, identifier="generated")
    # tester.parse_runs(doPlot=False, trim = 10)
    # tester.filter_runs()
    # tester.simulate_runs()

    # Fit residuals on TEST
    print("~~~Moving to tester")
    tester = FurataAnalyzer(cog_path, tau_filter_bw=25)
    tester.load_params_from_configs(config_path=config_path, withFs=True)
    tester.add_all_logs(base_path, identifier="generated")
    tester.parse_runs(doPlot=False, trim = 10)
    tester.filter_runs()
    #tester.simulate_runs()

    common = dict(error_weights=(1.0, 2.3), loss_lsq="linear", use_vel=False, dec=5, lam=1e-4, ftol_lsq=1e-6)
    for w in [0.5, 1.5, 3.0, 4.0, 5.0]:  # Fit to windows of progressive sizes, with warm starts and intermediate saves
        print(f"\n===== Window {w} s =====")
        cur_result = tester.fit_residual_model(window_s=w, stride_s=w, **common)
        print(cur_result)
        tester.save_residual_model(config_path, header_path=header_path,
                                   name=f"residual_model_s{str(w).replace('.', 'p')}_w23_linear_dt5")
    
    for w in [0.5, 1.5, 3.0, 4.0, 5.0]:  # Eval fits independently
        name = f"residual_model_s{str(w).replace('.', 'p')}_w23_linear_dt5"
        tester.load_residual_model(Path(config_path) / f"{name}.pkl")
        print(f"\n===== Evaluating {name} =====")
        tester.compare_residual_model()

    # # ### NONLINEAR FIT ###
    # print("-----Nonlinear (with J1)-----")
    # result = analyzer.fit_nonlinear(fit_source="pend_vel", filter_bw = 25)
    # analyzer.store_params_to_configs(header_path=header_path)
    # #analyzer.simulate_runs()

    # # TEST
    # print("~~~Moving to tester")
    # tester = FurataAnalyzer(cog_path, tau_filter_bw=25)
    # tester.load_params_from_configs(config_path=config_path, withFs=True)
    # tester.add_all_logs(base_path, identifier="generated")
    # tester.parse_runs(doPlot=False, trim = 10)
    # tester.filter_runs()
    # tester.simulate_runs()