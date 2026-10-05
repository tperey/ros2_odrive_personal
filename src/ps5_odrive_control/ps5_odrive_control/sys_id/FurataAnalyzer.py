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

    def load_params_from_configs(self, config_path, withFs = False):

        config_folder = Path(config_path)
        arm1_json = config_folder / "arm1.json"
        if not withFs:
            arm2_json = config_folder / "arm2.json"
        else:
            arm2_json = config_folder / "arm2_fs.json"


        self.arm1_params = json.loads(arm1_json.read_text())
        self.arm2_params = json.loads(arm2_json.read_text())

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

    # def reapply_tau_filter(self):
    #     # Correct the log_dict
    #     for log in self.log_dict.values():
    #         filter_list = [
    #             "tau_act",
    #             "tau_set",
    #             "tau_act_compensated",
    #             "tau_set_compensated"
    #         ]

    #         for filter in filter_list:
    #             log[f"{filter}_filt"] = self._implement_filtfilt(
    #                 log[filter],
    #                 cutoff_hz=self.tau_filter_bw
    #             )

    #     # Correct the RUN dict
    #     if self.runs_parsed:
    #         for name, run_list in self.runs_dict.items():
    #             log = self.log_dict[name]

    #             for run_idx, run in enumerate(run_list):
    #                 start = self.start_dict[name][run_idx]
    #                 end = self.end_dict[name][run_idx]

    #                 for filter in [
    #                     "tau_act",
    #                     "tau_set",
    #                     "tau_act_compensated",
    #                     "tau_set_compensated"
    #                 ]:
    #                     run[f"{filter}_filt"] = log[f"{filter}_filt"][start:end]

    # def _evaluate_true_accel(self, w, sgn_w, tau, J, b, f_s, eps=0.1):
    #     # Moving: Coulomb friction opposes velocity
    #     acc_moving = (tau - b * w - f_s * sgn_w) / J

    #     # Near zero speed: friction opposes net torque, capped at +/- f_s
    #     tau_net  = tau - b * w
    #     tau_fric = np.clip(tau_net, -f_s, f_s)
    #     acc_slow = (tau_net - tau_fric) / J      # exactly 0 when |tau_net| <= f_s

    #     return np.where(np.abs(w) >= eps, acc_moving, acc_slow)

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


    def take_state_deriv(self, state, tau_hat, J2yy_hat, J2xx):
        """ Deriv of state """
        # Unpack
        t1, t1d, t2, t2d = state

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

    def rk4_furata_step(self, state, dt, tau_hat, J2yy_hat, J2xx):
        k1 = self.take_state_deriv(state, tau_hat, J2yy_hat, J2xx)
        k2 = self.take_state_deriv(state + 0.5 * dt * k1, tau_hat, J2yy_hat, J2xx)
        k3 = self.take_state_deriv(state + 0.5 * dt * k2, tau_hat, J2yy_hat, J2xx)
        k4 = self.take_state_deriv(state + dt * k3, tau_hat, J2yy_hat, J2xx)
        return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    def simulate_furata(self, t, state0, tau_hat, J2yy_hat, J2xx):
        """RK4-integrate the diff equ at the exact (possibly non-uniform) sample times in t."""
        state = np.array(state0, dtype=float)
        out = np.empty((len(t), 4))
        out[0,:] = state
        for i in range(len(t) - 1):
            dt = t[i + 1] - t[i]
            state = self.rk4_furata_step(state, dt, tau_hat[i], J2yy_hat, J2xx)
            out[i+1, :] = state
        return out

    def simulate_runs(self):
        """ For each run, copmare actual to forward simulation """
        J2yy_hat = self.arm2_params["J2yy_hat"]
        J2xx = self.arm2_params["J2xx"]
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
                    cur_deriv = self.take_state_deriv(cur_state, tau[i], J2yy_hat, J2xx)
                    t1dd.append(cur_deriv[1])
                    t2dd.append(cur_deriv[3])
                t1dd_sim = np.array(t1dd)
                t2dd_sim = np.array(t2dd)
                t1dd = np.array(run["accel_filt"])
                t2dd = np.array(run["pend_accel_filt"])

                out_sim = self.simulate_furata(t, state0, tau, J2yy_hat, J2xx)
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
    def fit_linear(self, debug = True, tau_source = "act", filter_bw = 100.0):
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
                y2 = -t1dd*C*np.cos(t2) - t2dd*J2zz_hat - b2*t2d - G*np.sin(t2)
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

        def residuals(params):
            J2yy_hat, J2xx = params
            return A @ np.array([J2yy_hat, J2xx]) - y

        def jac(params):
            return A  # linear model -> constant Jacobian

        x0 = [J2zz_hat, J2zz_hat]
        bounds = ([m2*(l2**2), 0], [np.inf, np.inf])
        result = least_squares(residuals, x0, jac=jac, bounds=bounds)

        # Save result
        self.arm2_params["J2yy_hat"] = result.x[0]
        self.arm2_params["J2xx"] = result.x[1]
        print(result)
        return result

    # def optimize_linear_fit(self, debug = True, inclCoulomb = True, min_bw = 1.0, max_bw = 101.0, step_size = 5.0, tau_source = "act"):
    #     """ Run linear_fit repeatedly with tons of possible filter params and show me the best one """
    #     debug_count = 0
    #     cur_min_cost = np.inf
    #     final_bw = self.tau_filter_bw
    #     print("Starting optimum linear fit...")
    #     for bw in np.arange(min_bw, max_bw, step_size):
    #         cur_result = self.fit_linear(debug=False, inclCoulomb=inclCoulomb, tau_source=tau_source, filter_bw=bw)

    #         if cur_result.cost < cur_min_cost:
    #             cur_min_cost = cur_result.cost
    #             final_bw = bw

    #         if debug:
    #             debug_count += 1
    #             if debug_count % 10 == 0:
    #                 print(f"On iteration {debug_count}")

    #     # Rerun with the final best
    #     final_result = self.fit_linear(debug=False, inclCoulomb=inclCoulomb, tau_source=tau_source, filter_bw=final_bw)

    #     # Show final result
    #     print("---OPTIMUM BASIC LINEAR FIT RESULT---")
    #     print(final_result)
    #     print(f"Best filter bw = {final_bw}")
    #     self.test_linear_fit()

    # def test_linear_fit(self, useTrue = True):
    #     """
    #     For each run, compare the measured (filtered) acceleration against
    #     the model's predicted acceleration, using the fitted J, b, f_s.

    #         theta_dd_pred = (tau_compensated_filt - b*theta_d - f_s*sign(theta_d)) / J

    #     Requires fit_linear() to have been run first (self.motor_params must exist).
    #     """
    #     if not hasattr(self, "motor_params") or self.motor_params is None:
    #         raise RuntimeError("Run fit_linear() first to populate self.motor_params.")

    #     J = self.motor_params["J"]
    #     b = self.motor_params["b"]
    #     f_s = self.motor_params["f_s"]

    #     for run_list in self.runs_dict.values():
    #         for run in run_list:
    #             if "accel_filt" not in run:
    #                 # This run wasn't processed by fit_linear (e.g. added after fitting)
    #                 raise RuntimeError("Run fit_linear() first to populate acceleration.")

    #             t = np.array(run["time"])
    #             tau = np.array(run["used_tau_filt"])
    #             theta_d = run["vel_filt"]
    #             sgn_theta_d = run["sgn_vel_filt"]
    #             theta_dd_measured = run["accel_filt"]  # saved during fit_linear
    #             if useTrue:
    #                 theta_dd_predicted = self._evaluate_true_accel(theta_d, sgn_theta_d, tau, J, b, f_s)
    #             else:
    #                 theta_dd_predicted = (tau - b * theta_d - f_s * sgn_theta_d) / J
    #             residual = theta_dd_measured - theta_dd_predicted

    #             fig, axs = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
    #             axs[0].plot(t, theta_dd_measured, label="Measured accel (filtered)", color="black")
    #             axs[0].plot(t, theta_dd_predicted, label="Model-predicted accel", color="red", linestyle="--")
    #             axs[0].set_ylabel("Accel (rad/s^2)")
    #             axs[0].set_title("Measured vs Predicted Acceleration")
    #             axs[0].grid(True, alpha=0.3)
    #             axs[0].legend()

    #             axs[1].plot(t, tau, label="Tau", color="blue")
    #             axs[1].set_ylabel("Torque (N-m)")
    #             axs[1].set_xlabel("Time (ms)")
    #             axs[1].grid(True, alpha=0.3)
    #             axs[1].legend()

    #             axs[2].plot(t, residual, label="Residual (measured - predicted)", color="purple")
    #             axs[2].set_ylabel("Residual (rad/s^2)")
    #             axs[2].set_xlabel("Time (ms)")
    #             axs[2].grid(True, alpha=0.3)
    #             axs[2].legend()

    #             fig.tight_layout()
    #             plt.show()

    # def best_test_linear_fit(self, eps_range = [0.01, 0.3, 0.01]):
    #     """
    #     For each run, compare the measured (filtered) acceleration against
    #     the model's predicted acceleration, using the fitted J, b, f_s.

    #         theta_dd_pred = (tau_compensated_filt - b*theta_d - f_s*sign(theta_d)) / J

    #     Requires fit_linear() to have been run first (self.motor_params must exist).
    #     """
    #     if not hasattr(self, "motor_params") or self.motor_params is None:
    #         raise RuntimeError("Run fit_linear() first to populate self.motor_params.")

    #     J = self.motor_params["J"]
    #     b = self.motor_params["b"]
    #     f_s = self.motor_params["f_s"]

    #     for run_list in self.runs_dict.values():
    #         for run in run_list:
    #             if "accel_filt" not in run:
    #                 # This run wasn't processed by fit_linear (e.g. added after fitting)
    #                 raise RuntimeError("Run fit_linear() first to populate acceleration.")

    #             t = np.array(run["time"])
    #             tau = np.array(run["used_tau_filt"])
    #             theta_d = run["vel_filt"]
    #             sgn_theta_d = run["sgn_vel_filt"]
    #             theta_dd_measured = run["accel_filt"]  # saved during fit_linear

    #             # Find best eps
    #             best_eps = 0.0
    #             min_cost = np.inf
    #             for eps in np.arange(eps_range[0], eps_range[1], eps_range[2]):
    #                 theta_dd_predicted = self._evaluate_true_accel(theta_d, sgn_theta_d, tau, J, b, f_s, eps = eps)
    #                 residual = theta_dd_measured - theta_dd_predicted

    #                 cur_cost = np.sum(residual**2)
    #                 if cur_cost < min_cost:
    #                     best_eps = eps
                
    #             # Final re-eval
    #             print(f"Best eps = {best_eps}")
    #             theta_dd_predicted = self._evaluate_true_accel(theta_d, sgn_theta_d, tau, J, b, f_s, eps = eps)
    #             residual = theta_dd_measured - theta_dd_predicted

    #             fig, axs = plt.subplots(3, 1, figsize=(10, 6), sharex=True)
    #             axs[0].plot(t, theta_dd_measured, label="Measured accel (filtered)", color="black")
    #             axs[0].plot(t, theta_dd_predicted, label="Model-predicted accel", color="red", linestyle="--")
    #             axs[0].set_ylabel("Accel (rad/s^2)")
    #             axs[0].set_title("Measured vs Predicted Acceleration")
    #             axs[0].grid(True, alpha=0.3)
    #             axs[0].legend()

    #             axs[1].plot(t, tau, label="Tau", color="blue")
    #             axs[1].set_ylabel("Torque (N-m)")
    #             axs[1].set_xlabel("Time (ms)")
    #             axs[1].grid(True, alpha=0.3)
    #             axs[1].legend()

    #             axs[2].plot(t, residual, label="Residual (measured - predicted)", color="purple")
    #             axs[2].set_ylabel("Residual (rad/s^2)")
    #             axs[2].set_xlabel("Time (ms)")
    #             axs[2].grid(True, alpha=0.3)
    #             axs[2].legend()

    #             fig.tight_layout()
    #             plt.show()

    # # ---------------------------------------------------------------------------
    # # Method 2: forward-simulation (RK4) with coulombic friction + nonlinear least squares
    # # ---------------------------------------------------------------------------
    
    # """ FITTING """
    # def smooth_motor_lhs(self, state, tau, J, b, f_s, k = 100):
    #     """ Deriv of state """
    #     theta, omega = state
    #     dtheta = omega
    #     domega = (tau - b * omega - f_s * np.tanh(k*omega)) / J
    #     return np.array([dtheta, domega])

    # def smooth_rk4_motor_step(self, state, dt, tau, J, b, f_s):
    #     k1 = self.smooth_motor_lhs(state, tau, J, b, f_s)
    #     k2 = self.smooth_motor_lhs(state + 0.5 * dt * k1, tau, J, b, f_s)
    #     k3 = self.smooth_motor_lhs(state + 0.5 * dt * k2, tau, J, b, f_s)
    #     k4 = self.smooth_motor_lhs(state + dt * k3, tau, J, b, f_s)
    #     return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    # def smooth_simulate_motor(self, t, theta0, omega0, tau, J, b, f_s):
    #     """RK4-integrate the diff equa at the exact (possibly non-uniform) sample times in t."""
    #     state = np.array([theta0, omega0], dtype=float)
    #     out = np.empty((len(t), 2))
    #     out[0,:] = state
    #     for i in range(len(t) - 1):
    #         dt = t[i + 1] - t[i]
    #         state = self.smooth_rk4_motor_step(state, dt, tau[i], J, b, f_s)
    #         out[i+1, :] = state
    #     return out

    # def fit_nonlinear(self, J0, b0, fs0, tau_source = "act", fit_source = "vel", filter_bw = 0.0, inclCoulomb = True):
    #     """
    #     J0: initial inertia guess
    #     b0: initial damping guess
    #     fs0: initial coulombic friction guess
    #     """
    #     if not self.runs_parsed:
    #         self.parse_runs()

    #     # First, do proper filtering
    #     for run_list in self.runs_dict.values():
    #         for run in run_list:
    #             t = np.array(run["time"])
    #             if tau_source == "act":
    #                 tau_raw = np.array(run["tau_act_compensated"])
    #             elif tau_source == "set":
    #                 tau_raw = np.array(run["tau_set_compensated"])
    #             else:
    #                 raise ValueError("This is not a valid tau_source")
    #             theta = np.array(run["pos"])
    #             theta_d_raw = np.array(run["vel"])
    #             sgn_theta_d_raw = np.sign(theta_d_raw)

    #             dt = np.mean(np.diff(t))
    #             if not np.allclose(np.diff(t), dt, rtol=1.0):
    #                 raise ValueError("Method 1 requires (approximately) uniform sampling per run; "
    #                                 "resample/interpolate onto a uniform grid first.")
    #             theta_dd_raw = np.gradient(theta_d_raw, 0.001)
            
    #             # Filter
    #             self.linear_filter_bw = filter_bw
    #             if filter_bw != 0.0:
    #                 theta_d = self._implement_filtfilt(theta_d_raw, cutoff_hz=filter_bw)
    #                 sgn_theta_d = self._implement_filtfilt(sgn_theta_d_raw, cutoff_hz=filter_bw)
    #                 theta_dd = self._implement_filtfilt(theta_dd_raw, cutoff_hz=filter_bw)
    #                 tau = self._implement_filtfilt(tau_raw, cutoff_hz=filter_bw)
    #             else:
    #                 theta_d = theta_d
    #                 sgn_theta_d = sgn_theta_d_raw
    #                 theta_dd = theta_dd_raw
    #                 tau = tau_raw
    #             run["vel_filt"] = theta_d  # Save for eval
    #             run["sgn_vel_filt"] = sgn_theta_d
    #             run["accel_filt"] = theta_dd
    #             run["used_tau_filt"] = tau

    #     def residuals(params):
    #         J, b, f_s = params
    #         res = []
    #         for run_list in self.runs_dict.values():
    #             for run in run_list:
    #                 # Parse
    #                 t = np.array(run["time"])/1000.0
    #                 tau = np.array(run["used_tau_filt"])
    #                 theta = np.array(run["pos"])
    #                 omega = np.array(run["vel_filt"])
    
    #                 # Forward simulation
    #                 out_sim = self.smooth_simulate_motor(t, theta[0], omega[0], tau, J, b, f_s)
    #                 theta_sim = out_sim[:, 0]
    #                 omega_sim = out_sim[:, 1]
    
    #                 # Compare to actual
    #                 if fit_source == "pos":
    #                     cur_res = (theta_sim - theta)
    #                 elif fit_source == "vel":
    #                     cur_res = (omega_sim - omega)
    #                 else:
    #                     raise ValueError("Invalid fit source")
    #                 res.append(cur_res)
    #         return np.concatenate(res)

    #     print("Starting least_squares")
    #     result = least_squares(
    #         residuals, x0=[J0, b0, fs0],
    #         bounds=([1e-12, 0, 0], [np.inf, np.inf, np.inf]),
    #         method="trf",
    #         loss="soft_l1",   # mild robustness to outlier samples/glitches; use 'linear' for plain LS
    #         x_scale=[max(J0, 1e-9), max(b0, 1e-9), max(fs0, 1e-9)],  # helps scipy when J and b have very different magnitudes
    #         verbose=2,
    #     )

    #     # Save result
    #     self.motor_params = {
    #         "J": result.x[0],
    #         "b": result.x[1],
    #         "f_s": result.x[2] if inclCoulomb else 0.0
    #     }
    #     return result

# ---------------------------------------------------------------------------
# Fit dat data
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    base_path = "/Users/trevorperey/Desktop/PersonalProjects/ros2_odrive_personal/src/ps5_odrive_control/ps5_odrive_control/logs/"
    cog_path = "/Users/trevorperey/Desktop/PersonalProjects/ros2_odrive_personal/config/m8325s_furata/cogging_map.json"
    config_path = "/Users/trevorperey/Desktop/PersonalProjects/ros2_odrive_personal/config/m8325s_furata"
    one_example = base_path + "sinetau_nocog_motoronly_001/logs_20260823_211700.pkl"

    analyzer = FurataAnalyzer(cog_path, tau_filter_bw=25)
    analyzer.load_params_from_configs(config_path=config_path, withFs=True)
    analyzer.add_all_logs(base_path, identifier="generated")
    analyzer.parse_runs(doPlot=False, trim = 10)
    #analyzer.plot_raw_all()

    # ### LINEAR FIT ###
    print("-----Linear-----")
    result = analyzer.fit_linear(debug=False, filter_bw = 25)
    # analyzer.arm2_params["J2yy_hat"] = 0.0
    # analyzer.arm2_params["J2xx"] = 0.0
    # # analyzer.arm2_params["J2zz_hat"] = 0.0
    # analyzer.arm2_params["b2"] = 0.0
    # analyzer.arm2_params["L2"] = 0.0
    # analyzer.arm2_params["l2"] = 0.0
    analyzer.simulate_runs()