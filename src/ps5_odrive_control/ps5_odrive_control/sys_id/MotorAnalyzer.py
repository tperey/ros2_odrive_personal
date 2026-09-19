"""
Fit J (inertia) and b (damping) for a motor with a load:

    J*theta'' + b*theta' + friction_term = Kt*current + tau_cogging = tau_decogged

First, removes the effects of cogging with the pre-determined map.
Aka computes tau_decogged
Note this map was the current to hold position, i.e. OVERCOME cogging.
That means it is the NEGATIVE of cogging torque.
So it is SUBTRACTED to get the overall RHS above.

Includes options for actual torque filtering as well.

Then, Two independent methods are implemented so you
can cross-check them against each other:

  Method 1 (derivative-based, LINEAR):
      Smooth+differentiate theta(t) with a Savitzky-Golay filter to get
      theta, theta', theta''. The ODE is linear in [J, b, friction] so
      this collapses to a linear least-squares problem:
      minimize|| J*theta'' + b*theta' + friction_term - tau_decogged ||^2.
      Fast, no simulation needed, but sensitive to differentiation noise.

      There are variants with and without the friction term.
      The friction term used here is f_s*sign(theta')

  Method 2 (forward-simulation, NONLINEAR output-error):
      RK4-integrate the ODE from each run's initial condition (theta0 from
      data, omega0 = 0 since you drop it from rest) using trial (J, b), and
      minimize the mismatch between simulated theta(t) and measured theta(t).
      No differentiation of noisy data required. Slower (needs simulation
      inside the optimizer loop) but avoids double-differentiation error.

Both methods pool ALL N runs into one residual vector so a single (J, b, friction)
must explain every run simultaneously -- this is what keeps 2 parameters
from overfitting to noise in any one run.

Usage: fill in `runs` near the bottom with your real (t, theta) arrays per
run, or run as-is to see it validated on synthetic data with known J, b.

For reference:
Watch out for weirdness with odrive sign convention.
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

class MotorAnalyzer:

    def __init__(self, cog_file, tau_filter_bw = 100.0, accel_filter_bw = 100.0):
        # Infrastructure to combine several runs
        self.log_dict = {} # to be dict of dicts

        self.tau_filter_bw = tau_filter_bw
        self.accel_filter_bw = accel_filter_bw

        with open(cog_file, "r") as f:
            self.cog_map = np.asanyarray(json.load(f))

        # Convert everything to numpy arrays, and parse out all runs (periods between motor stops)
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

    def _implement_filtfilt(self, signal, order = 2, cutoff_hz = 100, fs = 1000):
        nyquist = fs / 2.0
        normal_cutoff = cutoff_hz / nyquist
        b, a = butter(order, normal_cutoff, btype='low')
        return filtfilt(b, a, signal)

    def _postprocess(self, log, name, filter_bw = 100):
        log["time"] -= log["time"][0]
        if "cogd" in name:
            print("---Shifting setpoint bwd by 1! Bc setpoint sent AFTER polling telemetry. So actual setpoint at N is logged at N+1")
            log["tau_set"] = np.roll(np.array(log["tau_set"]), -1)

        # Decog
        encoder_indices = ((np.round(np.array(log["pos"])*COUNTS_PER_REV) % COUNTS_PER_REV)).astype(int)
        log["tau_act_decogged"] = log["tau_act"] - self.cog_map[encoder_indices] 
        log["tau_set_decogged"] = log["tau_set"] - self.cog_map[encoder_indices]

        # Filter
        filter_list = ["tau_act", "tau_set", "tau_act_decogged", "tau_set_decogged"]
        for filter in filter_list:
            log[f"{filter}_filt"] = self._implement_filtfilt(log[filter], cutoff_hz=self.tau_filter_bw)

    def change_tau_filter(self, filter_bw = 100):
        # Overwrite the prior tau filter with a new one
        for log in self.log_dict.values():
            # Redo filter
            filter_list = ["tau_act", "tau_set", "tau_act_decogged", "tau_set_decogged"]
            for filter in filter_list:
                log[f"{filter}_filt"] = self._implement_filtfilt(log[filter], cutoff_hz=self.tau_filter_bw)

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

            # Basics
            fig, axes = plt.subplots(3, 1, figsize=(12, 7.5), sharex=True)

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

            axes[2].plot(log["time"], log["iq_set"], linewidth=1, marker = ".", color = "blue", label = "Iq setpoint")
            axes[2].plot(log["time"], log["iq_act"], linewidth=1, marker = ".", color = "green", label = "Iq actual")
            axes[2].set_ylabel("Current (A)")
            axes[2].set_title(f"{name} - Current/Torque Raw")
            axes[2].legend()
            axes[2].grid(True, alpha=0.3)

            # Torque, with modifications

            fig, axes = plt.subplots(3, 1, figsize=(12, 7.5), sharex=True, sharey=True)

            axes[0].plot(log["time"], log["tau_set"], linewidth=1, marker = ".", color = "orange", label = "Torque setpoint")
            axes[0].plot(log["time"], log["tau_set_filt"], linewidth=1, color = "red", label = "Tau set filtered")
            axes[0].plot(log["time"], log["tau_act"], linewidth=1, marker = ".", color = "gray", label = "Torque actual")
            axes[0].plot(log["time"], log["tau_act_filt"], linewidth=1, color = "black", label = "Tau actual filtered")
            axes[0].plot(log["time"], cmd, linewidth=1, color = "purple", label = "Command")
            axes[0].set_ylabel("Torque (N-m)")
            axes[0].set_xlabel("Time (ms)")
            axes[0].legend()
            axes[0].grid(True, alpha=0.3)

            axes[1].plot(log["time"], log["tau_set_decogged"], linewidth=1, marker = ".", color = "orange", label = "Torque setpoint decogged")
            axes[1].plot(log["time"], log["tau_set_decogged_filt"], linewidth=1, color = "red", label = "Tau set decogged Filtered")
            axes[1].plot(log["time"], log["tau_act_decogged"], linewidth=1, marker = ".", color = "gray", label = "Torque actual decogged")
            axes[1].plot(log["time"], log["tau_act_decogged_filt"], linewidth=1, color = "black", label = "Tau actual decogged Filtered")
            axes[1].plot(log["time"], cmd, linewidth=1, marker = ".", color = "purple", label = "Command")
            axes[1].set_ylabel("Torque (N-m)")
            axes[1].set_xlabel("Time (ms)")
            axes[1].legend()
            axes[1].grid(True, alpha=0.3)

            axes[2].plot(log["time"], (log["tau_set"] - cmd), label = "Delta (set vs cmd)", color = "blue")
            axes[2].plot(log["time"], (log["tau_set_decogged"] - cmd), label = "Delta (set vs cmd) AFTER decog", color = "green")
            axes[2].set_ylabel("Torque (N-m)")
            axes[2].set_xlabel("Time (ms)")
            axes[2].legend()
            axes[2].grid(True, alpha=0.3)
            
            fig.tight_layout()

            plt.show()

    def parse_runs(self, time_key="time", command_key="cmd", dt_thresh=3, nt_thresh = 10, zero_tol=1e-3, doPlot = True):
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

            # Indices where a gap occurs; +1 because diff shifts indices by one
            split_points = np.where(dt > dt_thresh)[0] + 1

            # Build segment boundaries: [0, split1, split2, ..., len(time)]
            boundaries = np.concatenate(([0], split_points, [len(time)]))

            runs = []
            starts = []
            ends = []
            counter = 0
            for start, end in zip(boundaries[:-1], boundaries[1:]):
                end -= 1 # Ends are 1 BEFORE the pause
                segment_cmd = np.asarray(log[command_key])[start:end]

                # Skip startup and pause segments: command is ~zero or short for the whole segment
                if (np.mean(np.abs(segment_cmd)) < zero_tol) or (len(segment_cmd) < nt_thresh):
                    continue

                run_dict = {k: np.asarray(v)[start:end] for k, v in log.items()}
                starts.append(start)
                ends.append(end)
                runs.append(run_dict)

                # optional by run plot
                if doPlot:
                    self._plot_raw(f"{name}_{counter}", run_dict, False)
                    counter += 1

            # Store the results
            self.start_dict[name] = starts
            self.end_dict[name] = ends
            self.runs_dict[name] = runs

    # def add_startpoints(self, start_list):
    #     for entry in start_list:
    #         idx = np.argmin(np.abs(entry - self.time))
    #         self.start_list.append(idx)
    #     self.find_endpoints()

    # def find_endpoints(self, thresh = 0.0001, hold = 100, jump = 1000):
    #     for cur_start in self.start_list:

    #         start = cur_start + jump
            
    #         cur_velocity = (self.data["pend_vel"])[start:]
    #         settled = np.abs(cur_velocity) < thresh

    #         # Find settling point
    #         for i in range(len(settled) - hold):
    #             if np.all(settled[i:i+hold]):
    #                 self.end_list.append(start+i)
    #                 break

    # # ---------------------------------------------------------------------------
    # # RK4 integrator (for non-linear simulation method)
    # # ---------------------------------------------------------------------------
    # def pendulum_rhs(self, state, J, b, mgl):
    #     """ Deriv of state """
    #     theta, omega = state
    #     dtheta = omega
    #     domega = -(b * omega + mgl * np.sin(theta)) / J
    #     return np.array([dtheta, domega])


    # def rk4_step(self, state, dt, J, b, mgl):
    #     k1 = self.pendulum_rhs(state, J, b, mgl)
    #     k2 = self.pendulum_rhs(state + 0.5 * dt * k1, J, b, mgl)
    #     k3 = self.pendulum_rhs(state + 0.5 * dt * k2, J, b, mgl)
    #     k4 = self.pendulum_rhs(state + dt * k3, J, b, mgl)
    #     return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


    # def simulate_theta(self, t, theta0, omega0, J, b, mgl):
    #     """RK4-integrate theta(t) at the exact (possibly non-uniform) sample times in t."""
    #     state = np.array([theta0, omega0], dtype=float)
    #     out = np.empty(len(t))
    #     out[0] = state[0]
    #     for i in range(len(t) - 1):
    #         dt = t[i + 1] - t[i]
    #         state = self.rk4_step(state, dt, J, b, mgl)
    #         out[i + 1] = state[0]
    #     return out


    # # ---------------------------------------------------------------------------
    # # Fitting helpers
    # # ---------------------------------------------------------------------------

    # def _generate_runs(self):
    #     self.runs = []  # Clear to start
    #     for start, end in zip(self.start_list, self.end_list):
    #         cur_t = self.time[start:end]
    #         cur_pos = (self.data["pend_pos"])[start:end]
    #         self.runs.append((cur_t, cur_pos))

    # def test_fit(self, J, b, mgl):
    #     """ For each run, copmare actual to forward simulation """
    #     self._generate_runs()
    #     for t, theta in self.runs:

    #         # Forward simulation
    #         theta_sim = self.simulate_theta(t, theta[0], 0.0, J, b, mgl)

    #         # Compare to actual
    #         residuals = (theta_sim - theta)**2

    #         # Plot
    #         fig, axs = plt.subplots(2,1, figsize=(10,6), sharex = True)
    #         axs[0].plot(t, theta, label = "Measured")
    #         axs[0].plot(t, theta_sim, label = "Predicted")
    #         axs[0].set_ylabel("Position (rad)")
    #         axs[0].set_title("Linear Method Evaluation")
    #         axs[0].legend()
    #         axs[0].grid(True, alpha=0.3)
    
    #         # Velocity
    #         axs[1].plot(t, residuals, linewidth=1, marker = ".", color = "green", label = "Square Error")
    #         axs[1].set_ylabel("Residual (rad^2)")
    #         axs[1].set_xlabel("Time (s)")
    #         axs[1].legend()
    #         axs[1].grid(True, alpha=0.3)
    
    #         fig.tight_layout()
    
    #         plt.show()

    # # ---------------------------------------------------------------------------
    # # Method 1: derivative-based, linear least squares (with bounds via least_squares)
    # # ---------------------------------------------------------------------------
    # def fit_linear(self, mgl, debug = True):
    #     """
    #     Uses numerical diff and filtering to get omega and alpha.
    #     Returns: OptimizeResult from least_squares (result.x = [J, b])
    #     """

    #     A_blocks, y_blocks = [], []

    #     self._generate_runs()
    #     for t, theta in self.runs:
    #         dt = np.mean(np.diff(t))
    #         if not np.allclose(np.diff(t), dt, rtol=1e-3):
    #             raise ValueError("Method 1 requires (approximately) uniform sampling per run; "
    #                             "resample/interpolate onto a uniform grid first.")

    #         theta_d_raw = np.gradient(theta, dt)
    #         theta_d = self._implement_filtfilt(theta_d_raw, cutoff_hz= 30.0)

    #         theta_dd_raw = np.gradient(theta_d, dt)
    #         theta_dd = self._implement_filtfilt(theta_dd_raw, cutoff_hz=30.0)

    #         A_blocks.append(np.column_stack([theta_dd, theta_d]))   # coefficients of [J, b]
    #         y_blocks.append(-mgl * np.sin(theta))                   # known-mgl term moved to RHS

    #         if debug:
    #             fig, axs = plt.subplots(1, 1, figsize = (10,6), sharex = True)
    #             axs.plot(t, theta, label = "Theta")
    #             axs.plot(t, theta_d, label = "Omega")
    #             axs.plot(t, theta_dd, label = "Alpha")
    #             axs.legend()
                
    #             plt.show()

    #     A = np.vstack(A_blocks)
    #     y = np.concatenate(y_blocks)

    #     def residuals(params):
    #         J, b = params
    #         return A @ np.array([J, b]) - y

    #     def jac(params):
    #         return A  # linear model -> constant Jacobian

    #     x0 = [1e-4, 1e-4]  # rough starting guess; replace with your CAD estimate if you have one
    #     result = least_squares(residuals, x0, jac=jac, bounds=([1e-12, 0], [np.inf, np.inf]))
    #     return result

    # # ---------------------------------------------------------------------------
    # # Method 2: forward-simulation (RK4) + nonlinear least squares
    # # ---------------------------------------------------------------------------

    # def fit_nonlinear(self, mgl, J0, b0, omega0=0.0):
    #     """
    #     mgl: known gravity term
    #     J0: initial inertia guess
    #     b0: initial damping guess
    #     omega0: initial angular velocity for every run (0.0 for lift-and-drop-from-rest).
    #             Pass a list instead if release velocity varies run to run.
    #     """
    #     self._generate_runs()
    #     N = len(self.runs)
    #     omega0_list = omega0 if hasattr(omega0, "__len__") else [omega0] * N

    #     def residuals(params):
    #         J, b = params
    #         res = []
    #         for (t, theta), w0 in zip(self.runs, omega0_list):
    #             sim = self.simulate_theta(t, theta[0], w0, J, b, mgl)
    #             res.append(sim - theta)
    #         return np.concatenate(res)

    #     result = least_squares(
    #         residuals, x0=[J0, b0],
    #         bounds=([1e-12, 0], [np.inf, np.inf]),
    #         method="trf",
    #         loss="soft_l1",   # mild robustness to outlier samples/glitches; use 'linear' for plain LS
    #         x_scale=[max(J0, 1e-9), max(b0, 1e-9)],  # helps scipy when J and b have very different magnitudes
    #         verbose=2,
    #     )
    #     return result

    # # ---------------------------------------------------------------------------
    # # Method 3: forward-simulation (RK4) with coulombic friction + nonlinear least squares
    # # ---------------------------------------------------------------------------
    # def pendulum_fs_rhs(self, state, J, b, mgl, f_s, k):
    #     """ Deriv of state """
    #     theta, omega = state
    #     dtheta = omega
    #     domega = -(b * omega + mgl * np.sin(theta) + f_s * np.tanh(k*omega)) / J
    #     return np.array([dtheta, domega])


    # def rk4_fs_step(self, state, dt, J, b, mgl, f_s, k):
    #     k1 = self.pendulum_fs_rhs(state, J, b, mgl, f_s, k)
    #     k2 = self.pendulum_fs_rhs(state + 0.5 * dt * k1, J, b, mgl, f_s, k)
    #     k3 = self.pendulum_fs_rhs(state + 0.5 * dt * k2, J, b, mgl, f_s, k)
    #     k4 = self.pendulum_fs_rhs(state + dt * k3, J, b, mgl, f_s, k)
    #     return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


    # def simulate_fs_theta(self, t, theta0, omega0, J, b, mgl, f_s, k):
    #     """RK4-integrate theta(t) at the exact (possibly non-uniform) sample times in t."""
    #     state = np.array([theta0, omega0], dtype=float)
    #     out = np.empty(len(t))
    #     out[0] = state[0]
    #     for i in range(len(t) - 1):
    #         dt = t[i + 1] - t[i]
    #         state = self.rk4_fs_step(state, dt, J, b, mgl, f_s, k)
    #         out[i + 1] = state[0]
    #     return out

    # def fit_fs_nonlinear(self, mgl, J0, b0, fs0, omega0=0.0, k=10):
    #     """
    #     mgl: known gravity term
    #     J0: initial inertia guess
    #     b0: initial damping guess
    #     fs0: initial coulombic friction guess
    #     omega0: initial angular velocity for every run (0.0 for lift-and-drop-from-rest).
    #             Pass a list instead if release velocity varies run to run.
    #     k: factor to use for tanh. NOT learned. Functionally a hyperparameter
    #     """
    #     self._generate_runs()
    #     N = len(self.runs)
    #     omega0_list = omega0 if hasattr(omega0, "__len__") else [omega0] * N

    #     def residuals(params):
    #         J, b, fs = params
    #         res = []
    #         for (t, theta), w0 in zip(self.runs, omega0_list):
    #             sim = self.simulate_fs_theta(t, theta[0], w0, J, b, mgl, fs, k)
    #             res.append(sim - theta)
    #         return np.concatenate(res)

    #     result = least_squares(
    #         residuals, x0=[J0, b0, fs0],
    #         bounds=([1e-12, 0, 0], [np.inf, np.inf, np.inf]),
    #         method="trf",
    #         loss="soft_l1",   # mild robustness to outlier samples/glitches; use 'linear' for plain LS
    #         x_scale=[max(J0, 1e-9), max(b0, 1e-9), max(fs0, 1e-9)],  # helps scipy when J and b have very different magnitudes
    #         verbose=2,
    #     )
    #     return result

    # def test_fs_fit(self, J, b, mgl, f_s, k = 10):
    #     """ For each run, copmare actual to forward simulation """
    #     self._generate_runs()
    #     for t, theta in self.runs:

    #         # Forward simulation
    #         theta_sim = self.simulate_fs_theta(t, theta[0], 0.0, J, b, mgl, f_s, k)

    #         # Compare to actual
    #         residuals = (theta_sim - theta)**2

    #         # Plot
    #         fig, axs = plt.subplots(2,1, figsize=(10,6), sharex = True)
    #         axs[0].plot(t, theta, label = "Measured")
    #         axs[0].plot(t, theta_sim, label = "Predicted")
    #         axs[0].set_ylabel("Position (rad)")
    #         axs[0].set_title("Linear Method Evaluation")
    #         axs[0].legend()
    #         axs[0].grid(True, alpha=0.3)
    
    #         # Velocity
    #         axs[1].plot(t, residuals, linewidth=1, marker = ".", color = "green", label = "Square Error")
    #         axs[1].set_ylabel("Residual (rad^2)")
    #         axs[1].set_xlabel("Time (s)")
    #         axs[1].legend()
    #         axs[1].grid(True, alpha=0.3)
    
    #         fig.tight_layout()
    
    #         plt.show()


# ---------------------------------------------------------------------------
# Fit dat data
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    base_path = "/Users/trevorperey/Desktop/PersonalProjects/ros2_odrive_personal/src/ps5_odrive_control/ps5_odrive_control/logs/"
    cog_path = "/Users/trevorperey/Desktop/PersonalProjects/ros2_odrive_personal/config/m8325s_furata/cogging_map.json"
    one_example = base_path + "sinetau_nocog_motoronly_001/logs_20260823_211700.pkl"

    analyzer = MotorAnalyzer(cog_path, tau_filter_bw=50)
    analyzer.add_all_logs(base_path, identifier="cogd_motoronly")
    analyzer.parse_runs()
    analyzer.plot_raw_all()


    # analyzer.add_startpoints([10.364, 25.072, 42.481, 55.682, 71.790, 88.580, 102.856, 117.666, 133.164, 149.647, 168.993, 186.630, 207.018, 233.207, 253.556])
    # #analyzer.add_startpoints([10.364])
    # #analyzer.plot_raw()

    # m = 38.03/1000.0  # [kg]
    # g = 9.81  # [m/s^2]
    # l = (77.171 - 30.0)/1000.0  # [m]

    # # Linear fit
    # linear_fit = analyzer.fit_linear(mgl=(m*g*l), debug=False)

    # J_fit, b_fit = linear_fit.x
    # print(f"Result of Linear Method: J = {J_fit}, b = {b_fit}")

    # #analyzer.test_fit(J_fit, b_fit, (m*g*l))

    # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

    # """ Non-linear fit """
    # nonlinear_fit = analyzer.fit_nonlinear((m*g*l), 1e-4, 1e-4, 0.0)

    # J_n_fit, b_n_fit = nonlinear_fit.x
    # print(f"Result of NON-Linear Method: J = {J_n_fit}, b = {b_n_fit}")

    # analyzer.test_fit(J_n_fit, b_n_fit, (m*g*l))

    # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

    # """ Non-linear fit, with friction """
    # # k_to_use = 10
    # # fs_nonlinear_fit = analyzer.fit_fs_nonlinear((m*g*l), 1e-4, 1e-4, 0.001, 0.0, k_to_use)

    # # Jnfs_fit, bnfs_fit, fs_fit = fs_nonlinear_fit.x
    # # print(f"Result of Nonlinear method WITH FRICTION: J = {Jnfs_fit}, b = {bnfs_fit}, fs = {fs_fit}")

    # # analyzer.test_fs_fit(Jnfs_fit, bnfs_fit, (m*g*l), fs_fit, k_to_use)