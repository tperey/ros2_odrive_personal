"""
Fit J (inertia) and b (damping) for a pendulum on an encoder:

    J*theta'' + b*theta' + mgl*sin(theta) = 0

mgl is known (from CAD/print). Two independent methods are implemented so you
can cross-check them against each other:

  Method 1 (derivative-based, LINEAR):
      Smooth+differentiate theta(t) with a Savitzky-Golay filter to get
      theta, theta', theta''. The ODE is linear in [J, b] (mgl is a known
      coefficient, not a fitted one), so this collapses to a linear
      least-squares problem: minimize || J*theta'' + b*theta' + mgl*sin(theta) ||^2.
      Fast, no simulation needed, but sensitive to differentiation noise.

  Method 2 (forward-simulation, NONLINEAR output-error):
      RK4-integrate the ODE from each run's initial condition (theta0 from
      data, omega0 = 0 since you drop it from rest) using trial (J, b), and
      minimize the mismatch between simulated theta(t) and measured theta(t).
      No differentiation of noisy data required. Slower (needs simulation
      inside the optimizer loop) but avoids double-differentiation error.

Both methods pool ALL N runs into one residual vector so a single (J, b)
must explain every run simultaneously -- this is what keeps 2 parameters
from overfitting to noise in any one run.

Usage: fill in `runs` near the bottom with your real (t, theta) arrays per
run, or run as-is to see it validated on synthetic data with known J, b.
"""

import pickle
import numpy as np
from scipy.optimize import least_squares
from scipy.signal import savgol_filter, butter, filtfilt
from pathlib import Path
import matplotlib.pyplot as plt

class PendulumAnalyzer:

    def __init__(self, log_path):

        # Handle paths
        self.log_path = Path(log_path)
        with open(self.log_path, "rb") as f:
            log = pickle.load(f)

        # Convert everything to numpy arrays
        self.dt = 0.001
        self.start_list = []
        self.end_list = []
        self.runs = []
        self.data = {k: np.asarray(v) for k, v in log.items()}

        # Post process
        self._postprocess()

    def _postprocess(self):
        self.time = np.arange(0, len(self.data["pend_pos"])/1000.0, self.dt)

    def plot_raw(self):
        """time vs tau_act and time vs pos, stacked in 2 rows, 1 col, shared x."""
        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

        # Position
        axes[0].plot(self.time, self.data["pend_pos"], linewidth=1, marker = ".", color = "blue", label = "Position")

        if self.start_list:
            axes[0].scatter(self.time[self.start_list], (self.data["pend_pos"])[self.start_list], color = "red", marker = "x", label = "Start points", s = 100)

        if self.end_list:
            axes[0].scatter(self.time[self.end_list], (self.data["pend_pos"])[self.end_list], color = "purple", marker = "^", label = "End points", s = 100)

        axes[0].set_ylabel("Position (rad)")
        axes[0].set_title("Pendulum Raw")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        # Velocity
        axes[1].plot(self.time, self.data["pend_vel"], linewidth=1, marker = ".", color = "green", label = "Velocity (Kalman)")
        axes[1].set_ylabel("Velocity (rad/s)")
        axes[1].set_xlabel("Time (s)")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        axes[1].set_xlabel("time")
        fig.tight_layout()

        plt.show()

    def add_startpoints(self, start_list):
        for entry in start_list:
            idx = np.argmin(np.abs(entry - self.time))
            self.start_list.append(idx)
        self.find_endpoints()

    def find_endpoints(self, thresh = 0.0001, hold = 100, jump = 1000):
        for cur_start in self.start_list:

            start = cur_start + jump
            
            cur_velocity = (self.data["pend_vel"])[start:]
            settled = np.abs(cur_velocity) < thresh

            # Find settling point
            for i in range(len(settled) - hold):
                if np.all(settled[i:i+hold]):
                    self.end_list.append(start+i)
                    break

    # ---------------------------------------------------------------------------
    # RK4 integrator (for non-linear simulation method)
    # ---------------------------------------------------------------------------
    def pendulum_rhs(self, state, J, b, mgl):
        """ Deriv of state """
        theta, omega = state
        dtheta = omega
        domega = -(b * omega + mgl * np.sin(theta)) / J
        return np.array([dtheta, domega])


    def rk4_step(self, state, dt, J, b, mgl):
        k1 = self.pendulum_rhs(state, J, b, mgl)
        k2 = self.pendulum_rhs(state + 0.5 * dt * k1, J, b, mgl)
        k3 = self.pendulum_rhs(state + 0.5 * dt * k2, J, b, mgl)
        k4 = self.pendulum_rhs(state + dt * k3, J, b, mgl)
        return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


    def simulate_theta(self, t, theta0, omega0, J, b, mgl):
        """RK4-integrate theta(t) at the exact (possibly non-uniform) sample times in t."""
        state = np.array([theta0, omega0], dtype=float)
        out = np.empty(len(t))
        out[0] = state[0]
        for i in range(len(t) - 1):
            dt = t[i + 1] - t[i]
            state = self.rk4_step(state, dt, J, b, mgl)
            out[i + 1] = state[0]
        return out


    # ---------------------------------------------------------------------------
    # Fitting helpers
    # ---------------------------------------------------------------------------

    def _implement_filtfilt(self, signal, order = 2, cutoff_hz = 100, fs = 1000):
        nyquist = fs / 2.0
        normal_cutoff = cutoff_hz / nyquist
        b, a = butter(order, normal_cutoff, btype='low')
        return filtfilt(b, a, signal)

    def _generate_runs(self):
        self.runs = []  # Clear to start
        for start, end in zip(self.start_list, self.end_list):
            cur_t = self.time[start:end]
            cur_pos = (self.data["pend_pos"])[start:end]
            self.runs.append((cur_t, cur_pos))

    def test_fit(self, J, b, mgl):
        """ For each run, copmare actual to forward simulation """
        self._generate_runs()
        for t, theta in self.runs:

            # Forward simulation
            theta_sim = self.simulate_theta(t, theta[0], 0.0, J, b, mgl)

            # Compare to actual
            residuals = (theta_sim - theta)**2

            # Plot
            fig, axs = plt.subplots(2,1, figsize=(10,6), sharex = True)
            axs[0].plot(t, theta, label = "Measured")
            axs[0].plot(t, theta_sim, label = "Predicted")
            axs[0].set_ylabel("Position (rad)")
            axs[0].set_title("Linear Method Evaluation")
            axs[0].legend()
            axs[0].grid(True, alpha=0.3)
    
            # Velocity
            axs[1].plot(t, residuals, linewidth=1, marker = ".", color = "green", label = "Square Error")
            axs[1].set_ylabel("Residual (rad^2)")
            axs[1].set_xlabel("Time (s)")
            axs[1].legend()
            axs[1].grid(True, alpha=0.3)
    
            fig.tight_layout()
    
            plt.show()

    # ---------------------------------------------------------------------------
    # Method 1: derivative-based, linear least squares (with bounds via least_squares)
    # ---------------------------------------------------------------------------
    def fit_linear(self, mgl, debug = True):
        """
        Uses numerical diff and filtering to get omega and alpha.
        Returns: OptimizeResult from least_squares (result.x = [J, b])
        """

        A_blocks, y_blocks = [], []

        self._generate_runs()
        for t, theta in self.runs:
            dt = np.mean(np.diff(t))
            if not np.allclose(np.diff(t), dt, rtol=1e-3):
                raise ValueError("Method 1 requires (approximately) uniform sampling per run; "
                                "resample/interpolate onto a uniform grid first.")

            theta_d_raw = np.gradient(theta, dt)
            theta_d = self._implement_filtfilt(theta_d_raw, cutoff_hz= 30.0)

            theta_dd_raw = np.gradient(theta_d, dt)
            theta_dd = self._implement_filtfilt(theta_dd_raw, cutoff_hz=30.0)

            A_blocks.append(np.column_stack([theta_dd, theta_d]))   # coefficients of [J, b]
            y_blocks.append(-mgl * np.sin(theta))                   # known-mgl term moved to RHS

            if debug:
                fig, axs = plt.subplots(1, 1, figsize = (10,6), sharex = True)
                axs.plot(t, theta, label = "Theta")
                axs.plot(t, theta_d, label = "Omega")
                axs.plot(t, theta_dd, label = "Alpha")
                axs.legend()
                
                plt.show()

        A = np.vstack(A_blocks)
        y = np.concatenate(y_blocks)

        def residuals(params):
            J, b = params
            return A @ np.array([J, b]) - y

        def jac(params):
            return A  # linear model -> constant Jacobian

        x0 = [1e-4, 1e-4]  # rough starting guess; replace with your CAD estimate if you have one
        result = least_squares(residuals, x0, jac=jac, bounds=([1e-12, 0], [np.inf, np.inf]))
        return result

    # ---------------------------------------------------------------------------
    # Method 2: forward-simulation (RK4) + nonlinear least squares
    # ---------------------------------------------------------------------------

    def fit_nonlinear(self, mgl, J0, b0, omega0=0.0):
        """
        mgl: known gravity term
        J0: initial inertia guess
        b0: initial damping guess
        omega0: initial angular velocity for every run (0.0 for lift-and-drop-from-rest).
                Pass a list instead if release velocity varies run to run.
        """
        self._generate_runs()
        N = len(self.runs)
        omega0_list = omega0 if hasattr(omega0, "__len__") else [omega0] * N

        def residuals(params):
            J, b = params
            res = []
            for (t, theta), w0 in zip(self.runs, omega0_list):
                sim = self.simulate_theta(t, theta[0], w0, J, b, mgl)
                res.append(sim - theta)
            return np.concatenate(res)

        result = least_squares(
            residuals, x0=[J0, b0],
            bounds=([1e-12, 0], [np.inf, np.inf]),
            method="trf",
            loss="soft_l1",   # mild robustness to outlier samples/glitches; use 'linear' for plain LS
            x_scale=[max(J0, 1e-9), max(b0, 1e-9)],  # helps scipy when J and b have very different magnitudes
            verbose=2,
        )
        return result

    # ---------------------------------------------------------------------------
    # Method 3: forward-simulation (RK4) with coulombic friction + nonlinear least squares
    # ---------------------------------------------------------------------------
    def pendulum_fs_rhs(self, state, J, b, mgl, f_s, k):
        """ Deriv of state """
        theta, omega = state
        dtheta = omega
        domega = -(b * omega + mgl * np.sin(theta) + f_s * np.tanh(k*omega)) / J
        return np.array([dtheta, domega])


    def rk4_fs_step(self, state, dt, J, b, mgl, f_s, k):
        k1 = self.pendulum_fs_rhs(state, J, b, mgl, f_s, k)
        k2 = self.pendulum_fs_rhs(state + 0.5 * dt * k1, J, b, mgl, f_s, k)
        k3 = self.pendulum_fs_rhs(state + 0.5 * dt * k2, J, b, mgl, f_s, k)
        k4 = self.pendulum_fs_rhs(state + dt * k3, J, b, mgl, f_s, k)
        return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


    def simulate_fs_theta(self, t, theta0, omega0, J, b, mgl, f_s, k):
        """RK4-integrate theta(t) at the exact (possibly non-uniform) sample times in t."""
        state = np.array([theta0, omega0], dtype=float)
        out = np.empty(len(t))
        out[0] = state[0]
        for i in range(len(t) - 1):
            dt = t[i + 1] - t[i]
            state = self.rk4_fs_step(state, dt, J, b, mgl, f_s, k)
            out[i + 1] = state[0]
        return out

    def fit_fs_nonlinear(self, mgl, J0, b0, fs0, omega0=0.0, k=10):
        """
        mgl: known gravity term
        J0: initial inertia guess
        b0: initial damping guess
        fs0: initial coulombic friction guess
        omega0: initial angular velocity for every run (0.0 for lift-and-drop-from-rest).
                Pass a list instead if release velocity varies run to run.
        k: factor to use for tanh. NOT learned. Functionally a hyperparameter
        """
        self._generate_runs()
        N = len(self.runs)
        omega0_list = omega0 if hasattr(omega0, "__len__") else [omega0] * N

        def residuals(params):
            J, b, fs = params
            res = []
            for (t, theta), w0 in zip(self.runs, omega0_list):
                sim = self.simulate_fs_theta(t, theta[0], w0, J, b, mgl, fs, k)
                res.append(sim - theta)
            return np.concatenate(res)

        result = least_squares(
            residuals, x0=[J0, b0, fs0],
            bounds=([1e-12, 0, 0], [np.inf, np.inf, np.inf]),
            method="trf",
            loss="soft_l1",   # mild robustness to outlier samples/glitches; use 'linear' for plain LS
            x_scale=[max(J0, 1e-9), max(b0, 1e-9), max(fs0, 1e-9)],  # helps scipy when J and b have very different magnitudes
            verbose=2,
        )
        return result

    def test_fs_fit(self, J, b, mgl, f_s, k = 10):
        """ For each run, copmare actual to forward simulation """
        self._generate_runs()
        for t, theta in self.runs:

            # Forward simulation
            theta_sim = self.simulate_fs_theta(t, theta[0], 0.0, J, b, mgl, f_s, k)

            # Compare to actual
            residuals = (theta_sim - theta)**2

            # Plot
            fig, axs = plt.subplots(2,1, figsize=(10,6), sharex = True)
            axs[0].plot(t, theta, label = "Measured")
            axs[0].plot(t, theta_sim, label = "Predicted")
            axs[0].set_ylabel("Position (rad)")
            axs[0].set_title("Linear Method Evaluation")
            axs[0].legend()
            axs[0].grid(True, alpha=0.3)
    
            # Velocity
            axs[1].plot(t, residuals, linewidth=1, marker = ".", color = "green", label = "Square Error")
            axs[1].set_ylabel("Residual (rad^2)")
            axs[1].set_xlabel("Time (s)")
            axs[1].legend()
            axs[1].grid(True, alpha=0.3)
    
            fig.tight_layout()
    
            plt.show()


# ---------------------------------------------------------------------------
# Fit dat data
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    base_path = "/Users/trevorperey/Desktop/PersonalProjects/ros2_odrive_personal/"
    pend_log = base_path + "src/ps5_odrive_control/ps5_odrive_control/logs/pend_drop_sid_001/logs_20260801_124712.pkl"

    analyzer = PendulumAnalyzer(pend_log)
    analyzer.add_startpoints([10.364, 25.072, 42.481, 55.682, 71.790, 88.580, 102.856, 117.666, 133.164, 149.647, 168.993, 186.630, 207.018, 233.207, 253.556])
    #analyzer.add_startpoints([10.364])
    #analyzer.plot_raw()

    m = 38.03/1000.0  # [kg]
    g = 9.81  # [m/s^2]
    l = (77.171 - 30.0)/1000.0  # [m]

    # Linear fit
    linear_fit = analyzer.fit_linear(mgl=(m*g*l), debug=False)

    J_fit, b_fit = linear_fit.x
    print(f"Result of Linear Method: J = {J_fit}, b = {b_fit}")

    #analyzer.test_fit(J_fit, b_fit, (m*g*l))

    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

    """ Non-linear fit """
    nonlinear_fit = analyzer.fit_nonlinear((m*g*l), 1e-4, 1e-4, 0.0)

    J_n_fit, b_n_fit = nonlinear_fit.x
    print(f"Result of NON-Linear Method: J = {J_n_fit}, b = {b_n_fit}")

    analyzer.test_fit(J_n_fit, b_n_fit, (m*g*l))

    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

    """ Non-linear fit, with friction """
    # k_to_use = 10
    # fs_nonlinear_fit = analyzer.fit_fs_nonlinear((m*g*l), 1e-4, 1e-4, 0.001, 0.0, k_to_use)

    # Jnfs_fit, bnfs_fit, fs_fit = fs_nonlinear_fit.x
    # print(f"Result of Nonlinear method WITH FRICTION: J = {Jnfs_fit}, b = {bnfs_fit}, fs = {fs_fit}")

    # analyzer.test_fs_fit(Jnfs_fit, bnfs_fit, (m*g*l), fs_fit, k_to_use)