import pickle
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import resample, find_peaks
from scipy.signal import butter, filtfilt
import json
from pathlib import Path

class PostprocessLinearTau:
    def __init__(self, pickle_path):
        self.pickle_path = pickle_path

        with open(pickle_path, "rb") as f:
            self.data = pickle.load(f)

        if not isinstance(self.data, dict):
            raise TypeError(
                f"Expected pickle to contain dict, got {type(self.data)}"
            )

        #self.data["tau_actual"] = -1*np.asarray(self.data["tau_actual"])

    def keys(self):
        return list(self.data.keys())

    def summary(self):
        for key, value in self.data.items():
            try:
                shape = np.shape(value)
            except Exception:
                shape = "N/A"

            print(f"{key:20s} | {type(value).__name__:15s} | {shape}")

    def plot(self, x_key, y_key, **plot_kwargs):
        x = np.asarray(self.data[x_key])
        y = np.asarray(self.data[y_key])

        plt.figure(figsize=(10, 5))
        plt.plot(x, y, **plot_kwargs)
        plt.xlabel(x_key)
        plt.ylabel(y_key)
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    def plot_key(self, key, **plot_kwargs):
        y = np.asarray(self.data[key])
        x = np.arange(len(y))

        plt.figure(figsize=(10, 5))
        plt.plot(x, y, **plot_kwargs)
        plt.xlabel("Index")
        plt.ylabel(key)
        plt.grid(True)
        plt.tight_layout()
        plt.show()
    
    def plot_linear(self):

        t = np.array(self.data["time"])
        p0_actual = np.array(self.data["pos_actual"])
        v0_actual = np.array(self.data["vel_actual"])
        tau_cmd = np.array(self.data["tau_cmd"])
        tau_actual = np.array(self.data["tau_actual"])

        fig, axs = plt.subplots(3,1, figsize=(12,4), sharex = True)

        axs[0].plot(t, p0_actual, label = "pos_actual", color = "blue")
        axs[0].legend()

        axs[1].plot(t, v0_actual, label = "vel_actual", color = "green")
        axs[1].legend()

        axs[2].plot(t, tau_cmd, label = "tau_cmd", color = "black")
        axs[2].scatter(t, tau_actual, color = "red", s = 10, label = "tau_actual")
        axs[2].legend()

        plt.show()
    
    def plot_linear_motoroff(self):

        t = np.array(self.data["time"])
        p0_actual = np.array(self.data["pos_actual"])
        v0_actual = np.array(self.data["vel_actual"])
        tau_cmd = np.array(self.data["tau_cmd"])
        tau_actual = np.array(self.data["tau_actual"])

        is_motor_on = (tau_cmd != 0.0)
        t = t[~is_motor_on]
        p0_actual = p0_actual[~is_motor_on]
        v0_actual = v0_actual[~is_motor_on]
        tau_cmd = tau_cmd[~is_motor_on]
        tau_actual = tau_actual[~is_motor_on]

        fig, axs = plt.subplots(3,1, figsize=(12,4), sharex = True)

        axs[0].scatter(t, p0_actual, label = "pos_actual", color = "blue")
        axs[0].legend()

        axs[1].scatter(t, v0_actual, label = "vel_actual", color = "green")
        axs[1].legend()

        axs[2].scatter(t, tau_cmd, label = "tau_cmd", color = "black")
        axs[2].scatter(t, tau_actual, color = "red", s = 10, label = "tau_actual")
        axs[2].legend()

        plt.show()
    
    import numpy as np
from scipy.optimize import least_squares

def fit_motor_off_dynamics(pos, vel, dt):
    """
    Fit motor-off dynamics:
        0 = J*accel + b*vel + tau_c*sign(vel)

    Args:
        pos: position array [rad]
        vel: velocity array [rad/s]
        dt: timestep [s]

    Returns:
        J, b, tau_c
    """

    # acceleration
    accel = np.gradient(vel, dt)
    # TODO FILTER

    # remove bad points
    mask = np.isfinite(accel) & np.isfinite(vel)

    accel = accel[mask]
    vel = vel[mask]

    # regression:
    # [accel vel sign(vel)] * [J b tau_c]^T = 0
    A = np.column_stack([
        accel,
        vel,
        np.sign(vel)
    ])

    y = np.zeros(len(accel))

    # Solve Ax = 0 with constraint to avoid trivial solution
    # Instead minimize residual directly
    def residual(params):
        J, b, tau_c = params
        return J*accel + b*vel + tau_c*np.sign(vel)

    result = least_squares(
        residual,
        x0=[0.001, 0.001, 0.01]
    )

    J, b, tau_c = result.x

    return J, b, tau_c

  
if __name__ == "__main__":
    # LINEAR TORQUE
    p = PostprocessLinearTau("sid_data_pi/sys_id/linear_si_data/hr_linear_sys_id_data_20260620_172258.pkl")

    # Summary
    print(p.keys())
    p.summary()

    # Plot
    p.plot_linear_motoroff()