import numpy as np
from scipy.signal import savgol_filter
from numpy.linalg import lstsq
import matplotlib.pyplot as plt

import pickle
from pathlib import Path


def fit_mass_damping_friction(tel_dict, vel_thresh_ratio=0.01, plot=True):
    """
    Fit a rotational system model (J, b, static friction, dynamic friction) from sys ID data.

    Args:
        tel_dict: dict from SysIdOdrive.tel_dict
        vel_thresh_ratio: fraction of max velocity to define "moving"
        plot: whether to show verification plots

    Returns:
        params: dict with keys ['J', 'b', 'tau_static', 'tau_dynamic']
    """
    # Extract data
    t = np.array(tel_dict['time'])
    theta = np.array(tel_dict['pos_actual'])
    vel = np.array(tel_dict['vel_actual'])
    tau = np.array(tel_dict['smooth_tau_actual'])
    #tau = np.array(tel_dict['tau_cmd'])

    # Compute acceleration (smooth with Savitzky-Golay filter)
    dt = np.mean(np.diff(t))
    acc = np.gradient(vel, dt)
    acc_smooth = savgol_filter(acc, window_length=101, polyorder=3)

    # Identify moving vs stationary points
    vel_thresh = vel_thresh_ratio * np.max(np.abs(vel))
    moving = np.abs(vel) > vel_thresh
    stationary = ~moving

    # Estimate static friction
    # Option: peak torque at standstill
    tau_static_est_PK = np.max(np.abs(tau[stationary]))
    print(f"tau_static_est_PK = {tau_static_est_PK}")

    # Option: mean at start
    crossings = np.where((np.abs(vel[:-1]) <= vel_thresh) & (np.abs(vel[1:]) > vel_thresh))[0] + 1
    tau_static_samples = np.abs(tau[crossings])
    tau_static_est_mn = np.mean(tau_static_samples)
    print(f"tau_static_est_mn: {tau_static_est_mn:.4f} N·m")
    
    # Fit dynamic model using moving points: tau = J*acc + b*vel + tau_dynamic*sign(vel)
    X = np.vstack([acc_smooth[moving], vel[moving], np.sign(vel[moving])]).T
    y = tau[moving]
    theta_hat, residuals, rank, s = lstsq(X, y, rcond=None)
    J_fit, b_fit, tau_dynamic_fit = theta_hat

    # Prepare plots
    if plot:
        tau_model = np.zeros_like(tau)
        tau_model[moving] = J_fit*acc_smooth[moving] + b_fit*vel[moving] + tau_dynamic_fit*np.sign(vel[moving])
        tau_model[stationary] = 0.0  # assume no motion torque while stationary

        # --- 3-row subplot ---
        fig, axs = plt.subplots(3, 1, figsize=(12,12), sharex=True)

        # --- Position ---
        axs[0].plot(t, theta, 'b-', label='Position [rad]')
        axs[0].set_ylabel('Position [rad]')
        axs[0].legend(loc='upper right')
        axs[0].grid(True, alpha=0.3)
        axs[0].set_title('System Identification Results')

        # --- Velocity ---
        axs[1].plot(t, vel, 'g-', label='Velocity [rad/s]')
        axs[1].set_ylabel('Velocity [rad/s]')
        axs[1].legend(loc='upper right')
        axs[1].grid(True, alpha=0.3)

        # --- Torque ---
        axs[2].plot(t, tau, 'r', label='Measured torque')
        axs[2].plot(t, tel_dict['tau_cmd'], 'g--', label='Commanded torque')
        axs[2].plot(t, tau_model, 'b--', label='Dynamic model')
        axs[2].axhline(tau_static_est_PK, color='k', linestyle=':', label='Static friction')
        axs[2].axhline(-tau_static_est_PK, color='k', linestyle=':')
        axs[2].set_xlabel('Time [s]')
        axs[2].set_ylabel('Torque [N·m]')
        axs[2].legend(loc='upper right')
        axs[2].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()


    # Return parameters
    params = {
        'J': J_fit,
        'b': b_fit,
        'tau_static_PK': tau_static_est_PK,
        'tau_static_mn': tau_static_est_mn,
        'tau_dynamic': tau_dynamic_fit
    }

    print("Fitted parameters:")
    print(f"  Inertia J: {J_fit:.4f} kg·m²")
    print(f"  Viscous damping b: {b_fit:.4f} N·m·s/rad")
    print(f"  Static friction tau_s PEAK: {tau_static_est_PK:.4f} N·m")
    print(f"  Static friction tau_s mean: {tau_static_est_mn:.4f} N·m")
    print(f"  Dynamic friction tau_d: {tau_dynamic_fit:.4f} N·m")

    return params

def main():
    # --- 1. Load pickle file ---
    # data_dir = Path('sys_id_data')  # your folder where pickles are saved
    # # Find the latest pickle (tel_dict version)
    # pickles = sorted(data_dir.glob('sys_id_data_*.pkl'))
    # if not pickles:
    #     raise FileNotFoundError(f"No pickle files found in {data_dir}")
    
    # latest_pickle = pickles[-1]  # take latest one
    latest_pickle = Path('sys_id_data/sys_id_data_nonlin_amp0.11_20251206_162618.pkl')
    print(f"Loading data from {latest_pickle}")
    
    with open(latest_pickle, 'rb') as f:
        tel_dict = pickle.load(f)
    
    # --- 2. Fit mass, damping, static & dynamic friction ---
    params = fit_mass_damping_friction(tel_dict, vel_thresh_ratio=0.01, plot=True)

    # --- 3. Optionally: save parameters ---
    # params_file = data_dir / f"sysid_params.pkl"
    # with open(params_file, 'wb') as f:
    #     pickle.dump(params, f)
    # print(f"Fitted parameters saved to {params_file}")

if __name__ == "__main__":
    main()
