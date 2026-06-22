import pickle
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import resample, find_peaks
from scipy.signal import butter, filtfilt, savgol_filter
import json
from pathlib import Path

class PostprocessStepVel:
    def __init__(self, pickle_path):
        self.pickle_path = pickle_path

        with open(pickle_path, "rb") as f:
            self.data = pickle.load(f)

        if not isinstance(self.data, dict):
            raise TypeError(
                f"Expected pickle to contain dict, got {type(self.data)}"
            )

        self.data["tau_actual"] = -1*np.asarray(self.data["tau_actual"])
        
        self.split_by_vel_cmd()

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
    
    # def plot_linear(self):

    #     t = np.array(self.data["time"])
    #     p0_actual = np.array(self.data["pos_actual"]) % 2*np.pi
    #     v0_actual = np.array(self.data["vel_actual"])
    #     tau_cmd = np.array(self.data["tau_cmd"])
    #     tau_actual = np.array(self.data["tau_actual"])

    #     fig, axs = plt.subplots(3,1, figsize=(12,4), sharex = True)

    #     axs[0].plot(t, p0_actual, label = "pos_actual", color = "blue")
    #     axs[0].legend()

    #     axs[1].plot(t, v0_actual, label = "vel_actual", color = "green")
    #     axs[1].legend()

    #     axs[2].plot(t, tau_cmd, label = "tau_cmd", color = "black")
    #     axs[2].scatter(t, tau_actual, color = "red", s = 10, label = "tau_actual")
    #     axs[2].legend()

    #     plt.show()
    
    def plot_step_velocity(self):

        t = np.array(self.data["time"])
        p0_actual = np.array(self.data["pos_actual"]) #% (2*np.pi)
        v0_actual = np.array(self.data["vel_actual"])
        vel_cmd = np.array(self.data["vel_cmd"])
        tau_actual = np.array(self.data["tau_actual"])

        fig, axs = plt.subplots(3,1, figsize=(8,8), sharex = True)

        axs[0].plot(t, p0_actual, label = "pos_actual", color = "blue")
        axs[0].legend()

        axs[1].plot(t, v0_actual, label = "vel_actual", color = "green")
        axs[1].plot(t, vel_cmd, label = "tau_cmd", color = "black")
        axs[1].legend()

        
        axs[2].scatter(t, tau_actual, color = "red", s = 10, label = "tau_actual")
        axs[2].legend()

        plt.show()

    """ FFT """
    def fft_analysis(self, x, y):
        # sampling frequency
        dt = np.mean(np.diff(x))
        fs = 1 / dt

        # remove mean (important for FFT)
        y = y - np.mean(y)

        N = len(y)

        # FFT
        Y = np.fft.fft(y)
        freqs = np.fft.fftfreq(N, d=dt)

        # positive frequencies only
        mask = freqs > 0

        freqs = freqs[mask]
        magnitude = np.abs(Y[mask]) / N

        plt.figure(figsize=(10,4))
        plt.plot(freqs, magnitude)
        plt.xlabel("Frequency (Hz)")
        plt.ylabel("Magnitude")
        plt.grid(True)
        plt.show()

        return freqs, magnitude

    def spatial_fft(self, theta, current):

        # unwrap
        theta = np.unwrap(theta)

        # number of samples
        N = len(theta)

        # interpolate onto uniform angle grid
        theta_uniform = np.linspace(theta[0], theta[-1], N)

        current_uniform = np.interp(
            theta_uniform,
            theta,
            current
        )

        # FFT
        fft = np.fft.rfft(current_uniform - np.mean(current_uniform))

        # spatial frequency in cycles/revolution
        dtheta = theta_uniform[1] - theta_uniform[0]
        freq = np.fft.rfftfreq(N, d=dtheta/(2*np.pi))

        return freq, np.abs(fft)
    
    def plot_sv_vs_tau(self):
        t = np.array(self.data["time"])
        p0_actual = np.array(self.data["pos_actual"]) #% (2*np.pi)
        v0_actual = np.array(self.data["vel_actual"])
        vel_cmd = np.array(self.data["vel_cmd"])
        tau_actual = np.array(self.data["tau_actual"])

        # Spatial
        is_motor_on = np.array(self.data["is_motor_on"], dtype=bool)
        print(vel_cmd)

        for multiple in [0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0]:
            print(f"cur multiple = {multiple}")
            
            cur_mask = (vel_cmd > multiple*(2*np.pi-0.01)) & (vel_cmd < multiple*(2*np.pi+0.01)) & (is_motor_on)
            cur_p0_actual = p0_actual[cur_mask]
            cur_tau_actual = tau_actual[cur_mask]

            plt.figure(figsize=(5,5))
            plt.plot(cur_p0_actual, cur_tau_actual)
            plt.show()

            # Spatial fft
            freq, mag = self.spatial_fft(cur_p0_actual, cur_tau_actual)
            plt.plot(freq, mag)
            plt.xlim(0,100)
            plt.show()
    
    """ ANTICOGGING """
    def split_by_vel_cmd(self, vel_levels=None, tol=0.1, require_motor_on=True):
        """Split flat tel_dict into {vel_cmd: {key: array}}."""
        vel_cmd = np.asarray(self.data["vel_cmd"])
        motor_on = np.asarray(self.data.get("is_motor_on", np.ones(len(vel_cmd), dtype=bool)), dtype=bool)

        if vel_levels is None:
            # auto-detect nonzero commanded speeds
            active = vel_cmd[motor_on] if require_motor_on else vel_cmd
            vel_levels = np.unique(active[np.abs(active) > tol])

        by_vel = {}
        for level in vel_levels:
            mask = (np.abs(vel_cmd - level) < tol)
            if require_motor_on:
                mask &= motor_on

            by_vel[round(float(level), 3)] = {
                key: np.asarray(val)[mask]
                for key, val in self.data.items()
            }

        self.by_vel = by_vel
    
    def filter_by_vel(self, cutoff_dict={"tau_actual": 50.0, "vel_actual": 50.0}, order=4, inplace=True):
        """
        Low-pass filtfilt on chosen signals inside each by_vel bucket.
        Uses that bucket's own time array for sample rate.
        """
        filtered = {}

        for vel_level, data in self.by_vel.items():
            t = np.asarray(data["time"])
            if len(t) < 2:
                filtered[vel_level] = data
                continue

            # dt = np.mean(np.diff(t))
            # plt.hist(dt)
            # fs = 1.0 / dt
            # nyq = 0.5 * fs
            dt = 0.005
            fs = 1.0 / dt
            nyq = 0.5 * fs
            cutoff_hz = np.min(cutoff_dict.values())

            print(f"fs = {fs}, nyq = {nyq}, cutoff_hz = {cutoff_hz}")

            new_data = {k: np.asarray(v).copy() for k, v in data.items()}

            for key, cutoff in cutoff_dict.items():
                y = np.asarray(data[key])

                b, a = butter(order, cutoff / nyq, btype="low")
                if cutoff >= nyq:
                    raise ValueError(f"cutoff {cutoff} Hz >= Nyquist {nyq:.1f} Hz for vel={vel_level}")

                padlen = 3 * (max(len(a), len(b)) - 1)
                if len(y) <= padlen:
                    print(f"skip {key} @ vel={vel_level}: too short ({len(y)} samples)")
                    continue

                new_data[key] = filtfilt(b, a, y)

            filtered[vel_level] = new_data

        if inplace:
            self.by_vel = filtered
        else:
            return filtered
    
    def plot_by_vel_cmd(self):

        for vel_cmd, data in self.by_vel.items():
            t = np.array(data["time"])
            p0_actual = np.array(data["pos_actual"]) #% (2*np.pi)
            v0_actual = np.array(data["vel_actual"])
            vel_cmd = np.array(data["vel_cmd"])
            tau_actual = np.array(data["tau_actual"])

            # Vel plot
            fig, axs = plt.subplots(3, 1, figsize=(15,7.5))
            axs[0].plot(t, vel_cmd, label = "vel_cmd", color = 'k')
            axs[0].plot(t, v0_actual, label = "vel_actual", color = 'g')
            axs[0].legend()
            axs[0].grid(True)
            axs[0].set_title(f"By Vel CMD plot for vel_cmd = {np.mean(vel_cmd)}")

            # Spatial torque plot
            axs[1].scatter(p0_actual % (2*np.pi), tau_actual, color = 'r')
            axs[1].legend() 
            axs[1].grid(True)

            # Spatial fft
            freq, mag = self.spatial_fft(p0_actual, tau_actual)
            axs[2].plot(freq, mag)  
            axs[2].grid(True)
        
        plt.show()
    
    def tauvpos_by_vel_cmd(self):
        # fig, axs = plt.subplots(2, 4, figsize=(15,7.5), sharex = True, sharey = True)
        # axs = axs.ravel()
        # for ax, (vel_cmd, data) in zip(axs, self.by_vel.items()):
        #     p0_actual = np.array(data["pos_actual"]) #% (2*np.pi)
        #     tau_actual = np.array(data["tau_actual"])

        #     # Spatial torque plot
        #     ax.scatter(p0_actual % (2*np.pi), tau_actual, color = 'r')
        #     ax.set_title(f"vel={vel_cmd}")
        #     ax.grid(True)

        fig, ax = plt.subplots(figsize=(15, 7.5))

        for vel_cmd, data in self.by_vel.items():
            if vel_cmd < 10.0:
                p0_actual = np.array(data["pos_actual"]) #% (2*np.pi)
                tau_actual = np.array(data["tau_actual"])

                # Spatial torque plot
                plt.scatter(p0_actual % (2*np.pi), tau_actual, label = f"{vel_cmd}")

        plt.grid(True)
        plt.legend()
        plt.show()
    
    def predict_cogging(self, theta, beta, harmonics):
        """
        theta: scalar or numpy array of rotor angles [rad]
        beta: fitted least squares coefficients
        harmonics: list of fitted harmonic numbers
        """

        theta = np.asarray(theta)

        X = [np.ones_like(theta)]

        for h in harmonics:
            X.append(np.sin(h * theta))
            X.append(np.cos(h * theta))

        X = np.column_stack(X)

        return X @ beta
    
    def fit_cogging_constvel(self, vel_cmd_to_use = 6.283, evaluate = True, N_harmonics = 10, save_path = (Path("..") / "config/cogging_constvel.json")):
        import numpy as np

        # theta: rotor angle [rad]
        # tau: measured torque = Kt * current
        theta = np.array(self.by_vel[vel_cmd_to_use]["pos_actual"])
        tau = np.array(self.by_vel[vel_cmd_to_use]["tau_actual"])

        # # Get harmonics
        # freqs, mag = self.spatial_fft(theta, tau)
        # mag[0] = 0  # Ignore DC
        # peaks, _ = find_peaks(mag)  # Find peaks
        # top_peaks = peaks[np.argsort(mag[peaks])[-N_harmonics:]]
        # harmonics = sorted(freqs[top_peaks].astype(int))
        # print(harmonics)

        # FORCE harmonics
        harmonics = np.asarray([18, 36, 54, 72, 20, 40, 60, 80])

        # Now fit
        X = [np.ones_like(theta)]

        for h in harmonics:
            X.append(np.sin(h * theta))
            X.append(np.cos(h * theta))

        X = np.column_stack(X)

        # Least squares fit
        beta, *_ = np.linalg.lstsq(X, tau, rcond=None)

        # Extract results
        C = beta[0]

        cogging_fit = {
            "offset": 0.0, #float(C),
            "harmonics": {}
        }

        idx = 1

        for h in harmonics:
            a = beta[idx]
            b = beta[idx + 1]

            amplitude = np.sqrt(a**2 + b**2)
            phase = np.arctan2(b, a)

            cogging_fit["harmonics"][str(h)] = {
                "amplitude": float(amplitude),
                "phase": float(phase),
                "a": float(a),
                "b": float(b)
            }

            idx += 2
        
        print(json.dumps(cogging_fit, indent=4))

        if evaluate:
            fig, axs = plt.subplots(2, 1, figsize=(10, 5), sharex = True, sharey = True)
            tau_predict = self.predict_cogging(theta, beta, harmonics)
            axs[0].plot(theta, tau, label = "Measured")
            axs[0].plot(theta, tau_predict, label = "Predicted")
            axs[0].legend()
            axs[0].grid(True)

            # Evaulate residuals over POSITION
            residuals = (tau - tau_predict)
            axs[1].plot(theta, residuals, label = f"Residuals (mean = {np.mean(residuals):.4f})")
            axs[1].legend()
            axs[1].grid(True)
            
            plt.title(f"Cogging Fit Evaluation for vel_cmd = {vel_cmd_to_use}")
            plt.show()

            # Evaluate residuals over TIME
            t = np.array(self.by_vel[vel_cmd_to_use]["time"])
            plt.plot(t, residuals, color = "red", label = "Residuals over time")
            plt.grid(True)
            plt.legend()
            plt.show

            # Evaulate across ALL
            fig, axs = plt.subplots(3, 1, figsize=(15, 7.5), sharex = True)
            for key, cur_data in self.by_vel.items():
                t_all = np.array(cur_data["time"])
                theta_all = np.array(cur_data["pos_actual"])
                vel_cmd_all = np.array(cur_data["vel_cmd"])
                vel_actual_all = np.array(cur_data["vel_actual"])
                tau_predict_all = self.predict_cogging(theta_all, beta, harmonics)
                tau_all = np.array(cur_data["tau_actual"])
                output_all = tau_all - tau_predict_all

                
                axs[0].plot(t_all, output_all, label = f"Predicted output torque (mean = {np.mean(output_all):.4f})", color = 'k', linewidth = 2)
                axs[0].legend()
                axs[0].grid(True)
                axs[0].set_title(f"Cogging Fit Evaluation across ALL")
                axs[0].plot(t_all, tau_all,'r--', linewidth = 1, label = f"Measured motor torque (mean = {np.mean(tau_all):.4f})")
                axs[0].legend()
                axs[0].grid(True)
                axs[1].plot(t_all, vel_cmd_all, 'p--', label = f"Commanded velocity (mean = {np.mean(vel_cmd_all):.4f})")
                axs[1].plot(t_all, vel_actual_all, 'g', label = f"ACTUAL velocity")
                axs[1].legend()
                axs[1].grid(True)

                # Compare output and velocity
                #norm_output = (output_all - np.mean(output_all))/np.max(output_all - np.mean(output_all))
                norm_output = (output_all)/np.max(output_all)
                #norm_vel_actual = (vel_actual_all - np.mean(vel_actual_all))/np.max(vel_actual_all - np.mean(vel_actual_all))
                norm_vel_actual = (vel_actual_all)/np.max(vel_actual_all)

                axs[2].plot(t_all, norm_output, 'p--', label = f"NORM output torque")
                axs[2].plot(t_all, norm_vel_actual, 'g', label = f"NORM actual velocity")
                axs[2].legend()
                axs[2].grid(True)
            plt.show()
        
        # Save to config file
        save_path = Path(save_path)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        with open(save_path, "w") as f:
            json.dump(cogging_fit, f, indent=4)

        print(f"Saved cogging fit to {save_path.resolve()}")
    
    """ MORE DETAIL """
    def crop_by_vel_cmd(self, min_vel_cmd = 0.0, max_vel_cmd = 7.0):
        self.orig_by_vel = self.by_vel
        for key in list(self.by_vel.keys()):
            if key > max_vel_cmd:
                self.by_vel.pop(key, None)
            elif key < min_vel_cmd:
                self.by_vel.pop(key,None)

    def compute_accel_from_pos(self, cutoff = 40.0):
        # t [s]
        # pos [rad]
        for key, cur_data in self.by_vel.items():
            t_all = np.array(cur_data["time"])
            theta_all = np.array(cur_data["pos_actual"])
            vel_cmd_all = np.array(cur_data["vel_cmd"])
            vel_actual_all = np.array(cur_data["vel_actual"])
            tau_all = np.array(cur_data["tau_actual"])

            # FILTER
            # Butterworth low-pass
            dt = 0.005
            fs = 1.0 / dt
            order = 4

            b, a = butter(
                order,
                cutoff / (fs/2),
                btype='low'
            )

            vel_filt = filtfilt(b, a, vel_actual_all)

            # Second derivative
            accel = np.gradient(
                vel_filt,
                dt
            )
            accel_filt = filtfilt(b, a, accel)

            # Filter tau
            tau_filt = filtfilt(b, a, tau_all)

            # Show result
            fig, axs = plt.subplots(3,1, figsize = (15, 7.5), sharex = True)

            axs[0].plot(t_all, theta_all % (2*np.pi), 'p--', label = "Raw position")
            axs[0].legend()
            axs[0].grid(True)

            axs[1].plot(t_all, vel_cmd_all, 'p--', label = f"Commanded velocity (mean = {np.mean(vel_cmd_all):.4f})")
            axs[1].plot(t_all, vel_actual_all, 'g-', label = f"ACTUAL velocity")
            axs[1].plot(t_all, vel_filt, 'o-', label = f"Filtered Velocity")
            axs[1].legend()
            axs[1].grid(True)
            axs[1].set_ylim(0.0*key, 2.0*key)

            axs[2].plot(t_all, accel, 'p-', label = f"RAW acceleration (gradient)")
            axs[2].plot(t_all, accel_filt, 'p-', label = f"FILTERED acceleration (butter)")
            axs[2].plot(t_all, tau_all, 'k-', label = f"RAW torque")
            axs[2].legend()
            axs[2].grid(True)
            axs[2].set_ylim(-120, 120)

            self.by_vel[key]["accel"] = accel
            self.by_vel[key]["accel_filt"] = accel
            self.by_vel[key]["vel_filt"] = vel_filt
            self.by_vel[key]["tau_filt"] = tau_filt
        plt.show()
    
    def predict_cogging_accel(self, theta, accel, vel, beta, harmonics):
        """
        theta: scalar or numpy array of rotor angles [rad]
        beta: fitted least squares coefficients
        harmonics: list of fitted harmonic numbers
        """

        theta = np.asarray(theta)

        X = [accel, np.sign(vel), vel]

        for h in harmonics:
            X.append(np.cos(h * theta))
            X.append(np.sin(h * theta))

        X = np.column_stack(X)

        return X @ beta
        
    def fit_cogging_accelonly(self, evaluate = True):
        theta_all = []
        vel_all = []
        acc_all = []
        tau_all = []
        time_all = []

        for key, cur_data in self.by_vel.items():

            time_all.append(np.array(cur_data["time"]))

            theta_all.append(
                np.array(cur_data["pos_actual"])
            )

            vel_all.append(
                np.array(cur_data["vel_filt"])
            )

            acc_all.append(
                np.array(cur_data["accel"])
            )

            tau_all.append(
                np.array(cur_data["tau_filt"])
            )


        # combine into one giant dataset
        time_all = np.concatenate(time_all)
        theta_all = np.concatenate(theta_all)
        vel_filt_all = np.concatenate(vel_all)
        acc_all = np.concatenate(acc_all)
        tau_filt_all = np.concatenate(tau_all)


        print("Samples:", len(theta_all))

        # Build regression matrix
        X = []

        # inertia term
        X.append(acc_all)

        # Couloumb friction
        X.append(np.sign(vel_filt_all))

        # Viscous friction
        X.append(vel_filt_all)

        # cogging harmonics
        harmonics = [18, 36, 20, 40]
        for k in harmonics:
            X.append(np.cos(k * theta_all))
            X.append(np.sin(k * theta_all))

        X = np.column_stack(X)


        # Solve least squares
        coeff, residuals, rank, s = np.linalg.lstsq(
            X,
            tau_filt_all,
            rcond=None
        )
        print(f"mean residuals = {np.mean(residuals)}")


        # Extract inertia
        J = coeff[0]
        tau_c = coeff[1]
        b = coeff[2]

        print("J =", J)
        print("Coulomb friction =", tau_c)
        print("Viscous friction =", b)

        # Extract cogging coefficients
        cog_coeff = coeff[3:]


        # Generate cogging map
        theta_map = np.linspace(
            0,
            2*np.pi,
            2048,
            endpoint=False
        )

        cog_map = np.zeros_like(theta_map)

        idx = 0

        for k in harmonics:

            A = cog_coeff[idx]
            B = cog_coeff[idx+1]

            cog_map += (
                A*np.cos(k*theta_map)
                +
                B*np.sin(k*theta_map)
            )

            idx += 2
        print(f"cog_map = {cog_map}")

        plt.scatter(acc_all, tau_filt_all, s=1)
        plt.xlabel("acceleration")
        plt.ylabel("torque")
        plt.grid()

        if evaluate:
            fig, axs = plt.subplots(3, 1, figsize=(10, 5), sharex = True, sharey = True)
            tau_predict = self.predict_cogging_accel(theta_all, acc_all, vel_filt_all, coeff, harmonics)
            axs[0].plot(time_all, tau_filt_all, label = "Measured")
            axs[1].plot(time_all, tau_predict, label = "Predicted")
            axs[2].plot(time_all, tau_filt_all-tau_predict, label = f"Residual (mean = {np.mean(tau_filt_all-tau_predict)})")
            for i in range(3):
                axs[i].legend()
                axs[i].grid(True)
            plt.show()


if __name__=="__main__":

    # STEP VELOCITY - mostly for cogging
    p = PostprocessStepVel("sid_data_pi/sys_id/step_vel_si_data/vel_step_sys_id_data_20260620_171218.pkl")

    # Summary
    print(p.keys())
    p.summary()

    # Cogging ACCEL BASED
    p.crop_by_vel_cmd()
    p.compute_accel_from_pos()
    p.fit_cogging_accelonly()

    # Cogging NO ACCEL
    #p.plot_step_velocity()
    # p.plot_sv_vs_tau()
    #p.filter_by_vel()
    p.tauvpos_by_vel_cmd()
    #p.plot_by_vel_cmd()
    p.fit_cogging_constvel()

    # # Linear
    # #p.plot_linear()