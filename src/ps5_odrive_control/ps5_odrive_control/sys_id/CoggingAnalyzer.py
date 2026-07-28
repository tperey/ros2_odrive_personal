"""
motor_telemetry_plotter.py

Simple loader/plotter for motor telemetry logs stored as a pickled dict:

    {
        'time': [...],
        'pos': [...],
        'vel': [...],
        'iq_set': [...],
        'iq_act': [...],
        'tau_set': [...],
        'tau_act': [...],
        'cmd': [...],
    }
"""

import pickle
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import norm, normaltest
import sys

COUNTS_PER_REV = 4096 #1024

class CoggingAnalyzer:
    def __init__(self, log_path, config_path):

        # Handle paths
        self.config_folder = Path(config_path)
        self.log_path = Path(log_path)
        with open(self.log_path, "rb") as f:
            log = pickle.load(f)

        # Convert everything to numpy arrays
        self.data = {k: np.asarray(v) for k, v in log.items()}
        self.time = self.data.get("time")

        # Post process
        self._postprocess()

    """ GENERAL """
    def summary(self):
        print(f"Loaded: {self.path}")
        for k, v in self.data.items():
            print(f"  {k:10s} len={len(v):<6d} min={np.nanmin(v):.4g} max={np.nanmax(v):.4g}")

    def plot_all(self):
        """One subplot per signal (excluding time), sharing the x-axis."""
        keys = [k for k in self.data if k != "time"]
        fig, axes = plt.subplots(len(keys), 1, figsize=(10, 2 * len(keys)), sharex=True)
        for ax, k in zip(axes, keys):
            ax.plot(self.time, self.data[k], linewidth=1)
            ax.set_ylabel(k)
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel("time")
        fig.tight_layout()

    def plot_tau_pos(self):
        """time vs tau_act and time vs pos, stacked in 2 rows, 1 col, shared x."""
        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

        axes[0].plot(self.time, self.data["tau_act"], linewidth=1, marker = ".", color = "green", label = "Measured")
        axes[0].plot(self.time, self.data["tau_act_corrected"],linewidth=1, marker = ".", color = "blue", label = "Centered")
        axes[0].set_ylabel("tau_act (N-m)")
        axes[0].set_title("Cogging Analysis Summary")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(self.time, self.data["pos"], linewidth=1, marker = "x", color = "black", label = "Actual")
        axes[1].plot(self.time, self.data["cmd"], linewidth = 0.5, color = "blue", label = "Cmd")
        axes[1].plot(self.time, self.data["counts_act"]/COUNTS_PER_REV, linewidth = 1, color = "red", linestyle = '--')
        axes[1].plot(self.time, self.data["counts_set"]/COUNTS_PER_REV, linewidth = 1, color = "purple", linestyle = '--')
        axes[1].set_ylabel("Pos (rev)")
        axes[1].set_xlabel("Time (ms)")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        axes[1].set_xlabel("time")
        fig.tight_layout()

    def _postprocess(self, testPlot = False, vel_thresh = 0.03):
        # Parse directions
        self._vel_cmd = np.diff(np.array(self.data["cmd"]))*1000 #[rev/ms] * [1000 ms/s] = rev/s
        self._vel_cmd = np.insert(self._vel_cmd, 0, 0)

        # Round to encoder
        encoder_counts = np.round(np.array(self.data["pos"])*COUNTS_PER_REV)
        cmd_counts = np.round(np.array(self.data["cmd"])*COUNTS_PER_REV)
        self.data["counts_act"] = encoder_counts
        self.data["counts_set"] = cmd_counts

        # Reduce to valid indices (where vel_cmd not weird)
        mask = np.abs(self._vel_cmd) <= vel_thresh

        for k in self.data.keys():
            self.data[k] = self.data[k][mask]

        self.time = self.data["time"]
        self._vel_cmd = self._vel_cmd[mask]

        # Remove, save avg currents
        pos_mask = self._vel_cmd > 0.0
        pos_currents = (np.array(self.data["tau_act"]))[pos_mask]
        self.pos_offset = np.mean(pos_currents)
        print(f"DC offset for POSITIVE currents = {self.pos_offset}")

        neg_mask = self._vel_cmd < 0.0
        neg_currents = (np.array(self.data["tau_act"]))[neg_mask]
        self.neg_offset = np.mean(neg_currents)
        print(f"DC offset for NEGATIVE currents = {self.neg_offset}")

        tau_act = self.data["tau_act"]
        tau_corrected = np.copy(tau_act)
        tau_corrected[pos_mask] = tau_act[pos_mask] - self.pos_offset
        tau_corrected[neg_mask] = tau_act[neg_mask] - self.neg_offset
        self.data["tau_act_corrected"] = tau_corrected

        # Test plot
        if testPlot:
            plt.plot(self.time, self._vel_cmd, marker = "x")


    """ COGGING "BY COUNT" ANALYSIS"""
    def analyze_tauc_vs_count(self):
        self._parse_tau_by_count()
        fig, axes = plt.subplots(1, 1, figsize=(10, 6), sharex=True)

        # Spread
        axes.scatter(self.counts_wrapped, self.data["tau_act_corrected"], label = "Datapoint")
        axes.set_xlabel("Encoder count")
        axes.set_ylabel("tau_act_corrected (N-m)")
        axes.grid(True, alpha=0.3)

        # AVERAGES
        ave_tau_by_count = []
        std_tau_by_count = []
        for tau_dataset in self.tau_by_counts:
            ave_tau_by_count.append(np.mean(tau_dataset))
            std_tau_by_count.append(np.std(tau_dataset))
        axes.plot(range(COUNTS_PER_REV), ave_tau_by_count, color = "red", linewidth = 2, label = "Mean with STD bars")
        axes.set_title("tau_act vs space")
        axes.errorbar(
            range(COUNTS_PER_REV),
            ave_tau_by_count,
            yerr=std_tau_by_count,
            color="red",
            linewidth=0.5,
            capsize=2
        )
        axes.grid()
        axes.legend()

        fig.tight_layout()

        # Save and show supplementary plots
        self.ave_tau_by_count = np.array(ave_tau_by_count)
        self._fft_tau_by_count()
        self._sample_tau_hists()


    def _parse_tau_by_count(self):
        self.counts_wrapped = self.data["counts_act"] % COUNTS_PER_REV

        tau_act_corrected = self.data["tau_act_corrected"]
        self.tau_by_counts = [tau_act_corrected[self.counts_wrapped == c] for c in range(COUNTS_PER_REV)]    

    def _fft_tau_by_count(self):
        ave_tau_by_count = self.ave_tau_by_count

        # Confirm no DC offset
        print(f"After correction, DC offset is {np.mean(ave_tau_by_count)}")
        tau_fft_input = ave_tau_by_count - np.mean(ave_tau_by_count)

        # FFT
        fft_vals = np.fft.rfft(tau_fft_input)
        fft_mag = np.abs(fft_vals)

        # Harmonic numbers
        harmonics = np.arange(len(fft_mag))

        plt.figure(figsize=(10,4))
        plt.stem(harmonics, fft_mag)
        plt.xlim([0, 360.0])
        plt.title("tau_act Spatial FFT")
        plt.xlabel("tau_act Spatial harmonic")
        plt.ylabel("tau_act Magnitude")
        plt.grid(True, alpha=0.3)

    def _sample_tau_hists(self, n_plots=3):

            for i in range(n_plots):
                plt.figure(figsize=(10,4))

                cur_hist_idx = np.random.randint(0, COUNTS_PER_REV)
                samples = self.tau_by_counts[cur_hist_idx]

                # Skip sparse bins
                if len(samples) < 20:
                    continue

                # Histogram
                count, bins, _ = plt.hist(
                    samples,
                    bins=30,
                    density=True,
                    alpha=0.6
                )

                # Fit Gaussian
                mu, sigma = norm.fit(samples)

                x = np.linspace(bins[0], bins[-1], 200)
                pdf = norm.pdf(x, mu, sigma)
                plt.plot(x, pdf, linewidth=2)

                # Normality test
                stat, p_value = normaltest(samples)

                plt.title(
                    f"Histogram of tau_act at Count {cur_hist_idx}: "
                    f"$\\mu={mu:.4f}$, $\\sigma={sigma:.4f}$\n"
                    f"K²={stat:.2f}, p={p_value:.3g}"
                )

                plt.xlabel("tau_act_corrected")
                plt.ylabel("Probability density")
                plt.grid(True, alpha=0.3)

    """ OUTPUT TO CONFIG """
    def output_to_json(self):

        # Setup
        self.config_folder.mkdir(parents=True, exist_ok=True)
        self.ave_tau_by_count = np.array(self.ave_tau_by_count)

        self.cogging_file = self.config_folder / "cogging_map.json"
        self.friction_file = self.config_folder / "friction_from_cogging.json"
        friction = 0.5*(self.pos_offset-self.neg_offset)

        with open(self.cogging_file, "w") as f:
            json.dump(self.ave_tau_by_count.tolist(), f, indent=2)
        print(f"Saved cogging map to {self.cogging_file}")

        with open(self.friction_file, "w") as f:
            json.dump(friction, f, indent=2)
        print(f"Saved friction output to {self.friction_file}")
        
if __name__ == "__main__":
    base_path = "/Users/trevorperey/Desktop/PersonalProjects/ros2_odrive_personal/"
    cog_log = base_path + "src/ps5_odrive_control/ps5_odrive_control/logs/cogging_personal_001/logs_20260720_203053.pkl"
    config_path = base_path + "config/m8325s_furata/"

    mt = CoggingAnalyzer(cog_log, config_path)
    mt.plot_tau_pos()
    mt.analyze_tauc_vs_count()
    #plt.show()
    mt.output_to_json()