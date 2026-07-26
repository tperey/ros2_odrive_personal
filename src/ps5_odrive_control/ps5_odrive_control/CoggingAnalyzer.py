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
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import sys


class CoggingAnalyzer:
    def __init__(self, path):
        self.path = Path(path)
        with open(self.path, "rb") as f:
            log = pickle.load(f)

        # Convert everything to numpy arrays
        self.data = {k: np.asarray(v) for k, v in log.items()}
        self.time = self.data.get("time")

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
        plt.show()

    def plot_tau_pos(self):
        """time vs tau_act and time vs pos, stacked in 2 rows, 1 col, shared x."""
        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

        axes[0].plot(self.time, self.data["tau_act"], linewidth=1)
        axes[0].set_ylabel("tau_act")
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(self.time, self.data["pos"], linewidth=1)
        axes[1].set_ylabel("pos")
        axes[1].grid(True, alpha=0.3)

        axes[1].set_xlabel("time")
        fig.tight_layout()
        plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python CoggignAnalyzer.py <path_to_pickle>")
        sys.exit(1)
    mt = CoggingAnalyzer(sys.argv[1])
    mt.summary()
    mt.plot_tau_pos()