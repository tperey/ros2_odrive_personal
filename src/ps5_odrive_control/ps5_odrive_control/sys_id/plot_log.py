#!/usr/bin/env python3
"""
Quick viewer for telemetry logs saved as .pkl by LoggerNode.save_plot_log().

Usage:
    python plot_pickle_log.py path/to/logs_20240101_120000.pkl
"""
import sys
import pickle
import argparse
import numpy as np
import matplotlib.pyplot as plt


def load_log(pkl_path):
    with open(pkl_path, 'rb') as f:
        log_dict = pickle.load(f)
    return log_dict


def plot_log(log_dict):
    t = np.array(log_dict['time'])
    t = (t - t[0]) / 1000  # ms -> s, relative to start

    pos = np.array(log_dict['pos'])
    vel = np.array(log_dict['vel'])
    iq_set = np.array(log_dict['iq_set'])
    iq_act = np.array(log_dict['iq_act'])
    tau_set = np.array(log_dict['tau_set'])
    tau_act = np.array(log_dict['tau_act'])
    cmd = np.array(log_dict['cmd'])
    pend_pos = np.array(log_dict['pend_pos'])
    pend_vel = np.array(log_dict['pend_vel'])

    has_pendulum = np.any(pend_pos != 0) or np.any(pend_vel != 0)

    """ POSITION, VELOCITY, AND COMMAND """
    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

    axes[0].plot(t, pos, 'b-', linewidth=1.5, label='Position')
    axes[0].plot(t, cmd, 'k--', linewidth=1.5, label='Position CMD')
    axes[0].set_ylabel('Position (rad)', fontsize=12)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='upper right')
    axes[0].set_title('Logs', fontsize=14, fontweight='bold')

    axes[1].plot(t, vel, 'g-', linewidth=1.5, label='Velocity')
    axes[1].set_ylabel('Velocity (rad/s)', fontsize=12)
    axes[1].set_xlabel('Time (s)', fontsize=12)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='upper right')

    """ TORQUE AND CURRENT """
    fig2, axes2 = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

    axes2[0].plot(t, iq_set, 'p-', linewidth=1.5, label='iq_set')
    axes2[0].plot(t, iq_act, 'r--', linewidth=1.5, label='iq_act')
    axes2[0].set_ylabel('Current (A)', fontsize=12)
    axes2[0].grid(True, alpha=0.3)
    axes2[0].legend(loc='upper right')
    axes2[0].set_title('Logs', fontsize=14, fontweight='bold')

    axes2[1].plot(t, tau_set, 'g-', linewidth=1.5, label='tau_set')
    axes2[1].plot(t, tau_act, 'b--', linewidth=1.5, label='tau_act')
    axes2[1].set_ylabel('Torque (Nm)', fontsize=12)
    axes2[1].set_xlabel('Time (s)', fontsize=12)
    axes2[1].grid(True, alpha=0.3)
    axes2[1].legend(loc='upper right')

    """ PENDULUM (only if data looks populated) """
    if has_pendulum:
        fig3, axes3 = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

        axes3[0].plot(t, pend_pos, 'p-', linewidth=1.5, label='Pendulum position')
        axes3[0].set_ylabel('Position (rad)', fontsize=12)
        axes3[0].grid(True, alpha=0.3)
        axes3[0].legend(loc='upper right')
        axes3[0].set_title('Logs', fontsize=14, fontweight='bold')

        axes3[1].plot(t, pend_vel, 'g-', linewidth=1.5, label='Pendulum velocity')
        axes3[1].set_ylabel('Velocity (rad/s)', fontsize=12)
        axes3[1].set_xlabel('Time (s)', fontsize=12)
        axes3[1].grid(True, alpha=0.3)
        axes3[1].legend(loc='upper right')

    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot a telemetry pickle log.")
    parser.add_argument('pkl_path', help="Path to the .pkl log file")
    args = parser.parse_args()

    log_dict = load_log(args.pkl_path)
    plot_log(log_dict)


if __name__ == '__main__':
    main()