#!/usr/bin/env python3
"""Create the assignment summary plot from the CSV logs produced by the nodes."""

import argparse
import csv
import os


def read_columns(path, names):
    result = {name: [] for name in names}
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            for name in names:
                result[name].append(float(row[name]))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-dir", default="logs")
    parser.add_argument("--output", default="logs/summary_plot.png")
    args = parser.parse_args()
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("matplotlib is required only for offline plot generation") from exc

    noisy_odom = read_columns(
        os.path.join(args.log_dir, "noisy_odom.csv"),
        ["x_true", "y_true", "x_noisy", "y_noisy"],
    )
    ekf = read_columns(
        os.path.join(args.log_dir, "ekf.csv"),
        ["time", "x", "y", "cov_x", "cov_y", "cov_yaw"],
    )
    diagnostics = read_columns(
        os.path.join(args.log_dir, "diagnostics.csv"),
        ["time", "x_ratio", "warning"],
    )
    figure, axes = plt.subplots(2, 1, figsize=(10, 7), constrained_layout=True)
    axes[0].plot(
        noisy_odom["x_true"],
        noisy_odom["y_true"],
        label="Ground truth",
    )
    axes[0].plot(
        noisy_odom["x_noisy"],
        noisy_odom["y_noisy"],
        label="Slipping odometry",
    )
    axes[0].plot(ekf["x"], ekf["y"], label="EKF trajectory")
    axes[0].set(
        xlabel="x [m]",
        ylabel="y [m]",
        title="Ground truth vs. slipping odometry vs. fused estimate",
    )
    axes[0].grid(True)
    axes[0].legend()
    axes[1].plot(diagnostics["time"], diagnostics["x_ratio"], label="Ixx/Iyy")
    axes[1].fill_between(
        diagnostics["time"], 0.0, diagnostics["warning"],
        alpha=0.2, label="degeneracy warning",
    )
    axes[1].set(xlabel="simulation time [s]", ylabel="information ratio")
    axes[1].grid(True)
    axes[1].legend()
    figure.savefig(args.output, dpi=150)
    print(f"diagnostic samples: {len(diagnostics['time'])}")
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
