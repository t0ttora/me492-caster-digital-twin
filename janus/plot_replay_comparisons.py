"""Plot axle trajectories for the verified Python–Simscape torque replays."""
import argparse
import json
from pathlib import Path

from matplotlib import pyplot as plt
import numpy as np


def render(folders, output):
    figure, axes = plt.subplots(2, len(folders), figsize=(4.5 * len(folders), 7),
                               squeeze=False, layout="constrained")
    for column, path in enumerate(folders):
        metrics = json.loads((path / "comparison.json").read_text())
        if not metrics["equal_applied_torque"]:
            raise ValueError(f"Applied torque equivalence failed: {path}")
        data = np.genfromtxt(path / "pose-overlay.csv", names=True, delimiter=",")
        xy, yaw = axes[:, column]
        for engine, name, color in (("python", "Python V2.3", "#a2482d"),
                                    ("native", "Simscape native", "#126e82")):
            xy.plot(data[f"{engine}_x_m"], data[f"{engine}_y_m"], label=name, color=color)
            yaw.plot(data["time_s"], np.rad2deg(np.unwrap(data[f"{engine}_yaw_rad"])),
                     label=name, color=color)
        case = metrics["python_run_id"]
        error = metrics["pose_error"]["position_rmse_m"]
        xy.set(title=f"{case}\nPosition RMSE {error:.3f} m", xlabel="World x (m)", ylabel="World y (m)")
        xy.set_aspect("equal", adjustable="datalim")
        yaw.set(xlabel="Time (s)", ylabel="Unwrapped yaw (deg)")
        for axis in (xy, yaw):
            axis.grid(alpha=0.22)
        if column == 0:
            xy.legend(loc="best")
    figure.suptitle("Equal applied axle torque, different plant responses\n"
                    "Open-loop simulation replays with matched horizontal initial state")
    figure.savefig(output, dpi=150)
    plt.close(figure)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_png", type=Path)
    parser.add_argument("comparisons", type=Path, nargs="+")
    args = parser.parse_args()
    render(args.comparisons, args.output_png)
