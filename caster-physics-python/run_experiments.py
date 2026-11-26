"""BAŞLANGIÇ — Önce iki deneyi çalıştır, sonra physics/motion.py'ı oku."""
from pathlib import Path
import argparse
import sys
import time

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/"src"))

from caster_physics.parameters import load_parameters
from caster_physics.simulation.experiments import EXPERIMENTS, experiment_by_name
from caster_physics.simulation.rk4 import simulate
from caster_physics.result_files import save_results, prepare_output_directory


def main():
    parser = argparse.ArgumentParser(description="Sentetik caster fiziği: deney çalıştır, CSV ve açıklamalı görseller üret.")
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--group", choices=["first-lesson", "all"], default="first-lesson")
    selection.add_argument("--case", choices=[experiment.name for experiment in EXPERIMENTS])
    parser.add_argument("--output", type=Path, default=ROOT/"outputs")
    parser.add_argument("--list", action="store_true", help="Deneylerin Türkçe açıklamalarını göster.")
    args = parser.parse_args()
    if args.list:
        for experiment in EXPERIMENTS:
            print(f"{experiment.name}: {experiment.description_tr}")
        return
    output = prepare_output_directory(args.output)
    parameters = load_parameters()
    if args.case:
        experiments = [experiment_by_name(args.case)]
    else:
        experiments = EXPERIMENTS if args.group == "all" else EXPERIMENTS[:2]
    results = {}
    for experiment in experiments:
        started = time.perf_counter()
        print(f"Çalışıyor: {experiment.description_tr}", flush=True)
        results[experiment.name] = simulate(parameters, experiment)
        print(f"  {len(results[experiment.name].records)} kayıt, {time.perf_counter()-started:.1f} s", flush=True)
    save_results(results, output)
    from caster_physics.visualization import create_plots, create_animation
    create_plots(results, output)
    create_animation(results, output/"robot_motion.html")
    print(f"Çıktılar: {output}\nBu sentetik modeldir; gerçek robot kalibrasyonu yapılmadı.")


if __name__ == "__main__":
    main()
