"""12 deneyi yeniden çalıştır; MATLAB referansını ve kaynak korumayı denetle."""
from pathlib import Path
import argparse
import sys
import time

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/"src"))

from caster_physics.parameters import load_parameters
from caster_physics.simulation.experiments import EXPERIMENTS
from caster_physics.simulation.rk4 import simulate
from caster_physics.verification import compare_results, physical_run_checks, verify_source_preservation, verify_reference_integrity
from caster_physics.result_files import save_results, write_json, prepare_output_directory


def main():
    parser = argparse.ArgumentParser(description="12 Python deneyi, üç tam MATLAB CSV'si ve 12 MATLAB son durumu ile karşılaştırılır.")
    parser.add_argument("--output", type=Path, default=ROOT/"outputs/verification")
    args = parser.parse_args()
    output = prepare_output_directory(args.output)
    preservation = verify_source_preservation()
    integrity = verify_reference_integrity()
    if not preservation["all_passed"] or not integrity["all_passed"]:
        write_json(output/"verification_report.json", {"all_passed": False, "matlab_source_preservation": preservation, "reference_integrity": integrity})
        raise SystemExit("MATLAB kaynak/referans koruma denetimi başarısız; karşılaştırma durduruldu.")
    parameters, results = load_parameters(), {}
    for experiment in EXPERIMENTS:
        started = time.perf_counter()
        print(f"[{len(results)+1}/12] {experiment.name}", flush=True)
        results[experiment.name] = simulate(parameters, experiment)
        print(f"  {time.perf_counter()-started:.1f} s", flush=True)
    comparison = compare_results(results)
    physics = physical_run_checks(results)
    save_results(results, output)
    from caster_physics.visualization import create_plots, create_animation
    create_plots(results, output)
    create_animation(results, output/"robot_motion.html")
    # Kaynak koruma tüm CSV/grafik/HTML yazımları tamamlandıktan sonra ölçülür.
    preservation = verify_source_preservation()
    passed = (comparison["all_passed"] and comparison["full_series_count"] == 3 and comparison["endpoint_count"] == 12
              and physics["all_passed"] and preservation["all_passed"])
    report = {"all_passed": passed, "matlab_comparison": comparison, "physical_consistency": physics,
              "matlab_source_preservation": preservation, "reference_integrity": integrity}
    write_json(output/"verification_report.json", report)
    print(f"Tam CSV: {comparison['full_series_count']}/3; son durum: {comparison['endpoint_count']}/12; fizik: {physics['passed']}/{physics['total']}")
    print(f"MATLAB kaynak koruma: {preservation['checked_files']} dosya; rapor: {output/'verification_report.json'}")
    if not passed:
        raise SystemExit("Doğrulama başarısız. Rapor eşikleri otomatik gevşetmez.")
    print("Tüm karşılaştırmalar geçti. Kayıtlı MATLAB verisiyle sayısal eşdeğerlik; robot doğrulaması değildir.")


if __name__ == "__main__":
    main()
