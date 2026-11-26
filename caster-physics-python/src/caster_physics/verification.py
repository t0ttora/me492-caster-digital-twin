"""MATLAB ile sayısal eşdeğerlik ve kaynak koruma denetimi.

Üç tam zaman serisi ile 12 son durum farklı kanıt düzeyleridir.
Karşılaştırma eşiği: |Python-MATLAB| <= atol + rtol*|MATLAB|.
"""
from datetime import datetime, timezone
from pathlib import Path
import csv
import hashlib
import json

import numpy as np

from .parameters import PROJECT_ROOT


def read_csv(path):
    with Path(path).open(newline="", encoding="utf-8-sig") as file:
        return [{key: float(value) for key, value in row.items()} for row in csv.DictReader(file)]


def error_summary(actual, expected, atol, rtol):
    actual, expected = np.asarray(actual, dtype=float), np.asarray(expected, dtype=float)
    if actual.shape != expected.shape or not np.all(np.isfinite(actual)) or not np.all(np.isfinite(expected)):
        return {"passed": False, "reason": "Boyut veya sonlu değer hatası"}
    absolute = np.abs(actual-expected)
    scaled = absolute/(atol+rtol*np.abs(expected))
    return {"passed": bool(np.all(scaled <= 1)), "max_absolute_error": float(np.max(absolute)),
            "max_scaled_error": float(np.max(scaled))}


def compare_results(results, reference_directory=None, atol=1e-7, rtol=1e-6):
    reference = Path(reference_directory) if reference_directory else PROJECT_ROOT/"reference/matlab_m2_v1"
    if atol <= 0 or rtol < 0 or not np.all(np.isfinite([atol, rtol])):
        raise ValueError("Karşılaştırma toleransları sonlu, atol>0 ve rtol>=0 olmalı.")
    manifest = json.loads((reference/"run_manifest.json").read_text())
    endpoints, full_series = {}, {}
    for native_run in manifest["runs"]:
        name = native_run["scenario_id"]
        if name not in results:
            continue
        rows = results[name].records
        keys = ["x_m", "y_m", "yaw_rad", "caster_angle_body_rad"]
        expected = [*native_run["final_pose"], native_run["final_caster_angle_body_rad"]]
        if not rows or any(key not in rows[-1] for key in keys):
            endpoints[name] = {"passed": False, "reason": "Son durum sütunları eksik"}
        else:
            endpoints[name] = error_summary([rows[-1][key] for key in keys], expected, atol, rtol)
        csv_path = reference/"results"/f"{name}_states.csv"
        if not csv_path.is_file():
            continue
        native_rows = read_csv(csv_path)
        names = list(native_rows[0])
        if len(rows) != len(native_rows) or any(set(row) != set(names) for row in rows):
            full_series[name] = {"passed": False, "reason": "CSV satır sayısı veya sütunları farklı", "columns": {}}
            continue
        columns = {key: error_summary([row[key] for row in rows], [row[key] for row in native_rows], atol, rtol)
                   for key in names}
        command_keys = ["time_s", "left_drive_torque_Nm", "right_drive_torque_Nm"]
        commands_passed = all(error_summary([row[key] for row in rows], [row[key] for row in native_rows], 1e-12, 0)["passed"] for key in command_keys)
        full_series[name] = {"passed": all(column["passed"] for column in columns.values()) and commands_passed,
                             "rows": len(rows), "columns": columns, "time_and_command_passed": commands_passed}
    all_comparisons = [*endpoints.values(), *full_series.values()]
    return {"checked_at_utc": datetime.now(timezone.utc).isoformat(),
            "absolute_tolerance": atol, "relative_tolerance": rtol,
            "all_passed": bool(all_comparisons) and all(item["passed"] for item in all_comparisons),
            "full_series_count": len(full_series), "endpoint_count": len(endpoints),
            "full_series": full_series, "endpoints": endpoints,
            "scope": "Recorded synthetic MATLAB model parity; not a fresh MATLAB run or real-robot validation."}


def verify_source_preservation():
    baseline = json.loads((PROJECT_ROOT/"verification/matlab_sources_before.json").read_text())
    changed, missing = [], []
    for name, expected_hash in baseline["files"].items():
        file = PROJECT_ROOT.parent/name
        if not file.is_file():
            missing.append(name)
        elif hashlib.sha256(file.read_bytes()).hexdigest() != expected_hash:
            changed.append(name)
    return {"all_passed": not changed and not missing, "checked_files": len(baseline["files"]),
            "changed": changed, "missing": missing, "algorithm": "SHA-256"}


def verify_reference_integrity():
    reference = PROJECT_ROOT/"reference/matlab_m2_v1"
    manifest = json.loads((reference/"file_manifest_sha256.json").read_text())
    expected_by_path = {entry["path"]: entry["sha256"] for entry in manifest["files"]}
    changed = []
    copied = [path for path in reference.rglob("*") if path.is_file() and path.name != "file_manifest_sha256.json"]
    for path in copied:
        name = path.relative_to(reference).as_posix()
        if name not in expected_by_path or hashlib.sha256(path.read_bytes()).hexdigest() != expected_by_path[name]:
            changed.append(name)
    return {"all_passed": not changed and len(copied) == 7, "checked_files": len(copied), "changed": changed}


def physical_run_checks(results):
    """Tüm deneylerde enerji/temas; uygun çiftlerde ayna ve adım yakınsaması."""
    checks = {}
    for name, result in results.items():
        d, records = result.parameters.dynamics, result.records
        finite = np.all(np.isfinite(np.array([list(row.values()) for row in records])))
        checks[f"{name}:finite"] = bool(finite)
        checks[f"{name}:energy_balance"] = bool(np.max(np.abs(result.column("energy_balance_error_J"))) < 1e-5)
        checks[f"{name}:passive_contact"] = bool(np.min(result.column("dissipation_power_W")) >= -1e-10)
        normal = np.column_stack([result.column(f"normal_{wheel}_N") for wheel in ["left", "right", "caster"]])
        checks[f"{name}:positive_support"] = bool(normal.min() > 5)
        force_checks = []
        for wheel in ["left", "right", "caster"]:
            force = np.hypot(result.column(f"{wheel}_fx_world_N"), result.column(f"{wheel}_fy_world_N"))
            coefficient = d.caster_friction_coefficient if wheel == "caster" else d.drive_friction_coefficient
            force_checks.append(bool(np.all(force <= coefficient*result.column(f"normal_{wheel}_N")+1e-8)))
        checks[f"{name}:friction_limits"] = all(force_checks)
        if d.caster_bore_effective_radius_m > 0:
            capacity = np.sqrt(result.column("caster_fx_world_N")**2+result.column("caster_fy_world_N")**2
                               +(result.column("bore_ground_torque_Nm")/d.caster_bore_effective_radius_m)**2)
            checks[f"{name}:shared_caster_budget"] = bool(np.all(capacity <= d.caster_friction_coefficient*normal[:, 2]+1e-8))
    if "passive_coast" in results:
        energy = results["passive_coast"].column("kinetic_energy_J")
        checks["passive_coast:energy_decreases"] = bool(np.diff(energy).max() < 1e-9 and energy[-1] < .1*energy[0])
    if "conservative_free" in results:
        result = results["conservative_free"]
        for key in ["kinetic_energy_J", "linear_momentum_x_kgmps", "linear_momentum_y_kgmps", "angular_momentum_z_kgm2ps"]:
            values = result.column(key)
            checks[f"conservative_free:conserved_{key}"] = bool(np.max(np.abs(values-values[0])) < 1e-8)
    if {"misaligned_forward", "mirror_misaligned"} <= results.keys():
        for key in ["y_m", "yaw_rad"]:
            checks[f"mirror:{key}"] = bool(np.max(np.abs(results["misaligned_forward"].column(key)+results["mirror_misaligned"].column(key))) < 1e-8)
    if {"misaligned_forward", "misaligned_fine", "misaligned_finer"} <= results.keys():
        coarse, fine, finer = [results[key].column("yaw_rad") for key in ["misaligned_forward", "misaligned_fine", "misaligned_finer"]]
        error1, error2 = np.max(np.abs(coarse-fine)), np.max(np.abs(fine-finer))
        checks["step_refinement:yaw"] = bool(error1 < 1e-4 and error2 <= max(.3*error1, 1e-10))
    return {"all_passed": all(checks.values()), "passed": sum(checks.values()), "total": len(checks), "checks": checks}
