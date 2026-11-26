"""Yalnız Python çıktılarını kaydet; parametre ve MATLAB referansına yazma."""
from pathlib import Path
from contextlib import contextmanager
import csv
import hashlib
import json
import os
import platform
import tempfile

from .parameters import PROJECT_ROOT


def prepare_output_directory(directory):
    directory = Path(directory).resolve()
    project = PROJECT_ROOT.resolve()
    # Eski MATLAB/CAD ağaçlarına veya yeni kaynaklara yanlışlıkla çıktı yazma.
    if project.parent in directory.parents and project not in directory.parents:
        raise ValueError("Arche'nin mevcut dosya ağaçlarına çıktı yazılamaz; Python outputs klasörünü seç.")
    protected = [project/name for name in ["reference", "parameters", "src", "tests", "docs", "verification", ".venv"]]
    if directory in [project, project.parent] or any(directory == path or path in directory.parents for path in protected):
        raise ValueError("Çıktı klasörü kaynak veya referans klasörü olamaz.")
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def validate_output_file(path):
    path = Path(path)
    if path.is_symlink() or (path.exists() and not path.is_file()):
        raise ValueError(f"Çıktı bir sembolik bağlantı (symlink) veya özel dosya olamaz: {path}")


@contextmanager
def atomic_output(path):
    """Geçici dosyaya yaz, sonra dosya girdisini atomik değiştir.

    Yaprak dosyaya sonradan symlink konsa bile hedef kaynak açılmaz.
    Hardlink de yerinde yazılmadığı için başka dosyanın içeriği korunur.
    """
    path = Path(path)
    directory = prepare_output_directory(path.parent)
    path = directory/path.name
    validate_output_file(path)
    descriptor, temporary_name = tempfile.mkstemp(prefix=".caster-output-", suffix=path.suffix, dir=directory)
    os.close(descriptor)
    temporary = Path(temporary_name)
    try:
        yield temporary
        validate_output_file(path)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def write_json(path, data):
    with atomic_output(path) as temporary:
        temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")


def save_results(results, directory):
    directory = prepare_output_directory(directory)
    runs = []
    for name, result in results.items():
        path = directory/f"{name}_states.csv"
        with atomic_output(path) as temporary:
            with temporary.open("w", newline="", encoding="utf-8") as file:
                writer = csv.DictWriter(file, fieldnames=result.records[0])
                writer.writeheader()
                writer.writerows(result.records)
        runs.append({"name": name, "rows": len(result.records), "solver_step_s": result.step_s,
                     "csv": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    source_hashes = {str(file.relative_to(PROJECT_ROOT)): hashlib.sha256(file.read_bytes()).hexdigest()
                     for directory_name in ["src", "parameters"]
                     for file in sorted((PROJECT_ROOT/directory_name).rglob("*"))
                     if file.is_file() and file.suffix in {".py", ".json"}}
    write_json(directory/"python_run_manifest.json", {
        "python_version": platform.python_version(), "parameter_status": "provisional_synthetic",
        "real_robot_validated": False, "solver": "fixed_step_RK4_discrete_command_ZOH",
        "runs": runs, "source_sha256": source_hashes,
    })
