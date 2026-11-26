"""JANUS CLI for repeated synthetic plant and camera-projection runs."""
import argparse
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import shutil

from janus_artifacts import atomic_json, write_run
from janus_metrics import summarize
from janus_sim import simulate
from scenarios import BLOCKED, FIXTURES, P0
from twin import verify_v1


def execute(fixture, output, baseline):
    run = simulate(fixture)
    metrics = summarize(fixture, run)
    return write_run(output, fixture, run, metrics, baseline)


def index(folder, baseline, state):
    records = []
    statuses = {case: {"status": "blocked", "reason": why} for case, why in BLOCKED.items()}
    for fixture in FIXTURES:
        statuses.setdefault(fixture.case, {"status": "ready", "runs": []})
    for path in sorted(folder.glob("*-python-*/manifest.json")):
        run = json.loads(path.read_text())
        record = {key: run[key] for key in ("run_id","case_id","variant","status","termination_reason","evidence_scopes",
                                           "measurement_checks_passed","completed_time_s","fixture_scope_limit")}
        record.update(manifest=str(path.relative_to(folder)), metrics=str(path.parent.relative_to(folder)/"metrics.json"))
        records.append(record)
        statuses[run["case_id"]]["runs"].append(record["run_id"])
    for case, data in statuses.items():
        observed = [r["status"] for r in records if r["case_id"] == case]
        expected = sum(f.case == case for f in FIXTURES)
        if observed:
            data["status"] = ("failed" if "failed" in observed else
                              "partial" if "partial" in observed or len(observed) < expected else "passed")
            data.update(executed_variants=len(observed), configured_variants=expected)
    decoder = folder/"decoder/decode_report.json"
    if decoder.is_file() and json.loads(decoder.read_text())["checks_passed"]:
        statuses["Q09"] = {"status":"partial", "reason":BLOCKED["Q09"],
                           "evidence_scope":"synthetic_image_decode", "evidence":"decoder/decode_report.json"}
    atomic_json(folder/"case_status.json", {"schema_version":"janus-python-1","state":state,
                                           "baseline":baseline,"cases":statuses,"runs":records})


def publish(output, shared, run):
    destination = shared/run["run_id"]
    staging = shared/("."+run["run_id"]+".writing")
    if destination.exists():
        raise FileExistsError(destination)
    staging.mkdir()
    for name in ("commands.csv","executed_velocity.csv","applied_torque.csv","scenario.json","robot_config.json"):
        shutil.copy2(output/run["run_id"]/name, staging/name)
    copy = {k: run[k] for k in ("run_id", "case_id", "variant", "status", "evidence_scopes", "solver",
                                  "initial_axle_pose", "initial_solver_state", "command_roles", "model_sha256",
                                  "command_sha256", "termination_reason", "completed_time_s")}
    copy.update(schema_version="janus-python-1", origin_manifest=str(output/run["run_id"]/"manifest.json"),
                files={p.name: run["files"][p.name] for p in staging.iterdir()})
    atomic_json(staging/"manifest.json", copy)
    staging.rename(destination)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--group", choices=("p0","additional","all"), default="p0")
    parser.add_argument("--case", nargs="+", choices=sorted({f.case for f in FIXTURES}))
    parser.add_argument("--variant", help="Choose one variant for the selected case")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--publish-commands", type=Path)
    parser.add_argument("--jobs", type=int, default=1)
    args = parser.parse_args(argv)
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    selected = [f for f in FIXTURES if (args.case and f.case in args.case) or
                (not args.case and (args.group == "all" or (f.case in P0) == (args.group == "p0")))]
    selected = [f for f in selected if args.variant in (None, f.variant)]
    if not selected:
        parser.error("No case matches the requested variant")
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.publish_commands:
        args.publish_commands.mkdir(parents=True, exist_ok=True)
    baseline = {"git_base":"924f3a2c81ae9350fbec3bc79db0ff868c8ef3d6","v1_preservation":verify_v1()}
    index(output, baseline, "in_progress")
    for fixture in selected:
        if (output/f"{fixture.case}-python-{fixture.variant}").exists():
            parser.error(f"Existing output for {fixture.case}/{fixture.variant}; use a fresh directory")

    def accept(run):
        if args.publish_commands:
            publish(output, args.publish_commands, run)
        index(output, baseline, "in_progress")
        print(f"{run['run_id']} {run['status']} {run['termination_reason']} {run['completed_time_s']:.2f}s", flush=True)

    if args.jobs == 1:
        for fixture in selected:
            accept(execute(fixture, output, baseline))
    else:
        from concurrent.futures import as_completed

        with ProcessPoolExecutor(max_workers=args.jobs) as executor:
            pending = [executor.submit(execute, f, output, baseline) for f in selected]
            for future in as_completed(pending):
                accept(future.result())
    index(output, baseline, "batch_finished")


if __name__ == "__main__":
    main()
