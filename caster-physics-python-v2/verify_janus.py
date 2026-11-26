"""Independently audit JANUS artifacts and replay selected constant-surface runs."""
import argparse
import csv
from dataclasses import fields
import json
import math
from pathlib import Path

import numpy as np

from evaluate_recording import evaluate
from janus_artifacts import atomic_json, csv_file, digest
from twin import State


def read_csv(path, columns=None):
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if columns and reader.fieldnames != list(columns):
            raise ValueError(f"{path} header {reader.fieldnames} differs from {columns}")
        return list(reader)


def table(rows, *fields):
    return np.array([[float(row[k]) for k in fields] for row in rows])


def audit(directory, shared):
    manifest = json.loads((directory/"manifest.json").read_text())
    for relative, sha in manifest["files"].items():
        assert digest(directory/relative)==sha, f"{directory.name}: SHA-256 mismatch for {relative}"
    for relative in ("commands.csv","executed_velocity.csv","applied_torque.csv","robot_config.json","scenario.json"):
        assert digest(shared/directory.name/relative)==manifest["files"][relative]
    copy = json.loads((shared/directory.name/"manifest.json").read_text())
    assert copy["initial_solver_state"] == manifest["initial_solver_state"]
    assert copy["initial_axle_pose"] == manifest["initial_axle_pose"]
    assert copy["solver"] == manifest["solver"]
    dt = manifest["solver"]["command_period_s"]
    velocity_fields = ("time_s","v_mps","yaw_rate_radps")
    torque_fields = ("time_s","tau_left_Nm","tau_right_Nm")
    requested = table(read_csv(directory/"commands.csv", velocity_fields), *velocity_fields)
    limited = table(read_csv(directory/"executed_velocity.csv", velocity_fields), *velocity_fields)
    torque = table(read_csv(directory/"applied_torque.csv", torque_fields), *torque_fields)
    body = read_csv(directory/"results.csv")
    time = torque[:,0]
    assert np.allclose(time, np.arange(len(time))*dt, atol=1e-9, rtol=0.)
    assert np.isfinite([requested,limited,torque]).all()
    assert np.array_equal(requested, table(body,"time_s","requested_v_mps","requested_yaw_rate_radps"))
    assert np.array_equal(limited, table(body,"time_s","limited_v_mps","limited_yaw_rate_radps"))
    assert np.array_equal(torque, table(body,"time_s","left_drive_torque_Nm","right_drive_torque_Nm"))
    assert np.allclose(limited[:,1:], np.clip(requested[:,1:],[-.25,-1.],[.25,1.]))
    assert np.max(abs(torque[:,1:])) <= .5
    pose = table(body,"x_m","y_m","yaw_rad")
    assert np.allclose(pose[0], [manifest["initial_axle_pose"][k] for k in ("x_m","y_m","yaw_rad")])
    metrics = json.loads((directory/"metrics.json").read_text())
    assert metrics["samples"] == len(time)
    assert math.isclose(time[-1],manifest["completed_time_s"])
    for gate in metrics["gates"]:
        # Recompute entire dwell using axle signals and the manifest target.
        assert gate["time_s"]-gate["dwell_start_s"] >= .25-1e-9
        mask = (time >= gate["dwell_start_s"]) & (time <= gate["time_s"])
        assert mask.sum() >= 26
        target = np.asarray(gate["target_pose"])
        yaw = (pose[mask,2]-target[2]+math.pi)%(2*math.pi)-math.pi
        axle = table(body,"axle_v_mps","axle_lateral_mps","body_yaw_rate_radps")[mask]
        assert np.max(np.linalg.norm(pose[mask,:2]-target[:2],axis=1)) <= gate["position_limit_m"]
        assert np.max(abs(yaw)) <= gate["yaw_limit_rad"]
        assert np.max(np.linalg.norm(axle[:,:2],axis=1)) <= .02
        assert np.max(abs(axle[:,2])) <= .05
    if manifest["case_id"] in ("M05","M06"):
        assert [gate["kind"] for gate in metrics["gates"]] == ["edge","turn"]*4
        yaw = np.array([g["target_pose"][2] for g in metrics["gates"] if g["kind"]=="turn"])
        expected = -math.pi/2 if manifest["case_id"]=="M05" else math.pi/2
        assert np.allclose(np.diff(np.r_[0.,yaw]),expected)
    if manifest["case_id"].startswith("Q"):
        epoch = None
        last_phase = None
        for row in body:
            t, v, w = (float(row[k]) for k in ("time_s","requested_v_mps","requested_yaw_rate_radps"))
            error = float(row["qr_error"]) if row["qr_error"] else None
            valid = row["qr_valid"]=="True"
            if valid:
                capture, arrival = float(row["qr_capture_time_s"]), float(row["qr_arrival_time_s"])
                assert 0. <= capture <= arrival <= t+1e-9 and t-capture <= .20+1e-9
            if not valid or abs(error) > .02:
                epoch = None
            elif epoch is None:
                epoch = t
            if v > 0.:
                assert valid and abs(error) <= .04
                # At the transition to advance, an uninterrupted inner-band dwell is required.
                if last_phase != "advance":
                    assert epoch is not None and t-epoch >= .40-1e-9
            if valid and row["phase"] in ("center","advance","recenter"):
                assert error*w <= 1e-12
            last_phase = row["phase"]
        events = read_csv(directory/"events.csv")
        phases = {e["event"]:float(e["time_s"]) for e in events}
        if body[-1]["charge_confirmed"] == "True":
            assert phases["aligned"] <= phases["contact"] <= phases["virtual_bms_frame"] <= phases["success"]
            mask = time >= phases["aligned"]
            assert np.all(requested[mask,1:]==0.)
            speeds = table(body,"axle_v_mps","axle_lateral_mps","body_yaw_rate_radps")
            assert np.max(np.linalg.norm(speeds[mask,:2],axis=1)) <= .02
            assert np.max(abs(speeds[mask,2])) <= .05
    return {"run_id":directory.name,"samples":len(time),"gates_recomputed":len(metrics["gates"]),
            "status":manifest["status"],"failed_acceptance_checks":[key for key,value in metrics["checks"].items() if not value]}


def recording_replay(directory, target):
    manifest = json.loads((directory/"manifest.json").read_text())
    config = json.loads((directory/"robot_config.json").read_text())
    if config["surface_after_boundary"]:
        raise ValueError("The legacy recording evaluator expects a constant parameter set")
    target.mkdir()
    metadata = {
        "run_id":directory.name,"session_id":"janus-python-internal-replay","split":"development","data_kind":"synthetic",
        "surface_id":"constant-fixture","robot_revision":"V2.3","pose_source":"generated_V2.3_plant_truth",
        "pose_frame":"drive_axle_midpoint_world_SI","torque_units":"axle_Nm","torque_source":"applied ZOH output",
        "parameter_provenance":"exact effective simulation parameters from robot_config.json",
        "initial_state":[manifest["initial_solver_state"][f.name] for f in fields(State)],
        "parameter_overrides":config["parameters"],
    }
    rows = read_csv(directory/"results.csv")
    source_fields = ("time_s","left_drive_torque_Nm","right_drive_torque_Nm","left_wheel_rate_radps",
                     "right_wheel_rate_radps","x_m","y_m","yaw_rad")
    csv_file(target/"recording.csv", source_fields[:5]+("reference_x_m","reference_y_m","reference_yaw_rad"),
             ([row[k] for k in source_fields] for row in rows))
    atomic_json(target/"metadata.json",metadata)
    report, _ = evaluate(target/"recording.csv",target/"metadata.json")
    atomic_json(target/"evaluation.json",report)
    error = report["m2_torque_replay_vs_reference"]
    assert error["position_max_m"] < 1e-10 and error["yaw_rmse_rad"] < 1e-10
    print(f"Replay {directory.name} {error}",flush=True)
    return {"run_id":directory.name,**error}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs",type=Path,required=True)
    parser.add_argument("--shared-commands",type=Path,required=True)
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--replay",nargs="*",default=[])
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=False)
    audited = []
    for path in sorted(args.runs.glob("*-python-*/manifest.json")):
        audited.append(audit(path.parent,args.shared_commands))
    assert audited, "No runs found"
    result = {"audit":audited,"same_plant_torque_replay":[recording_replay(args.runs/run,args.output_dir/run)
                                                        for run in args.replay]}
    atomic_json(args.output_dir/"verification.json",result)
    print(f"Audited {len(audited)} complete runs",flush=True)


if __name__=="__main__":
    main()
