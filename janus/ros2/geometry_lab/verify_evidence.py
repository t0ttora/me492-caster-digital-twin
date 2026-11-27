"""Check evidence integrity and exercise the six case-specific coverage contracts."""
import argparse
import hashlib
import json
from pathlib import Path


def read(path):
    return json.loads(path.read_text())


def inspect_run(root, run_id):
    run = root/run_id
    manifest = read(run/"manifest.json")
    for name, actual in manifest["files_sha256"].items():
        with (run/name).open("rb") as stream:
            digest = hashlib.sha256()
            for chunk in iter(lambda: stream.read(1024*1024), b""):
                digest.update(chunk)
            assert digest.hexdigest() == actual, (run_id, name)
    runtime = (run/"versions.txt").read_text()
    assert "aarch64" in runtime and "jazzy" in runtime and "8.15.0" in runtime, run_id
    assert "janus_geometry_84_" in runtime and "virtiofs" in runtime, run_id
    assert "Valid." in (run/"sdf_check.log").read_text(), run_id
    if (run/"ros_publisher_graph.json").exists():
        graph = read(run/"ros_publisher_graph.json")
        assert all(len(nodes) == 1 for nodes in graph.values()), (run_id, graph)
        assert graph["/cmd_vel"] == ["janus_command_guard"], (run_id, graph["/cmd_vel"])
    if "pinned" in run_id:
        guard = "c8c60df246c55033877d7206bba68a9146ded53beae64e58e4fad47d0ebf3979"
        source = (run/"input-sha256.txt").read_text()
        assert any(line.startswith(guard) and line.endswith("/integration-629bf31/command_guard")
                   for line in source.splitlines()), run_id
    result = read(run/"metrics.json")
    assert result["runtime_result"] == "motion_logged", run_id
    return result, read(run/"fixture.json")


def verified_dwell(value):
    boundaries = value["qualifying_dwell_bounds_s"]
    return boundaries and boundaries[1]-boundaries[0] >= .25-1e-8 and value["held_to_end"]


def verify_m07(root, run_id):
    motion, fixture = inspect_run(root, run_id)
    assert fixture["bounds_m"] == [-.34, .74, -.36, .76]
    assert motion["native_wall_contact_points"] == 0
    assert motion["minimum_model_wall_clearance_m"] > 0.
    assert motion["minimum_upstream_rectangle_wall_clearance_m"] > 0.
    assert verified_dwell(motion["terminal"])
    assert len(motion["reference_segments_distance_turn_duration"]) == 8


def verify_m11(root, run_id):
    motion, fixture = inspect_run(root, run_id)
    assert [surface["sdf_mu"] for surface in fixture["boxes"]] == [.60, .12]
    for wheel in ("left_wheel", "right_wheel", "caster_wheel"):
        transition = motion["both_drive_wheels_surface_transition_s"][wheel]
        assert transition["surface_a"] < transition["surface_b"]
    assert motion["surface_breakdown"]["B"]["braking_drive_slip_peak_mps"] > 0.
    stop = motion["stop"]
    assert stop["dwell_verified_s"]-stop["settle_s"] >= .25-1e-8
    assert stop["brake_duration_s"] > 0. and stop["brake_distance_m"] > 0.


def verify_m12(root, run_id, should_clear):
    motion, fixture = inspect_run(root, run_id)
    assert motion["all_wheels_contacted_finite_threshold"]
    assert motion["all_robot_collision_solids_cleared_threshold"] is should_clear
    assert motion["chassis_vertical_ground_clearance_m"]["finite_threshold"] > 0.
    assert sum(motion["native_chassis_terrain_contact_steps"].values()) == 0
    for wheel in ("left_wheel", "right_wheel", "caster_wheel"):
        contact = motion["direct_solver_step_contact"][wheel]
        assert contact["expected_solver_ticks"] == contact["data_ready_rows"]
        assert contact["missing_rows"] == 0
        assert contact["world_vertical_peak_N"] > 0.
        assert contact["verified_zero_contact_ticks"] > 0.
    assert len(fixture["boxes"]) == 4


def verify_o02(root, run_id, reverse):
    motion, _ = inspect_run(root, run_id)
    assert motion["native_wall_contact_points"] == 0
    assert motion["minimum_upstream_rectangle_wall_clearance_m"] > 0.
    assert verified_dwell(motion["egress_terminal"])
    assert motion["egress_caster_realign_time_s"] is not None
    assert motion["egress_uses_reverse_gear"] is reverse
    assert all(verified_dwell(gate) for gate in motion["segment_ground_truth_gates"].values())
    if reverse:
        assert motion["peak_reverse_speed_mps"] > .05


def verify_crossing(root, run_id, authorized, rendered_station):
    motion, _ = inspect_run(root, run_id)
    check = motion["case_checks"]
    assert check["robot_moving_at_hazard"]
    assert check["stop_dwell_sampled"] and check["geometry_contact_free"]
    assert motion["native_wall_contact_points"] == 0
    assert check["clear_observed"]
    assert motion["request_motion_while_interlocked"] == 0
    assert motion["guard_motion_after_first_zero_and_until_gate"] == 0
    assert motion["physical_hold"]["stop_dwell_until_gate_or_log_end_s"] >= .25
    assert motion["physical_hold"]["peak_speed_mps"] <= .02
    assert motion["physical_hold"]["peak_yaw_rate_radps"] <= .05
    assert check["consent_observed"] is authorized and check["actual_restart"] is authorized
    if authorized:
        assert motion["permission_gate_s"] >= max(motion["clear_s"], motion["authority_s"])
        assert motion["first_resumed_applied_command_s"] >= motion["permission_gate_s"]
    if rendered_station:
        assert motion["rendered_qr_valid_station_observations"] > 0
        assert motion["forward_requests_without_fresh_valid_qr"] == 0
        if "pinned" in run_id:
            windows = motion["qr_fresh_inner_band_dwell_at_forward_starts"]
            assert windows
            assert all(entry["continuous_fresh_inner_band_s"] is not None
                       and entry["continuous_fresh_inner_band_s"] >= .40-1e-8
                       for entry in windows), (run_id, windows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--o03", required=True)
    parser.add_argument("--q14", required=True)
    parser.add_argument("--o03-no-consent", required=True)
    parser.add_argument("--q14-no-consent", required=True)
    args = parser.parse_args()
    root = args.root

    failed, _ = inspect_run(root, "M07-gazebo-original-v2")
    assert failed["native_wall_contact_points"] > 0
    verify_m07(root, "M07-gazebo-narrow-replanned-v1")
    verify_m07(root, "M07-gazebo-narrow-replanned-v2")
    print("M07 original contact failure and two contact-free replans verified")

    verify_m11(root, "M11-gazebo-boundary-v1")
    print("M11 native crossing, longitudinal slip and braking verified")

    verify_m12(root, "M12-gazebo-ramp-step05-instrumented", should_clear=True)
    verify_m12(root, "M12-gazebo-ramp-step20-instrumented", should_clear=False)
    print("M12 5 mm traversal, 20 mm stall and direct contact/wrenches verified")

    verify_o02(root, "O02-gazebo-corridor-v1", reverse=True)
    verify_o02(root, "O02-gazebo-turned-egress-v1", reverse=False)
    print("O02 turn, egress, clearance and caster recovery verified twice")

    verify_crossing(root, args.o03, authorized=True, rendered_station=False)
    verify_crossing(root, args.o03_no_consent, authorized=False, rendered_station=False)
    print("O03 native obstacle, braking, hold and consent edge verified")

    verify_crossing(root, args.q14, authorized=True, rendered_station=True)
    verify_crossing(root, args.q14_no_consent, authorized=False, rendered_station=True)
    print("Q14 rendered station, native obstacle, braking and consent edge verified")


if __name__ == "__main__":
    main()
