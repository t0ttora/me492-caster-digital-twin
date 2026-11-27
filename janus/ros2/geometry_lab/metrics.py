"""Reduce recorded simulator streams without using controller assertions as evidence."""
import argparse
import collections
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import struct

import numpy as np
from gz.msgs10.contacts_pb2 import Contacts

from motion import STILL_DWELL, STILL_SPEED, STILL_YAW, advance, rounded_segments


WHEELS = ("left_wheel", "right_wheel", "caster_wheel")
BASE = "base_link"
SDF_SLEEP_GAP = .05


def sha(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def load_trace(path):
    channels = collections.defaultdict(list)
    with path.open() as stream:
        for index, line in enumerate(stream):
            row = json.loads(line)
            row["trace_index"] = index
            channels[row["source"]].append(row)
    return channels


def arrays(rows, names):
    return np.asarray([[row[name] for name in names] for row in rows], dtype=float)


def stamps(rows, field="time_s"):
    return np.asarray([row[field] for row in rows])


def rotation(quaternion):
    x, y, z, w = quaternion
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def collision_extents(xyz, rot, caster):
    """Signed extrema [xmin, xmax, ymin, ymax] from SDF collision solids."""
    chassis = xyz + rot @ np.array([0., 0., .065])
    extent = np.abs(rot) @ np.array([.25, .16, .04])
    low, high = chassis-extent, chassis+extent
    swivel_cos, swivel_sin = math.cos(caster), math.sin(caster)
    for local_center, local_axis, radius, half_width in [
            ([0., .19, 0.], [0., 1., 0.], .075, .015),
            ([0., -.19, 0.], [0., 1., 0.], .075, .015),
            ([-.18-.05*swivel_cos, -.05*swivel_sin, -.035],
             [-swivel_sin, swivel_cos, 0.], .04, .01),
    ]:
        center = xyz + rot @ local_center
        axis = np.clip(rot @ local_axis, -1., 1.)
        support = radius * np.sqrt(1.-axis**2) + half_width * np.abs(axis)
        low = np.minimum(low, center-support)
        high = np.maximum(high, center+support)
    return np.array([low[0], high[0], low[1], high[1]])


def nearest(reference, query):
    upper = np.searchsorted(reference, query).clip(0, len(reference)-1)
    lower = (upper-1).clip(0)
    return np.where(abs(reference[upper]-query) < abs(reference[lower]-query), upper, lower)


def dwell(time, valid, minimum=STILL_DWELL, gap=SDF_SLEEP_GAP):
    start = None
    for i, acceptable in enumerate(valid):
        if i and time[i]-time[i-1] > gap:
            start = None
        if not acceptable:
            start = None
        elif start is None:
            start = i
        elif time[i]-time[start] >= minimum-1e-7:
            return start, i
    return None


def stop(time, xy, speed, yaw_rate, trigger, before=math.inf):
    if trigger is None:
        return None
    candidates = (time >= trigger-1e-7) & (time < before)
    result = dwell(time, candidates & (speed <= STILL_SPEED) & (np.abs(yaw_rate) <= STILL_YAW))
    if result is None:
        return None
    first, confirmed = result
    trigger_xy = [np.interp(trigger, time, xy[:, i]) for i in range(2)]
    if trigger > time[first]:
        trigger_xy = xy[first]
    span = np.vstack((trigger_xy, xy[np.searchsorted(time, trigger-1e-7):first+1]))
    return {
        "trigger_s": trigger, "settle_s": float(time[first]), "dwell_verified_s": float(time[confirmed]),
        "brake_duration_s": max(0., float(time[first]-trigger)),
        "brake_distance_m": float(np.linalg.norm(np.diff(span, axis=0), axis=1).sum()),
        "trigger_axle_speed_mps": float(np.interp(trigger, time, speed)),
    }


def frames(path):
    with gzip.open(path, "rb") as stream:
        while size := stream.read(4):
            count, = struct.unpack("<I", size)
            value = stream.read(count)
            if len(value) != count:
                raise ValueError(f"truncated contact frame in {path.name}")
            yield Contacts.FromString(value)


def time_ms(stamp):
    return stamp.sec*1000 + round(stamp.nsec/1_000_000)


def contact_records(run, names, start, end):
    """Collision-pair samples and actual solver wrench per raw simulation tick."""
    stats, supports = {}, {}
    for link in names:
        path = run/f"contact-{link}.pb.gz"
        counts = collections.Counter()
        samples = collections.defaultdict(set)
        steps = {}
        for message in frames(path):
            tick = time_ms(message.header.stamp)
            t = tick/1000.
            if not start-1e-7 <= t <= end+1e-7:
                continue
            for contact in message.contact:
                first, second = contact.collision1.name, contact.collision2.name
                other = second if f"arche::{link}::" in first else first
                label = other.split("::")[0]
                if not label:
                    label = f"entity_id_{contact.collision2.id}"
                counts[label] += len(contact.position)
                samples[label].add(tick)
                # Both bodies are action/reaction. Select the wrench on the measured robot link.
                for wrench, normal in zip(contact.wrench, contact.normal):
                    body = wrench.body_1_wrench if other == second else wrench.body_2_wrench
                    force = body.force
                    vertical, normal_sum = steps.setdefault(tick, [0., 0.])
                    steps[tick][:] = (vertical+force.z,
                                      normal_sum+abs(force.x*normal.x+force.y*normal.y+force.z*normal.z))
        stats[link] = {
            "points_by_nonrobot_entity": dict(counts),
            "steps_by_nonrobot_entity": {key: len(ticks) for key, ticks in samples.items()},
            "all_detection_steps": len(set().union(*samples.values())),
            "contact_topic": names[link],
        }
        supports[link] = (steps, samples)
    return stats, supports


def all_rms(values):
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else None


def terminal(time, xyz, yaw, yaw_rate, speed, target, after):
    position = np.linalg.norm(xyz[:, :2]-target[:2], axis=1)
    heading = np.abs(np.arctan2(np.sin(yaw-target[2]), np.cos(yaw-target[2])))
    valid = ((position <= .02) & (heading <= .05) & (speed <= STILL_SPEED)
             & (np.abs(yaw_rate) <= STILL_YAW))
    gate = dwell(time, valid & (time >= after-1e-7))
    return {
        "target_xy_yaw": list(target), "final_position_error_m": float(position[-1]),
        "final_heading_error_rad": float(heading[-1]),
        "qualifying_dwell_bounds_s": [float(time[i]) for i in gate] if gate else None,
        "held_to_end": bool(np.all(valid[gate[1]:])) if gate else False,
    }


def clip_face(vertices, axis, bound, sign):
    if not vertices:
        return vertices
    output = []
    for start, end in zip(vertices, vertices[1:]+vertices[:1]):
        left, right = sign*(start[axis]-bound), sign*(end[axis]-bound)
        if left >= 0.:
            output.append(start)
        if (left < 0.) != (right < 0.):
            output.append(start + left/(left-right)*(end-start))
    return output


def vertical_clearance(xyz, rot, fixtures):
    corners = {(a, b, c): xyz+rot @ np.array([a*.25, b*.16, .065+c*.04])
               for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)}
    faces = []
    for axis in range(3):
        for level in (-1, 1):
            keys = list(corners)
            keys = [key for key in keys if key[axis] == level]
            faces.append([corners[keys[i]] for i in (0, 1, 3, 2)])
    for part in fixtures:
        x, y, z = part["center_m"]
        sx, sy, sz = part["size_m"]
        theta = part["pitch_rad"]
        entry = math.cos(theta)*sx/2
        top_x = x + math.sin(theta)*sz/2
        top_z = z + math.cos(theta)*sz/2
        resolutions = []
        for face in faces:
            for axis, bound, sign in [(0, top_x-entry, 1), (0, top_x+entry, -1),
                                      (1, y-sy/2, 1), (1, y+sy/2, -1)]:
                face = clip_face(face, axis, bound, sign)
            for px, _, pz in face:
                resolutions.append(pz-(top_z-(px-top_x)*math.tan(theta)))
        yield min(resolutions) if resolutions else None


def summarise(run):
    fixture = json.loads((run/"fixture.json").read_text())
    termination = json.loads((run/"termination.json").read_text())
    channels = load_trace(run/"trace.jsonl")
    origin = termination["origin_sim_time_s"]
    if origin is None:
        return {"case": fixture["case"], "variant": fixture["variant"],
                "runtime_result": "stream_setup_failed", "termination": termination,
                "raw_stream_counts": {key: len(value) for key, value in channels.items()}}
    end = termination["end_sim_time_s"]
    case = fixture["case"]
    gt = [r for r in channels["ground_truth"] if origin-1e-7 <= r["time_s"] <= end]
    if len(gt) < 2:
        return {"case": case, "variant": fixture["variant"], "runtime_result": "insufficient_live_motion_data",
                "termination": termination,
                "raw_stream_counts": {key: len(value) for key, value in channels.items()}}
    time = stamps(gt)-origin
    events = {row["event"]: row["read_time_s"]-origin for row in channels["event"]
              if row["read_time_s"] is not None and row["read_time_s"] >= origin}
    joints = channels["joint_states"]
    js_t = stamps(joints)-origin
    j_idx = nearest(js_t, time)
    abs_joint_alignment = np.abs(js_t[j_idx]-time)
    xyz = arrays(gt, ("x_m", "y_m", "z_m"))
    v_body = arrays(gt, ("v_mps", "lateral_mps", "vertical_mps"))
    speed = np.linalg.norm(v_body[:, :2], axis=1)
    yaw = arrays(gt, ("yaw_rad",))[:, 0]
    rotations = [rotation(row["quat_xyzw"]) for row in gt]
    yaw_rate = arrays(gt, ("yaw_rate_radps",))[:, 0]
    caster = arrays(joints, ("caster_swivel_angle_rad",))[:, 0][j_idx]
    caster_rate = arrays(joints, ("caster_swivel_radps",))[:, 0][j_idx]
    wheel_rates = arrays(joints, ("left_wheel_radps", "right_wheel_radps"))[j_idx]
    slip = .075*wheel_rates - np.column_stack((v_body[:, 0]-.19*yaw_rate, v_body[:, 0]+.19*yaw_rate))
    contacts, support = contact_records(run, fixture["contacts"], origin, end)
    contact_points = collections.Counter()
    for entry in contacts.values():
        contact_points.update(entry["points_by_nonrobot_entity"])
    out = {
        "scope": fixture["scope"], "runtime_result": "motion_logged",
        "case": case, "variant": fixture["variant"], "events_s": events,
        "termination": termination, "measurement_frame": "Gazebo world (map), axle midpoint",
        "ground_truth_samples": len(gt), "ground_truth_largest_gap_s": float(np.max(np.diff(time))),
        "maximum_joint_stamp_offset_s": float(abs_joint_alignment.max()),
        "command_authorities": {"request": "geometry recorder", "guard_output": "integration command_guard"},
        "streams_count_including_setup": {key: len(value) for key, value in channels.items()},
        "body_x_range_m": [float(np.min(xyz[:, 0])), float(np.max(xyz[:, 0]))],
        "total_axle_path_m": float(np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1).sum()),
        "peak_planar_speed_mps": float(speed.max()),
        "caster_peak_abs_swivel_radps": float(np.max(np.abs(caster_rate))),
        "final_caster_angle_rad": float(np.arctan2(np.sin(caster[-1]), np.cos(caster[-1]))),
        "native_contact_by_robot_collision": contacts,
        "native_wall_contact_points": sum(v for k, v in contact_points.items() if k.startswith("wall_")),
        "applied_axle_torque_Nm": None,
    }
    derived = {"time_s": time, "drive_left_slip_mps": slip[:, 0], "drive_right_slip_mps": slip[:, 1]}
    if fixture["bounds_m"]:
        ext = np.asarray([collision_extents(po, rot, angle) for po, rot, angle in zip(xyz, rotations, caster)])
        xmin, xmax, ymin, ymax = fixture["bounds_m"]
        limits = np.array([xmin, xmax, ymin, ymax])
        signs = np.array([1., -1., 1., -1.])
        exact_wall = np.min((ext-limits)*signs, axis=1)
        hx, hy = .25, .205
        abs_c, abs_s = np.abs(np.cos(yaw)), np.abs(np.sin(yaw))
        ex, ey = hx*abs_c+hy*abs_s, hx*abs_s+hy*abs_c
        proxy = np.min(np.column_stack((xyz[:, 0]-ex-xmin, xmax-xyz[:, 0]-ex,
                                        xyz[:, 1]-ey-ymin, ymax-xyz[:, 1]-ey)), axis=1)
        derived.update(model_wall_clearance_m=exact_wall, upstream_rectangle_wall_clearance_m=proxy)
        i = int(np.argmin(exact_wall))
        out.update(minimum_model_wall_clearance_m=float(exact_wall[i]),
                   minimum_upstream_rectangle_wall_clearance_m=float(np.min(proxy)),
                   minimum_model_wall_clearance_at_s=float(time[i]),
                   clearance_method="projected chassis box and 3 wheel cylinders at each 50 Hz Gazebo pose")
    if case == "M07":
        start = tuple(fixture["spawn_pose_xyz_rpy"][i] for i in (0, 1, 5))
        pieces = []
        segment_list = rounded_segments(fixture["variant"])
        for distance, turn, _ in segment_list:
            pieces.extend(advance(start, distance*u, turn*u)[:2] for u in np.linspace(0., 1., 251))
            start = advance(start, distance, turn)
        path = np.asarray(pieces)
        error = np.sqrt(np.min(np.sum((xyz[:, None, :2]-path[None, :, :])**2, axis=2), axis=1))
        duration = sum(seg[2] for seg in segment_list)
        derived.update(cross_track_m=error)
        out.update(cross_track_rmse_m=all_rms(error), cross_track_motion_only_rmse_m=all_rms(error[time < duration]),
                   reference_segments_distance_turn_duration=segment_list,
                   reference_complete_time_s=duration,
                   terminal=terminal(time, xyz, yaw, yaw_rate, speed, start, duration),
                   yaw_net_rad=float(np.unwrap(yaw)[-1]-np.unwrap(yaw)[0]))
    if case == "M11":
        out["stop"] = stop(time, xyz[:, :2], speed, yaw_rate, events.get("brake_step"))
        braking = ((time >= events["brake_step"]) & (time <= out["stop"]["dwell_verified_s"])
                   if out["stop"] else np.zeros(len(time), dtype=bool))
        sub = {}
        for side, mask in [("A", xyz[:, 0] < .30), ("B", xyz[:, 0] >= .30)]:
            sub[side] = {
                "samples_by_axle_location": int(mask.sum()),
                "drive_longitudinal_slip_rms_mps": all_rms(slip[mask]),
                "moving_samples": int(np.count_nonzero(mask & (time < 4.))),
                "moving_drive_longitudinal_slip_rms_mps": all_rms(slip[mask & (time < 4.)]),
                "peak_body_speed_mps": float(np.max(speed[mask])) if mask.any() else None,
                "braking_drive_slip_peak_mps": float(np.max(np.abs(slip[mask & braking])))
                if np.any(mask & braking) else None,
            }
        out["surface_breakdown"] = sub
        out["both_drive_wheels_surface_transition_s"] = {link: {
            surface: ((min(support[link][1][surface])/1000.)-origin
                      if support[link][1][surface] else None)
            for surface in ("surface_a", "surface_b")}
            for link in ("left_wheel", "right_wheel", "caster_wheel")}
        out["axle_boundary_crossed_s"] = (float(time[np.flatnonzero(xyz[:, 0] >= .30)[0]])
                                          if np.any(xyz[:, 0] >= .30) else None)
        out["terminal"] = terminal(time, xyz, yaw, yaw_rate, speed, (.6, 0., 0.), 4.)
    if case == "M12":
        out.update(stepped_metrics(run, time, xyz, rotations, fixture, support, derived, origin, end))
        out["all_robot_collision_solids_cleared_threshold"] = bool(
            collision_extents(xyz[-1], rotations[-1], caster[-1])[0] > 1.76)
        out["stop"] = stop(time, xyz[:, :2], speed, yaw_rate, events.get("brake_step"))
    if case == "O02":
        turned = fixture["variant"] == "turned-egress"
        phase_name = "forward_exit_after_turn" if turned else "reverse_exit"
        egress = time >= events.get(f"start_{phase_name}", math.inf)
        reverse = egress & (v_body[:, 0] > .05 if turned else v_body[:, 0] < -.05)
        first = np.flatnonzero(reverse)
        caster_target = 0. if turned else math.pi
        error = np.abs(np.arctan2(np.sin(caster-caster_target), np.cos(caster-caster_target)))
        align = dwell(time, reverse & (error <= math.radians(10.)))
        derived.update(egress_caster_error_rad=error)
        out.update(peak_reverse_speed_mps=float(-v_body[:, 0].min()),
                   net_body_x_travel_m=float(xyz[-1, 0]-xyz[0, 0]),
                   egress_uses_reverse_gear=not turned,
                   egress_motion_start_s=float(time[first[0]]) if len(first) else None,
                   egress_caster_10deg_dwell_s=[float(time[i]) for i in align] if align else None,
                   egress_caster_realign_time_s=float(time[align[0]]-time[first[0]]) if align else None,
                   egress_terminal=terminal(time, xyz, yaw, yaw_rate, speed, (0., 0., math.pi if turned else 0.),
                                            events.get(f"start_{phase_name}", math.inf)))
        gates = {}
        for row in channels["event"]:
            event = row["event"]
            if event.startswith("start_") and "target" in row:
                phase = event.removeprefix("start_")
                phase_start = row["read_time_s"]-origin
                phase_finish = events.get(f"gate_{phase}", math.inf)
                select = (time >= phase_start) & (time <= phase_finish+.03)
                if np.count_nonzero(select) < 2:
                    continue
                selected_time = time[select]
                gate = terminal(selected_time, xyz[select], yaw[select], yaw_rate[select],
                                speed[select], tuple(row["target"]), phase_start)
                gate["yaw_net_rad"] = float(np.diff(np.unwrap(yaw[select])[[0, -1]])[0])
                gates[phase] = gate
        nominal_path_error = np.hypot(xyz[:, 1], xyz[:, 0]-xyz[:, 0].clip(0., .35))
        out["cross_track_rmse_m"] = all_rms(nominal_path_error)
        out["segment_ground_truth_gates"] = gates
    if case in {"O03", "Q14"}:
        out.update(interlock_metrics(run, channels, time, events, xyz, ext, speed, yaw_rate,
                                     fixture, origin, contact_points, derived))
        direct_path = run/"native_contact_step.csv"
        if direct_path.exists():
            out["direct_solver_step_contact"] = direct_wheel_contact(
                direct_path, round((origin+time[0])*1000), round((origin+time[-1])*1000),
                support, origin)
    with (run/"derived.csv").open("w", newline="") as stream:
        rows = csv.writer(stream)
        rows.writerow(derived)
        rows.writerows(zip(*derived.values()))
    return out


def contiguous(ticks):
    result = []
    start = end = None
    for tick in sorted(ticks):
        if end is not None and tick != end + 1:
            result.append((start, end))
            start = None
        if start is None:
            start = tick
        end = tick
    if end is not None:
        result.append((start, end))
    return result


def stepped_metrics(run, time, xyz, rotations, fixture, support, derived, origin, end):
    boxes = fixture["boxes"]
    gap = np.asarray([[np.nan if g is None else g for g in vertical_clearance(pos, rot, boxes)]
                      for pos, rot in zip(xyz, rotations)])
    terrain_names = [box["name"] for box in boxes]
    force_path = run/"support.csv"
    start_tick = round((origin+time[0])*1000)
    end_tick = min(round(end*1000), round((time[-1]+origin)*1000))
    ticks = np.arange(start_tick, end_tick+1)
    wheel_loads = {}
    for i, name in enumerate(terrain_names):
        derived[f"chassis_vertical_clearance_{name}_m"] = [
            "" if math.isnan(item) else item for item in gap[:, i]]
    for link in WHEELS:
        force, labels = support[link]
        all_contact = set().union(*labels.values())
        absent = [(first, last) for first, last in contiguous(set(ticks)-all_contact)
                  if last-first+1 >= 20]
        loads = np.asarray([force.get(int(tick), [0., 0.]) for tick in ticks])
        wheel_loads[link] = {
            "vertical_peak_N": float(loads[:, 0].max()),
            "contact_normal_peak_N": float(loads[:, 1].max()),
            "vertical_median_when_contact_N": float(np.median(loads[loads[:, 0] > 0., 0]))
            if np.any(loads[:, 0] > 0.) else None,
            "contact_wrench_ticks": len(force),
            "no_contact_or_transport_gaps_at_least_20ms": [
                {"start_s": first/1000-origin, "end_s": last/1000-origin,
                 "duration_s": (last-first+1)/1000.}
                for first, last in absent],
            "terrain_contact_steps": {label: len(labels[label]) for label in terrain_names},
        }
    with force_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s"] + [f"{link}_{field}" for link in WHEELS
                                     for field in ("contact", "vertical_N", "normal_N")])
        all_contacts = {link: set().union(*support[link][1].values()) for link in WHEELS}
        for tick in ticks:
            row = [tick/1000-origin]
            for link in WHEELS:
                row.extend((tick in all_contacts[link], *support[link][0].get(int(tick), [None, None])))
            writer.writerow(row)
    peak_pitch = max(abs(math.asin(float(np.clip(-rot[2, 0], -1., 1.)))) for rot in rotations)
    native = run/"native_contact_step.csv"
    direct = direct_wheel_contact(native, start_tick, end_tick, support, origin) if native.exists() else None
    return {
        "direct_solver_step_contact": direct,
        "chassis_vertical_ground_clearance_m": {
            name: float(np.nanmin(gap[:, i])) if np.any(np.isfinite(gap[:, i])) else None
            for i, name in enumerate(terrain_names)},
        "wheel_loading": wheel_loads, "peak_abs_pitch_rad": peak_pitch,
        "native_chassis_terrain_contact_steps": {
            name: len(support[BASE][1][name]) for name in terrain_names},
        "all_wheels_contacted_finite_threshold": all(support[link][1]["finite_threshold"] for link in WHEELS),
        "full_chassis_cleared_threshold": bool(xyz[-1, 0]-.25 > 1.76),
        "force_source": "DART Gazebo Contact point wrenches summed by wheel at 1 ms",
        "absence_limit": "Contact publisher emits only positive detections. A gap alone cannot disambiguate loss from transport; inspect synchronized loads and contact geometry.",
    }


def direct_wheel_contact(path, start, end, support, origin):
    groups = collections.defaultdict(dict)
    with path.open() as stream:
        for row in csv.DictReader(stream):
            instant = int(row["sim_time_ns"])
            tick, remainder = divmod(instant, 1_000_000)
            if remainder or not start <= tick <= end:
                continue
            link = row["collision_entity"].split("::")[1]
            if tick in groups[link]:
                raise ValueError(f"duplicate native contact step {link} {tick}")
            groups[link][tick] = row
    result = {}
    for link in (*WHEELS, BASE):
        rows = groups[link]
        expected = end-start+1
        initialized = {tick for tick, row in rows.items() if row["component_ready"] == "1"}
        zero = {tick for tick in initialized if int(rows[tick]["contact_pairs"]) == 0}
        transport = support[link][1]
        transport_steps = set().union(*transport.values()) & initialized
        wrench_steps = [float(rows[tick]["force_z_N"]) for tick in initialized
                        if int(rows[tick]["wrench_points"])]
        result[link] = {
            "expected_solver_ticks": expected, "rows": len(rows),
            "data_ready_rows": len(initialized), "missing_rows": expected-len(rows),
            "verified_zero_contact_ticks": len(zero),
            "transport_positive_without_solver_positive_ticks": len(transport_steps & zero),
            "solver_positive_without_transport_positive_ticks": len(initialized - zero - transport_steps),
            "longest_verified_loss_s": max(((last-first+1)/1000. for first, last in contiguous(zero)), default=0.),
            "verified_loss_windows_s": [
                {"start_s": first/1000-origin, "duration_s": (last-first+1)/1000.}
                for first, last in contiguous(zero) if last-first+1 >= 10. and link in WHEELS],
            "world_vertical_peak_N": max(wrench_steps) if wrench_steps else None,
            "world_vertical_contact_median_N": float(np.median(wrench_steps)) if wrench_steps else None,
            "finite_threshold_vertical_peak_N": max(
                (float(rows[tick]["force_z_N"]) for tick in transport["finite_threshold"] & initialized),
                default=None),
        }
    return result


def rectangle_distance(one, two):
    a, b, c, d = one
    x, y, u, v = two
    return np.hypot(np.maximum(np.maximum(x-b, a-y), 0.),
                    np.maximum(np.maximum(u-d, c-v), 0.))


def continuous_qr_dwell(qr_rows, requests, events, origin):
    """Measure fresh inner-band coverage at each start of forward requests."""
    records = sorted((*qr_rows, *requests, *events), key=lambda row: row["trace_index"])
    begin = expiry = None
    latest = None
    last_forward = False
    starts = []
    for row in records:
        now = row["read_time_s"]
        if now is None:
            continue
        if row["source"] == "event" and row["event"] in {"case_start", "resume_gate_open"}:
            begin = now
        elif row["source"] == "rendered_qr":
            latest = row
            fresh = (row["valid"] and abs(row["normalized_error"]) <= .02
                     and -1e-9 <= now-row["capture_time_s"] <= .20+1e-9)
            if not fresh:
                begin = expiry = None
            else:
                if begin is None or expiry is None or now > expiry+1e-8:
                    begin = now
                expiry = row["capture_time_s"]+.20
        elif row["source"] == "request":
            if row["phase"] == "blocked_hold":
                begin = expiry = None
            elif (begin is None and latest and latest["valid"] and abs(latest["normalized_error"]) <= .02
                  and -1e-9 <= now-latest["capture_time_s"] <= .20+1e-9):
                begin = now
                expiry = latest["capture_time_s"]+.20
            moving = row["v_mps"] > 0.
            if moving and not last_forward:
                duration = now-begin if begin is not None and expiry is not None and now <= expiry+1e-8 else None
                starts.append({"first_request_s": now-origin, "continuous_fresh_inner_band_s": duration})
            last_forward = moving
    return starts


def interlock_metrics(run, channels, time, events, xyz, ext, speed, yaw_rate,
                      fixture, origin, collision_points, derived):
    crossing = channels["fixture_world_pose"]
    samples = sorted({row["time_s"]: row for row in crossing}.values(), key=lambda row: row["time_s"])
    native_pose = run/"fixture_native_pose.csv"
    if native_pose.exists():
        with native_pose.open() as stream:
            samples = [{"time_s": int(row["sim_time_ns"])/1e9, "x_m": float(row["x_m"]),
                        "y_m": float(row["y_m"])} for row in csv.DictReader(stream)]
    fixture_time = stamps(samples)-origin
    closest = None
    blocked_measurement_rows = 0
    if len(fixture_time):
        pose = arrays(samples, ("x_m", "y_m"))
        indices = nearest(fixture_time, time)
        sx, sy, _ = fixture["boxes"][-1]["size_m"]
        aligned = np.abs(fixture_time[indices]-time) <= .06
        px, py = pose[indices].T
        other = (px-sx/2, px+sx/2, py-sy/2, py+sy/2)
        dist = rectangle_distance(ext.T, other)
        dist[~aligned] = np.nan
        occupied = aligned & (py - sy/2 < fixture["bounds_m"][3]) & (py + sy/2 > fixture["bounds_m"][2])
        blocked_measurement_rows = int(occupied.sum())
        if occupied.any():
            closest = float(np.min(dist[occupied]))
        derived["fixture_conservative_aabb_separation_m"] = [
            "" if math.isnan(value) else value for value in dist]
    hazard_event = "obstacle_detected" if fixture["case"] == "Q14" else "pedestrian_detected"
    trigger = events.get(hazard_event)
    hazard_rows = [row for row in channels["event"] if row.get("event") == hazard_event]
    producer_time = hazard_rows[0].get("producer_time_s") if hazard_rows else None
    first_hazard_index = hazard_rows[0]["trace_index"] if hazard_rows else None
    clear = events.get("fixture_clear_observed")
    auth = events.get("authorized_resume")
    gate = events.get("resume_gate_open")
    requests = channels["request"]
    guard = [row for row in channels["guard_output"] if isinstance(row["read_time_s"], (float, int))]
    request_t = stamps(requests, "read_time_s")-origin
    guard_t = stamps(guard, "read_time_s")-origin
    rv = arrays(requests, ("v_mps", "yaw_rate_radps"))
    gv = arrays(guard, ("v_mps", "yaw_rate_radps"))
    hold_end = gate if gate is not None else math.inf
    pre = (request_t < trigger) if trigger is not None else np.zeros(len(rv), dtype=bool)
    aft = (request_t > gate) if gate is not None else np.zeros(len(rv), dtype=bool)
    guard_aft = (guard_t > gate) if gate is not None else np.zeros(len(gv), dtype=bool)
    requests_after = request_t[aft & (rv[:, 0] > 1e-6)]
    guards_after = guard_t[guard_aft & (gv[:, 0] > 1e-6)]
    stop_result = stop(time, xyz[:, :2], speed, yaw_rate, trigger, hold_end)
    q14 = fixture["case"] == "Q14"
    qr_rows = channels["rendered_qr"]
    valid_qr = [row for row in qr_rows if row["valid"] and row["id"] == "JANUS-DOCK-01"]
    measured_resume_start = None
    timeout_latency = None
    no_motion = None
    if stop_result:
        physical_hold = (time >= stop_result["settle_s"]) & (time < hold_end)
        no_motion = {
            "samples": int(physical_hold.sum()),
            "peak_speed_mps": float(speed[physical_hold].max()) if physical_hold.any() else None,
            "peak_yaw_rate_radps": float(np.abs(yaw_rate[physical_hold]).max()) if physical_hold.any() else None,
            "stop_dwell_until_gate_or_log_end_s": min(time[-1], hold_end)-stop_result["settle_s"],
            "largest_hold_sample_gap_s": float(np.diff(time[physical_hold]).max())
            if np.count_nonzero(physical_hold) > 1 else None,
        }
        restarted = time[(time > hold_end) & (speed > STILL_SPEED)]
        measured_resume_start = float(restarted[0]) if len(restarted) else None
    if trigger is not None:
        after_hazard_message = stamps(guard, "trace_index") > first_hazard_index
        zero_rows = guard_t[(guard_t >= trigger) & after_hazard_message
                            & np.all(abs(gv) < 1e-6, axis=1)]
        timeout_latency = float(zero_rows[0]-trigger) if len(zero_rows) else None
    req_hold = ((request_t >= trigger) & (request_t < hold_end)) if trigger is not None else np.zeros(len(rv), dtype=bool)
    guard_hold = ((guard_t >= trigger+timeout_latency-1e-7) & (guard_t < hold_end)
                  if trigger is not None and timeout_latency is not None else np.zeros(len(gv), dtype=bool))
    if q14:
        qr_sorted = [row for row in qr_rows if row["read_time_s"] is not None]
        qr_t = stamps(qr_sorted, "trace_index")
        qr_invalid_forward = 0
        if len(qr_t):
            for idx in np.flatnonzero(rv[:, 0] > 0.):
                prior = int(np.searchsorted(qr_t, requests[idx]["trace_index"])-1)
                if prior < 0:
                    qr_invalid_forward += 1
                    continue
                observation = qr_sorted[prior]
                age = (request_t[idx]+origin) - observation["capture_time_s"]
                if not observation["valid"] or not -1e-9 <= age <= .20+1e-9:
                    qr_invalid_forward += 1
        else:
            qr_invalid_forward = int(np.count_nonzero(rv[:, 0] > 0.))
    return {
        "fixture_pose_samples": len(samples), "physically_occupied_aligned_samples": blocked_measurement_rows,
        "fixture_pose_evidence": ("native Gazebo system at 50 Hz" if native_pose.exists()
                                  else "received Gazebo PosePublisher callbacks"),
        "hazard_input_time_s": producer_time-origin if producer_time is not None else None,
        "hazard_delivery_sim_delay_s": trigger-(producer_time-origin) if producer_time is not None else None,
        "minimum_obstacle_aabb_separation_m_while_occupied": closest,
        "fixture_y_range_m": [float(np.min(pose[:, 1])), float(np.max(pose[:, 1]))]
        if len(fixture_time) else None,
        "native_obstacle_contact_points": collision_points["crossing_fixture"],
        "lab_control_scope": "scripted fixture hazard and authority, native obstacle collision/pose and measured robot braking",
        "pre_trigger_positive_request_samples": int(np.count_nonzero(pre & (rv[:, 0] > 0.))),
        "first_guard_zero_latency_s": timeout_latency,
        "request_motion_while_interlocked": int(np.count_nonzero(req_hold & np.any(np.abs(rv) > 1e-6, axis=1))),
        "guard_motion_after_first_zero_and_until_gate": int(np.count_nonzero(guard_hold & np.any(np.abs(gv) > 1e-6, axis=1))),
        "stop": stop_result, "physical_hold": no_motion,
        "case_checks": {
            "robot_moving_at_hazard": bool(stop_result and stop_result["trigger_axle_speed_mps"] > STILL_SPEED),
            "stop_dwell_sampled": stop_result is not None,
            "geometry_contact_free": closest is not None and closest > 0 and collision_points["crossing_fixture"] == 0,
            "clear_observed": clear is not None,
            "consent_observed": auth is not None,
            "actual_restart": measured_resume_start is not None,
        },
        "clear_s": clear, "authority_s": auth, "permission_gate_s": gate,
        "first_resumed_request_s": float(requests_after[0]) if len(requests_after) else None,
        "first_resumed_applied_command_s": float(guards_after[0]) if len(guards_after) else None,
        "first_resumed_ground_truth_motion_s": measured_resume_start,
        "rendered_qr_valid_station_observations": len(valid_qr),
        "forward_requests_without_fresh_valid_qr": qr_invalid_forward if q14 else None,
        "qr_fresh_inner_band_dwell_at_forward_starts": continuous_qr_dwell(
            qr_rows, requests, channels["event"], origin) if q14 else None,
        "first_forward_request_s": float(request_t[np.flatnonzero(rv[:, 0] > 1e-6)[0]])
        if np.any(rv[:, 0] > 1e-6) else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    run = args.run
    result = summarise(run)
    (run/"metrics.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    included_exts = {".sdf", ".json", ".jsonl", ".yaml", ".csv", ".txt", ".gz", ".png", ".log"}
    files = {path.name: sha(path) for path in sorted(run.iterdir())
             if (path.is_file() and path.suffix in included_exts and path.name != "manifest.json")}
    manifest = {
        "run_id": run.name, "engine_scope": "native Gazebo Harmonic simulation",
        "integration_base": "629bf31ce9dec5d238e6030fce93e0d5ac69b075",
        "case": result.get("case"),
        "variant": result.get("variant"),
        "seed": 492,
        "files_sha256": files,
        "contact_raw_format": "gzip of repeated little-endian uint32 size and serialized gz.msgs.Contacts",
        "trace_clock": "native absolute sim time. Subtract origin_sim_time_s from timestamp seconds.",
        "data_gaps": ["actuator applied torque and hardware-dependent acceptance"],
    }
    (run/"manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False)+"\n")
    print(f"{run.name} {result['runtime_result']} {result['termination']['reason']}")


if __name__ == "__main__":
    main()
