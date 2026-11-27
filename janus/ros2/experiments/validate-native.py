#!/usr/bin/env python3
"""Derive independent accepted gates from the native ROS/Gazebo evidence."""
from collections import Counter, defaultdict
import argparse
import json
import math
from pathlib import Path


def xy(row):
    return row['x_m'], row['y_m']


def stopped(row):
    return abs(row['v_mps']) <= .02 and abs(row['yaw_rate_radps']) <= .05


def stop_after(rows, trigger):
    """First continuously observed 250 ms stop, with the nominal 50 Hz state topic."""
    first = previous = None
    for row in rows:
        stamp = row['time_s']
        if stamp < trigger:
            continue
        if not stopped(row):
            first = previous = None
            continue
        if previous is None or stamp - previous['time_s'] > .040001:
            first = row
        previous = row
        if stamp - first['time_s'] >= .249999:
            return first, row
    return None, None


def percentile(values, fraction):
    if not values:
        return None
    sorted_values = sorted(values)
    return sorted_values[math.ceil(len(values)*fraction)-1]


def summarize(folder):
    execution = json.loads((folder/'execution.json').read_text())
    choices = defaultdict(list)
    latest_observation = None
    camera_motion_violations = 0
    with (folder/'telemetry.jsonl').open() as trace:
        for line in trace:
            row = json.loads(line)
            source = row.pop('source')
            choices[source].append(row)
            if source == 'qr':
                latest_observation = row
            if source == 'request' and row['v_mps'] > 1e-9:
                if not latest_observation or not latest_observation['valid']:
                    camera_motion_violations += 1
                elif (abs(latest_observation['normalized_error']) > .04 or
                      not 0 <= row['time_s']-latest_observation['capture_time_s'] <= .20):
                    camera_motion_violations += 1
    events = {row['event']: row for row in choices['event']}
    body = choices['truth']
    guard = choices['guard_output']
    reports = choices['guard_status']
    requests = choices['request']
    obs = choices['qr']
    end = body[-1] if body else {}
    start = body[0] if body else {}
    case = execution['case_id']
    gates = {
        'fixture_reached_its_end': execution['ran_until_fixture_end'],
        'clock_odom_world_and_joint_topics_sampled': all(choices[key] for key in ('clock','odom','truth','joint')),
        'world_pose_and_odom_contract_valid': not (choices['odom_error'] or choices['map_error']),
        'one_finite_case_interval': bool(body and len(choices['clock']) > 1 and
                                        all(a['time_s'] < b['time_s'] for a,b in zip(body,body[1:]))),
    }
    if 'publishers_per_runtime_topic' in execution:
        gates['exactly_one_clock_odometry_world_and_command_publisher'] = all(
            count == 1 for count in execution['publishers_per_runtime_topic'].values())
    values = {
        'case_interval_s': execution['clock_end_s']-execution['clock_start_s'] if execution['clock_start_s'] is not None else None,
        'applied_twist_samples': len(guard),
        'world_pose_samples': len(body),
        'observation_reasons': dict(Counter(row['reason'] for row in obs)),
        'observed_decode_p95_ms': percentile([row['decode_ms'] for row in obs if row.get('decode_ms') is not None], .95),
    }
    positive = [row for row in requests if row['v_mps'] > 1e-9]
    requested_at_plant = [row for row in guard if row['v_mps'] > 1e-9 and row['time_s'] is not None]
    if not gates['clock_odom_world_and_joint_topics_sampled']:
        return gates, values

    if case in ('M01','M02'):
        values['world_x_displacement_m'] = end['x_m']-start['x_m']
        values['max_xy_displacement_m'] = max(math.dist(xy(row),xy(start)) for row in body)
        if case == 'M01':
            gates.update(zero_twist=not positive and not requested_at_plant,
                         synthetic_xy_drift_within_1mm=values['max_xy_displacement_m'] <= .001)
        if case == 'M02':
            forward_start = positive[0]['time_s'] if positive else None
            brake = events.get('brake',{}).get('read_time_s')
            unwanted = [row['reason'] for row in reports if forward_start is not None and brake is not None
                        and forward_start <= row['time_s'] < brake and row['reason'] != 'ready']
            values['forward_window_guard_faults'] = unwanted
            gates['forward_window_uninterrupted'] = forward_start is not None and not unwanted
            directional_wheels = [row for row in choices['joint'] if brake is not None and brake-.5 <= row['time_s'] < brake-.1]
            gates['both_wheel_rates_and_world_forward_sign'] = (
                bool(directional_wheels) and all(row['left_wheel_radps'] > 0 and row['right_wheel_radps'] > 0
                                                 for row in directional_wheels) and end['x_m'] > start['x_m'])
            if brake is not None:
                first, verified = stop_after(body, brake)
                start_brake = next((row for row in body if row['time_s'] >= brake),None)
                if first:
                    values.update(stop_to_speed_band_s=first['time_s']-brake,
                                  stop_to_dwell_confirmation_s=verified['time_s']-brake,
                                  braking_distance_m=math.dist(xy(first),xy(start_brake)),
                                  brake_trigger_simulation_s=brake)
                gates['measured_250ms_stopped_hold'] = first is not None and stopped(end)
            else:
                gates['measured_250ms_stopped_hold'] = False
            values['error_from_nominal_twist_integral_m'] = math.hypot(end['x_m']-start['x_m']-.24,end['y_m']-start['y_m'])
            values['final_yaw_error_rad'] = abs(end['yaw_rad']-start['yaw_rad'])
            gates['terminal_pose_within_2cm_and_0p05rad'] = (values['error_from_nominal_twist_integral_m'] <= .02
                                                           and values['final_yaw_error_rad'] <= .05)
    elif case in ('Q01','Q02','Q03','Q06','Q08','Q13'):
        valid = [row for row in obs if row['valid']]
        values.update(real_rendered_observations=len(obs), accepted_observations=len(valid),
                      forward_requests=len(positive), invalid_or_expired_forward_requests=camera_motion_violations)
        gates['no_stale_wrong_or_outer_band_forward_request'] = camera_motion_violations == 0
        if case == 'Q08':
            wrong = [row for row in obs if row['reason'] == 'wrong_id']
            values['wrong_ids'] = sorted({c['id'] for row in wrong for c in row['candidates'] if c['id']})
            values['world_xy_drift_m'] = max(math.dist(xy(row),xy(start)) for row in body)
            gates['rendered_nonmatching_id_rejected'] = bool(wrong) and not valid
            gates['never_moved_forward'] = not positive and not requested_at_plant and values['world_xy_drift_m'] <= .001
        elif case == 'Q13':
            fault = events.get('replay_old_frame', {})
            old_time = fault.get('capture_s')
            emission = fault.get('read_time_s')
            restored = events.get('camera_restore',{}).get('read_time_s')
            old = [row for row in obs if emission is not None and row['read_time_s'] >= emission and row['capture_time_s'] == old_time]
            values['old_frame_rejection_reasons'] = sorted({row['reason'] for row in old})
            no_motion = [row for row in positive if emission is not None and restored is not None and emission <= row['time_s'] <= restored]
            gates['expired_real_frame_received_and_rejected'] = bool(old) and all(not row['valid'] for row in old)
            gates['actual_approach_then_no_progress_with_old_frame'] = (emission is not None and restored is not None
                and any(row['time_s'] < emission-.3 for row in positive) and not no_motion)
            gates['continued_only_after_new_valid_observation'] = (restored is not None
                and any(row['time_s'] > restored+.35 for row in positive))
        else:
            end_spec = events.get('standoff',{}).get('read_time_s')
            first_correction = next((row for row in requests if row['state']=='centering' and row['normalized_error']
                                     and abs(row['normalized_error']) > .02),None)
            gates['fresh_rendered_target_controls_motion'] = bool(valid and positive and choices['image'])
            if case in ('Q02','Q03'):
                desired = 1 if case == 'Q02' else -1
                gates['initial_yaw_sign_matches_marker_side'] = bool(first_correction and desired*first_correction['yaw_rate_radps'] > 0)
                values['first_centering_request'] = first_correction
            if positive:
                first_positive = positive[0]
                gates['forward_only_at_inner_center_band'] = abs(first_positive['normalized_error']) <= .02
                values['first_forward_s'] = first_positive['time_s']-execution['clock_start_s']
                values['initial_valid_center_error'] = valid[0]['normalized_error']
            else:
                gates['forward_only_at_inner_center_band'] = False
            center_y = {'Q02':.28,'Q03':-.28}.get(case,0.)
            world_optical_depth = (1.5-end['x_m'])*math.cos(end['yaw_rad'])+(center_y-end['y_m'])*math.sin(end['yaw_rad'])-.25
            values['independent_final_optical_depth_m'] = world_optical_depth
            values['final_normalized_camera_error'] = valid[-1]['normalized_error'] if valid else None
            gates['standoff_marker_fresh_and_within_2_percent'] = bool(valid and execution['clock_end_s']-valid[-1]['capture_time_s'] <= .20
                                                                   and abs(valid[-1]['normalized_error']) <= .02)
            if case != 'Q06':
                gates['independent_optical_standoff_in_245_to_265mm'] = .245 <= world_optical_depth <= .265
                gates['measured_250ms_terminal_stop'] = end_spec is not None and stop_after(body,end_spec)[0] is not None and stopped(end)
            else:
                lost = events.get('visible_loss',{}).get('read_time_s')
                hide = events.get('hide',{}).get('read_time_s')
                restore = events.get('restore',{}).get('read_time_s')
                if lost is not None and restore is not None and hide is not None:
                    first_stop, confirm = stop_after(body,lost)
                    gates['forward_inhibited_for_observed_blackout'] = (any(row['time_s'] < hide for row in positive)
                        and not any(lost <= row['time_s'] < restore for row in positive))
                    gates['measured_250ms_stop_before_marker_restore'] = confirm is not None and confirm['time_s'] <= restore
                    gates['recenter_then_resume_after_real_marker_return'] = any(row['time_s'] > restore+.35 for row in positive)
                    if first_stop:
                        at_loss = next(row for row in body if row['time_s'] >= lost)
                        values['loss_to_stop_s'] = first_stop['time_s']-lost
                        values['loss_to_stop_distance_m'] = math.dist(xy(first_stop),xy(at_loss))
                else:
                    gates['blackout_confirmed_by_observer'] = False
    elif case == 'O04':
        cut = events.get('cut', {}).get('read_time_s')
        abort = next((row for row in reports if row['reason']=='operator_abort'),None)
        timeout = next((row for row in reports if row['reason']=='command_timeout' and cut is not None and row['time_s']>=cut),None)
        gates['moving_before_silence'] = cut is not None and any(row['time_s'] < cut for row in requested_at_plant)
        if timeout:
            values['cut_to_guard_timeout_s'] = timeout['time_s']-cut
            first, confirm = stop_after(body,timeout['time_s'])
            gates['0p25s_command_timeout_and_250ms_stop'] = (values['cut_to_guard_timeout_s'] <= .28
                                            and confirm is not None and abort is not None and confirm['time_s'] < abort['time_s'])
            if first:
                at_timeout = next(row for row in body if row['time_s'] >= timeout['time_s'])
                values['timeout_to_measured_stop_band_s'] = first['time_s']-timeout['time_s']
                values['timeout_braking_distance_m'] = math.dist(xy(first),xy(at_timeout))
        else:
            gates['0p25s_command_timeout_and_250ms_stop'] = False
        gates['abort_latches_against_positive_requests'] = (abort is not None and stopped(end)
            and any(row['state']=='deliberate_post_abort_probe' for row in positive)
            and not any(row['read_wall_s'] > abort['read_wall_s']+.05 for row in requested_at_plant))
    elif case == 'O08':
        stall = next((row for row in reports if row['reason']=='clock_stall'),None)
        authorized = events.get('authorized_restart_input',{})
        restore = events.get('clock_restore',{})
        initial = events.get('clock_cut',{})
        if stall and authorized and restore and initial:
            values['cut_to_clock_stall_wall_s'] = stall['read_wall_s']-initial['read_wall_s']
            values['recovered_observation_gap_before_new_request_s'] = authorized['read_time_s']-restore['read_time_s']
            gates['advancing_movement_then_live_clock_stall'] = any(row['read_wall_s']<stall['read_wall_s'] for row in requested_at_plant)
            gates['zero_on_recovery_until_explicit_new_request'] = not any(stall['read_wall_s'] <= row['read_wall_s'] < authorized['read_wall_s']
                                                                            for row in requested_at_plant)
            gates['new_request_and_measured_motion_after_authorization'] = (any(row['read_wall_s']>authorized['read_wall_s'] for row in requested_at_plant)
                                                                                and end['v_mps'] > .02)
        else:
            gates['live_pause_restore_and_operator_step_seen'] = False
    return gates, values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    gates, values = summarize(args.directory)
    result = {'status': 'passed' if all(gates.values()) else 'failed', 'gates': gates, 'measurements': values}
    text = json.dumps(result, indent=2, allow_nan=False)+'\n'
    (args.directory/'acceptance.json').write_text(text)
    print(text,end='')
    return 0 if result['status']=='passed' else 2


if __name__ == '__main__':
    raise SystemExit(main())
