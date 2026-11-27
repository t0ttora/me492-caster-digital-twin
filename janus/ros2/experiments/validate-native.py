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


def odometry_recovery_gates(choices, events, body, first, confirm, gates, values):
    restored = events.get('odom_stream_restored')
    permission_sent = events.get('authorization_signal_sent')
    permission_ack = events.get('recovery_ack_received')
    restart = events.get('authorized_restart_input')
    if not all((first, confirm, restored, permission_sent, permission_ack, restart)):
        gates['odometry_recovery_sequence_recorded'] = False
        return
    restored_time = restored['read_time_s']
    restart_time = restart['read_time_s']
    source_stamp = events['regressive_odometry']['source_capture_s']
    fresh_odom = [row for row in choices['odom'] if restored_time <= row['time_s'] < restart_time
                  and row['time_s'] > source_stamp and row['read_time_s'] is not None
                  and 0 <= row['read_time_s']-row['time_s'] <= .25]
    unauthorized = [row for row in choices['request'] if row['state']=='fresh_unauthorized_request' and row['v_mps'] > 0]
    active = [row for row in choices['guard_output'] if row['v_mps'] > 0 or row['yaw_rate_radps'] != 0]
    stationary = [row for row in body if first['time_s'] <= row['time_s'] < restart_time]
    no_motion_before_restart = not any(restored['read_wall_s']+.04 <= row['read_wall_s'] < restart['read_wall_s']
                                       for row in active)
    gates['advancing_fresh_odometry_during_measured_stationary_hold'] = (
        len(fresh_odom) > 1 and fresh_odom[-1]['time_s']-fresh_odom[0]['time_s'] >= .25
        and bool(stationary) and all(stopped(row) for row in stationary)
        and confirm['time_s'] <= permission_sent['read_time_s'])
    gates['fresh_unauthorized_requests_rejected'] = bool(unauthorized) and no_motion_before_restart
    gates['separate_permission_ack_and_new_command'] = (
        permission_ack['read_wall_s'] >= permission_sent['read_wall_s']
        and restart_time-permission_ack['read_time_s'] >= .25 and no_motion_before_restart)
    motion = next((row for row in body if row['time_s'] > restart_time and row['v_mps'] > .02), None)
    gates['measured_restart_requires_new_authorized_twist'] = bool(motion) and any(
        row['read_wall_s'] > restart['read_wall_s'] and row['v_mps']>0 for row in active)
    values.update(restored_fresh_odometry_samples=len(fresh_odom), unauthorized_requests=len(unauthorized),
                  first_stop_time_s=first['time_s'], confirmed_stop_time_s=confirm['time_s'],
                  permission_sent_time_s=permission_sent['read_time_s'], permission_ack_time_s=permission_ack['read_time_s'],
                  new_request_time_s=restart_time, resumed_motion_time_s=motion['time_s'] if motion else None)


def camera_flow_stall_gates(folder, events, body, obs, requests, guarded, gates, values):
    cut, replay, restore = (events.get(name) for name in ('camera_cut', 'replay_old_frame', 'camera_restore'))
    if not all((cut, replay, restore)):
        gates['camera_bridge_cut_and_restore_recorded'] = False
        return
    cut_s, replay_s, restore_s = (row['read_time_s'] for row in (cut, replay, restore))
    last = next((row for row in reversed(obs) if row['read_wall_s'] < replay['read_wall_s']), None)
    bridge_path = cut.get('camera_path')
    if bridge_path is None:
        # The original duplicate run predates the explicit camera-path event field.
        invocation = (folder/'invocation.txt').read_text()
        launch = (folder/'launch.log').read_text()
        if 'camera_raw_topic:=' not in invocation and '/janus/gz/image (gz.msgs.Image) -> /janus/camera/image_raw' in launch:
            bridge_path = 'native_bridge_process'
    gates['camera_flow_absent_after_cut_until_replay'] = bool(last and last['valid']
        and last['capture_time_s'] <= cut_s and bridge_path=='native_bridge_process'
        and cut_s < replay_s < restore_s)
    if not gates['camera_flow_absent_after_cut_until_replay']:
        return
    capture = last['capture_time_s']
    zero = next((row for row in requests if cut_s <= row['time_s'] < replay_s
                and row['state']=='qr_missing_invalid_or_expired' and row['capture_time_s']==capture
                and row['v_mps']==0 and row['yaw_rate_radps']==0), None)
    gates['camera_freshness_watchdog_zeroes_before_replayed_packet'] = bool(zero and zero['time_s']-capture > .20
        and any(row['time_s'] < zero['time_s'] and row['v_mps'] > .035 for row in body))
    if not zero:
        return
    first, confirm = stop_after(body, zero['time_s'])
    gates['independent_plant_stop_begins_before_replay_and_holds'] = bool(first and confirm
        and first['time_s'] < replay_s and confirm['time_s'] < restore_s
        and all(stopped(row) for row in body if first['time_s'] <= row['time_s'] < restore_s)
        and not any(first['time_s'] <= row['time_s'] < restore_s for row in guarded))
    values.update(camera_flow_cut_s=cut_s, last_valid_capture_s=capture, watchdog_zero_request_s=zero['time_s'],
                  last_capture_age_at_watchdog_s=zero['time_s']-capture, first_replayed_capture_delivery_s=replay_s,
                  camera_flow_restored_s=restore_s, first_stop_time_s=first['time_s'] if first else None,
                  confirmed_stop_time_s=confirm['time_s'] if confirm else None)


def reacquisition_dwell(observations, requests, restored):
    timeline = sorted([*(('observation', row) for row in observations),
                       *(('control', row) for row in requests)], key=lambda item: item[1]['read_wall_s'])
    frame = None
    dwell_start = None
    for kind, row in timeline:
        if kind == 'observation':
            frame = row
            continue
        if row['time_s'] < restored:
            continue
        inside = (frame is not None and frame['valid'] and frame['capture_time_s'] >= restored
                  and 0 <= row['time_s']-frame['capture_time_s'] <= .20
                  and abs(frame['normalized_error']) <= .02)
        if not inside:
            dwell_start = None
        elif dwell_start is None:
            dwell_start = frame['arrival_time_s']
        if row['v_mps'] > 0:
            if dwell_start is None:
                return None
            return dict(start_arrival_s=dwell_start, end_arrival_s=frame['arrival_time_s'],
                        dwell_s=frame['arrival_time_s']-dwell_start, first_fresh_forward_request_s=row['time_s'],
                        first_forward_frame_capture_s=frame['capture_time_s'],
                        first_forward_frame_age_s=row['time_s']-frame['capture_time_s'],
                        first_forward_normalized_error=frame['normalized_error'], station_id=frame['id'])
    return None


def summarize(folder, camera_flow_stall=False):
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
    if camera_flow_stall and (case != 'Q13' or execution.get('q13_mode') != 'duplicate'):
        raise ValueError('camera flow stall verification requires the Q13 native bridge duplicate fixture')
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
    elif case in ('Q01','Q02','Q03','Q04','Q05','Q06','Q07','Q08','Q13','Q17','Q19'):
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
            variant = execution.get('q13_mode','resume')
            fault = events.get('replay_old_frame', {})
            old_time = fault.get('capture_s')
            emission = fault.get('read_time_s')
            restored = events.get('camera_restore',{}).get('read_time_s')
            old = [row for row in obs if emission is not None and row['read_time_s'] >= emission and row['capture_time_s'] == old_time]
            invalid = next((row for row in old if not row['valid']), None)
            observed_at = invalid['read_time_s'] if invalid else None
            values['old_frame_rejection_reasons'] = sorted({row['reason'] for row in old})
            values['fixture_mode'] = variant
            gates['retained_real_frame_received_and_rejected'] = bool(old) and all(not row['valid'] for row in old)
            if invalid:
                values['repeat_capture_age_on_delivery_s'] = invalid['arrival_time_s']-invalid['capture_time_s']
                values['retained_capture_time_s'] = old_time
            if variant == 'regression':
                previous_capture = events.get('camera_cut',{}).get('previous_observed_capture_s')
                gates['fresh_nonduplicate_capture_rejected_as_regressive'] = bool(invalid and previous_capture is not None
                    and old_time < previous_capture and invalid['reason']=='out_of_order_frame'
                    and 0 <= values['repeat_capture_age_on_delivery_s'] <= .20)
                values['immediately_previous_valid_capture_s'] = previous_capture
            else:
                gates['expired_capture_rejected'] = bool(invalid and values['repeat_capture_age_on_delivery_s'] > .20)
            gates['actual_approach_then_no_progress_with_old_frame'] = (emission is not None and restored is not None and invalid is not None
                and any(row['time_s'] < emission and row['v_mps'] > .035 for row in body) and
                not any(observed_at <= row['time_s'] <= restored for row in positive) and
                not any(observed_at+.05 <= row['time_s'] < restored for row in requested_at_plant))
            if observed_at is not None and restored is not None:
                stop_entry, stop_confirmation = stop_after(body, observed_at)
                gates['independent_measured_stop_during_stale_interval'] = (stop_confirmation is not None
                                                                             and stop_confirmation['time_s'] < restored)
                if stop_entry:
                    values['rejection_to_stopped_band_s'] = stop_entry['time_s']-observed_at
            if variant == 'resume':
                gates['continued_only_after_new_valid_observation'] = (restored is not None
                    and any(row['time_s'] > restored+.35 for row in positive)
                    and any(row['time_s'] > restored+.45 and row['v_mps'] > .02 for row in body))
            if camera_flow_stall:
                camera_flow_stall_gates(folder, events, body, obs, requests, requested_at_plant, gates, values)
        elif case == 'Q17':
            abort_stamp = events.get('retry_budget_exhausted',{}).get('read_time_s')
            hidden_stamps = sorted(row['read_time_s'] for key,row in events.items()
                                   if key.startswith('hide_') and key.removeprefix('hide_').isdigit())
            restored_stamps = sorted(row['read_time_s'] for key,row in events.items()
                                     if key.startswith('restore_') and key.removeprefix('restore_').isdigit())
            values.update(measured_retry_restorations=len(restored_stamps), total_fixture_losses=len(hidden_stamps),
                          retry_abort_s=abort_stamp)
            gates['two_retries_then_explicit_abort'] = len(hidden_stamps) == 3 and len(restored_stamps) == 2 and abort_stamp is not None
            ends = restored_stamps + ([abort_stamp] if abort_stamp is not None else [])
            gates['all_three_fixture_gaps_observed_by_camera'] = (len(ends) == 3 and
                all(any(not row['valid'] and row['reason'] == 'no_qr' and start_s < row['capture_time_s'] < end_s
                        for row in obs) for start_s,end_s in zip(hidden_stamps,ends)))
            gates['no_forward_after_budget_is_exhausted'] = (abort_stamp is not None and stopped(end)
                and not any(row['time_s'] > abort_stamp+.10 for row in requested_at_plant))
        else:
            end_spec = events.get('standoff',{}).get('read_time_s')
            first_correction = next((row for row in requests if row['state']=='centering' and row['normalized_error']
                                     and abs(row['normalized_error']) > .02),None)
            gates['fresh_rendered_target_controls_motion'] = bool(valid and positive and choices['image'])
            if case in ('Q02','Q03'):
                desired = 1 if case == 'Q02' else -1
                gates['initial_yaw_sign_matches_marker_side'] = bool(first_correction and desired*first_correction['yaw_rate_radps'] > 0)
                values['first_centering_request'] = first_correction
            if case == 'Q05':
                acquired = events.get('qr_acquired',{}).get('read_time_s')
                initial_yaw = start['yaw_rad']
                sweep = [abs(math.remainder(row['yaw_rad']-initial_yaw,2*math.pi)) for row in body
                         if acquired is not None and row['time_s'] <= acquired]
                values['max_measured_search_displacement_rad'] = max(sweep,default=None)
                values['search_to_acquisition_s'] = acquired-execution['clock_start_s'] if acquired is not None else None
                gates['initial_out_of_view_then_actual_acquisition'] = (acquired is not None and 'search_started' in events
                    and any(not row['valid'] and row['capture_time_s'] < acquired for row in obs))
                gates['bounded_rotation_and_zero_linear_drive_during_search'] = (bool(sweep) and max(sweep) <= math.radians(55)
                    and not any(row['time_s'] < acquired for row in positive))
            if positive:
                first_positive = positive[0]
                gates['forward_only_at_inner_center_band'] = abs(first_positive['normalized_error']) <= .02
                values['first_forward_s'] = first_positive['time_s']-execution['clock_start_s']
                values['initial_valid_center_error'] = valid[0]['normalized_error']
            else:
                gates['forward_only_at_inner_center_band'] = False
            center_y = {'Q02':.28,'Q03':-.28,'Q07':.12,'Q19':.035}.get(case,0.)
            world_optical_depth = (1.5-end['x_m'])*math.cos(end['yaw_rad'])+(center_y-end['y_m'])*math.sin(end['yaw_rad'])-.25
            values['independent_final_optical_depth_m'] = world_optical_depth
            values['final_normalized_camera_error'] = valid[-1]['normalized_error'] if valid else None
            if case not in ('Q06','Q07'):
                values['terminal_observation_age_s'] = execution['clock_end_s']-valid[-1]['capture_time_s'] if valid else None
                values['controller_depth_goal_m'] = execution.get('controller_depth_goal_m', .265)
                gates['standoff_marker_fresh_and_within_2_percent'] = (bool(valid)
                    and 0 <= values['terminal_observation_age_s'] <= .20 and abs(valid[-1]['normalized_error']) <= .02)
                gates['independent_optical_standoff_in_245_to_265mm'] = .245 <= world_optical_depth <= .265
                gates['measured_250ms_terminal_stop'] = end_spec is not None and stop_after(body,end_spec)[0] is not None and stopped(end)
                if case in ('Q04','Q19'):
                    config = json.loads((folder/'fixture.json').read_text())
                    dock_x,dock_y,dock_yaw = config['dock_nominal_axle_xy_yaw']
                    values['sim_nominal_dock_transform_error_m'] = math.hypot(end['x_m']-dock_x,end['y_m']-dock_y)
                    values['sim_nominal_dock_yaw_error_rad'] = math.remainder(end['yaw_rad']-dock_yaw,2*math.pi)
                    values['configured_image_goal_u_px'] = config['target_u_px']
                    values['scope'] = 'camera alignment and mechanistic pose error are separate; no physical dock contact model'
                    gates['configured_target_matches_actual_observer'] = any(math.isclose(row['target_u_px'],config['target_u_px'])
                                                                             for row in valid)
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
                    if case == 'Q07':
                        recovered = [row for row in valid if row['capture_time_s'] > restore]
                        values['restored_station_ids'] = sorted({row['id'] for row in recovered})
                        values['observed_restore_count'] = sum(row['event']=='restore' for row in choices['event'])
                        gates['one_restoration_of_matching_actual_marker'] = bool(recovered) and (
                            values['restored_station_ids']==['JANUS-DOCK-01'] and
                            values['observed_restore_count']==1 and 'restore_service_ack' in events)
                        gates['measured_forward_motion_only_after_fresh_reacquisition'] = bool(recovered) and any(
                            row['time_s'] >= recovered[0]['capture_time_s']+.4 and row['v_mps'] > .02 for row in body)
                        dwell = reacquisition_dwell(obs, requests, restore)
                        values['post_restore_dwell'] = dwell
                        gates['continuous_fresh_inner_band_dwell_before_forward'] = (
                            bool(dwell) and dwell['dwell_s'] >= .4-1e-9 and dwell['station_id']=='JANUS-DOCK-01')
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
    elif case == 'O05':
        injection = events.get('regressive_odometry', {})
        fault = next((row for row in reports if row['reason'] == 'odom_regression'), None)
        before = injection.get('read_time_s')
        gates['actual_motion_before_odometry_fault'] = before is not None and any(
            row['time_s'] < before and row['v_mps'] > .05 for row in body)
        gates['injected_old_pose_reaches_guard_and_latches'] = bool(injection and fault)
        if injection and fault:
            first, confirm = stop_after(body, fault['time_s'])
            if execution.get('o05_recovery'):
                odometry_recovery_gates(choices, events, body, first, confirm, gates, values)
            else:
                gates['measured_250ms_stopped_hold_after_fault'] = confirm is not None and stopped(end)
                gates['positive_request_cannot_override_fault'] = (any(row['time_s']>fault['time_s']+.08 for row in positive)
                    and not any(row['read_wall_s']>fault['read_wall_s']+.06 for row in requested_at_plant))
            values['regressive_capture_s'] = injection['injected_capture_s']
            values['event_to_fault_s'] = fault['time_s']-injection['read_time_s']
            values['scope'] = 'one synthetic regressive odometry stamp injected into native ROS/Gazebo; encoder-only outage untested'
            if first:
                at_fault = next(row for row in body if row['time_s'] >= fault['time_s'])
                values['fault_to_measured_stop_band_s'] = first['time_s']-fault['time_s']
                values['fault_braking_distance_m'] = math.dist(xy(first),xy(at_fault))
    elif case == 'O08':
        authorized = events.get('authorized_restart_input',{})
        restore = events.get('clock_restore',{})
        restore_ack = events.get('clock_restore_service_ack',{})
        cutoff_ack = events.get('clock_cut_service_ack',{})
        advancing = events.get('clock_advancing_observed',{})
        unauthorized = events.get('unauthorized_probe',{})
        permission_sent = events.get('authorization_signal_sent',{})
        permission_ack = events.get('recovery_ack_received',{})
        initial = events.get('clock_cut',{})
        stall = next((row for row in reports if row['reason']=='clock_stall' and initial
                      and row['read_wall_s'] >= initial['read_wall_s']),None)
        ready = all((stall,authorized,restore,restore_ack,cutoff_ack,advancing,unauthorized,permission_sent,permission_ack,initial))
        if ready:
            values['cut_to_clock_stall_wall_s'] = stall['read_wall_s']-initial['read_wall_s']
            values['restore_service_roundtrip_wall_s'] = restore_ack['read_wall_s']-restore['read_wall_s']
            values['permission_ack_to_new_request_s'] = authorized['read_time_s']-permission_ack['read_time_s']
            values['actual_clock_advance_after_restore_ack_s'] = advancing['read_time_s']-restore_ack['read_time_s']
            values['fresh_unauthorized_requests'] = sum(row['state'] == 'fresh_unauthorized_request' for row in positive)
            gates['advancing_movement_then_live_clock_stall'] = any(row['read_wall_s']<stall['read_wall_s'] for row in requested_at_plant)
            gates['separate_restore_ack_and_advancing_clock'] = (restore_ack['read_wall_s'] >= restore['read_wall_s']
                and advancing['read_wall_s'] >= restore_ack['read_wall_s'] and advancing['read_time_s'] > restore['read_time_s'])
            gates['fresh_unauthorized_twist_rejected_with_diagnostic'] = (values['fresh_unauthorized_requests'] > 0 and
                any(row['reason']=='recovery_permission_required' for row in reports) and
                not any(advancing['read_wall_s'] <= row['read_wall_s'] < permission_ack['read_wall_s'] for row in requested_at_plant))
            gates['permission_received_and_no_queued_command_reactivated'] = (permission_ack['read_wall_s'] >= permission_sent['read_wall_s']
                and authorized['read_time_s']-permission_ack['read_time_s'] >= .25 and
                not any(permission_ack['read_wall_s'] <= row['read_wall_s'] < authorized['read_wall_s'] for row in requested_at_plant))
            gates['new_request_and_measured_motion_after_authorization'] = (any(row['read_wall_s']>authorized['read_wall_s'] for row in requested_at_plant)
                                                                                and end['v_mps'] > .02)
        else:
            gates['live_pause_restore_and_operator_step_seen'] = False
    return gates, values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--camera-flow-stall', action='store_true', help='also verify pre-replay camera silence for Q13')
    args = parser.parse_args()
    gates, values = summarize(args.directory, args.camera_flow_stall)
    result = {'status': 'passed' if all(gates.values()) else 'failed', 'gates': gates, 'measurements': values}
    text = json.dumps(result, indent=2, allow_nan=False)+'\n'
    report = 'camera-flow-stall-acceptance.json' if args.camera_flow_stall else 'acceptance.json'
    (args.directory/report).write_text(text)
    print(text,end='')
    return 0 if result['status']=='passed' else 2


if __name__ == '__main__':
    raise SystemExit(main())
