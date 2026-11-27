"""Mac regressions for the integration review's trace and guard findings."""
from hashlib import sha256
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import MagicMock, Mock, call, patch

from arche_experiments.commands import CommandGuard
from arche_experiments.trace_metrics import measure_trace

ROOT = Path(__file__).resolve().parents[1]
COMMAND_CSV = "time_s,v_mps,yaw_rate_radps\n0,0,0\n1,0,0\n"
COMMAND_HASH = sha256(COMMAND_CSV.encode()).hexdigest()
REQUEST = "/janus/cmd_vel/request"
OUTPUT = "/cmd_vel"


def trace_rows(clock, request=(), output=()):
    mark = {"topic": "/janus/replay/event", "run_id": "probe", "command_sha256": COMMAND_HASH}
    yield {**mark, "event": "replay_start", "sim_time_s": 12.0}
    for t in clock:
        yield {"topic": "/clock", "time_s": t}
    for t in (12.0, 13.0):
        for topic in ("/odom", "/janus/ground_truth"):
            yield {"topic": topic, "time_s": t, "x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
        yield {"topic": "/joint_states", "time_s": t, "left_wheel_radps": 0.0}
    for topic, stamps in ((REQUEST, request), (OUTPUT, output)):
        for t in stamps:
            yield {"topic": topic, "bag_time_ns": round(t * 1e9), "v_mps": 0.0, "yaw_rate_radps": 0.0}
    yield {**mark, "event": "replay_end", "sim_time_s": 13.0}


class TraceReviewTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.folder = Path(folder.name)
        self.trace = self.folder / "trace.jsonl"

    def write_trace(self, rows):
        self.trace.write_text("".join(json.dumps(row) + "\n" for row in rows))

    def test_disconnected_clock_blocks_state_overlap(self):
        self.write_trace(trace_rows((1.0, 1.01)))
        metrics, rows = measure_trace(self.trace, "probe", COMMAND_HASH, 0.02)
        self.assertEqual(metrics["status"], "blocked")
        self.assertEqual(metrics["reason"], "insufficient_clock_coverage")
        self.assertEqual(metrics["clock_interval_samples"], 0)
        self.assertEqual(rows, [])

    def test_clock_must_advance_inside_replay(self):
        self.write_trace(trace_rows((11.99, 12.5, 12.5, 13.01)))
        metrics, rows = measure_trace(self.trace, "probe", COMMAND_HASH, 0.02)
        self.assertEqual(metrics["reason"], "insufficient_clock_coverage")
        self.assertEqual(rows, [])

    def test_clock_bounds_state_grid(self):
        self.write_trace(trace_rows((11.99, 12.21, 12.58, 13.01)))
        metrics, rows = measure_trace(self.trace, "probe", COMMAND_HASH, 0.02)
        self.assertEqual(metrics["clock_interval_samples"], 2)
        self.assertAlmostEqual(metrics["clock_max_gap_s"], 0.37)
        self.assertAlmostEqual(rows[0]["time_s"], 0.22)
        self.assertAlmostEqual(rows[-1]["time_s"], 0.58)

    def run_cli(self, request, output):
        self.write_trace(trace_rows((12.0, 13.0), request, output))
        commands = self.folder / "commands.csv"
        commands.write_text(COMMAND_CSV)
        out = self.folder / "report"
        result = subprocess.run(
            [sys.executable, "-B", str(ROOT / "bench.py"), "--case", "M01", "--commands", str(commands),
             "--trace", str(self.trace), "--native-run-id", "probe", "--out", str(out)],
            capture_output=True, text=True,
        )
        run = out / "M01-gazebo-bag-trace"
        return result, json.loads((run / "manifest.json").read_text()), json.loads((run / "metrics.json").read_text()), run

    def test_cli_fails_with_no_guard_output_in_replay(self):
        result, manifest, metrics, run = self.run_cli((12.0, 13.0), (11.99, 13.01))
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertEqual(manifest["check_status"], "failed")
        self.assertEqual(manifest["termination_reason"], "missing_command_overlap")
        self.assertEqual(manifest["case_acceptance"], "blocked_pending_native_case_run")
        self.assertTrue((run / "results.csv").is_file())
        self.assertGreater(metrics["command_overlap_samples"][REQUEST], 0)
        self.assertEqual(metrics["command_overlap_samples"][OUTPUT], 0)

    def test_cli_fails_with_no_request_in_replay(self):
        result, manifest, metrics, _ = self.run_cli((), (12.0, 13.0))
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertEqual(manifest["check_status"], "failed")
        self.assertEqual(metrics["command_overlap_samples"][REQUEST], 0)

    def test_cli_passes_with_both_recorded_command_streams(self):
        result, manifest, metrics, _ = self.run_cli((12.0, 13.0), (12.0, 13.0))
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(manifest["check_status"], "passed")
        self.assertEqual(metrics["command_overlap_samples"], {REQUEST: 51, OUTPUT: 51})


class GuardReviewTests(unittest.TestCase):
    def test_odometry_regression_recovery_requires_advancing_stamp_permission_and_new_request(self):
        guard = CommandGuard(require_recovery_permission=True, allow_odom_regression_recovery=True)
        guard.clock(1.0, 10.0)
        guard.odometry(1.0)
        guard.request(.10, 0., 10.0)
        self.assertEqual(guard.output(10.0).v_mps, .10)
        guard.odometry(.75)
        self.assertEqual(guard.output(10.01).reason, 'odom_regression')
        guard.odometry(1.0)
        self.assertFalse(guard.authorize_recovery(10.01))
        guard.clock(1.02, 10.02)
        guard.odometry(1.02)
        guard.request(.08, 0., 10.02)
        self.assertEqual(guard.output(10.02).reason, 'recovery_permission_required')
        self.assertTrue(guard.authorize_recovery(10.03))
        self.assertEqual(guard.output(10.03).v_mps, 0.)
        guard.request(.08, 0., 10.04)
        self.assertEqual(guard.output(10.04).v_mps, .08)

    def test_odometry_regression_latches_without_explicit_fixture_opt_in(self):
        guard = CommandGuard(require_recovery_permission=True)
        guard.clock(1.0, 10.0)
        guard.odometry(1.0)
        guard.odometry(.75)
        guard.clock(1.02, 10.02)
        guard.odometry(1.02)
        self.assertFalse(guard.authorize_recovery(10.03))
        self.assertEqual(guard.output(10.03).reason, 'odom_regression')

    def test_recovered_clock_permission_drops_fresh_unauthorized_request(self):
        guard = CommandGuard(require_recovery_permission=True)
        guard.clock(1.0, 10.0)
        guard.odometry(1.0)
        guard.request(.1, 0., 10.0)
        self.assertEqual(guard.output(10.0).v_mps, .1)
        self.assertEqual(guard.output(10.6).reason, 'clock_stall')
        self.assertFalse(guard.authorize_recovery(10.6))
        guard.clock(1.04, 10.7)
        guard.odometry(1.04)
        guard.request(.08, 0., 10.71)
        self.assertEqual(guard.output(10.71).reason, 'recovery_permission_required')
        self.assertTrue(guard.authorize_recovery(10.72))
        self.assertEqual(guard.output(10.72).reason, 'command_timeout')
        guard.request(.08, 0., 10.73)
        self.assertEqual(guard.output(10.73).v_mps, .08)

    def test_new_odometry_cannot_reactivate_request_from_outage(self):
        guard = CommandGuard()
        guard.clock(0, 0)
        guard.odometry(0)
        guard.request(0.14, 0, 0)
        self.assertEqual(guard.output(0).v_mps, 0.14)
        guard.clock(0.26, 0.26)
        guard.request(0.14, 0, 0.26)
        self.assertEqual(guard.output(0.26).reason, "odom_timeout")
        guard.clock(0.28, 0.28)
        guard.odometry(0.28)
        self.assertEqual(guard.output(0.28).v_mps, 0)
        guard.request(0.14, 0, 0.28)
        self.assertEqual(guard.output(0.28).v_mps, 0.14)

    def test_clock_recovery_without_timer_drops_cached_command(self):
        guard = CommandGuard()
        guard.clock(0.4, 0)
        guard.odometry(0.4)
        guard.request(0.14, 0, 0)
        self.assertEqual(guard.output(0).v_mps, 0.14)
        guard.clock(0.42, 0.52)
        guard.odometry(0.42)
        self.assertEqual(guard.output(0.52).v_mps, 0)
        guard.request(0.14, 0, 0.52)
        self.assertEqual(guard.output(0.52).v_mps, 0.14)

    def test_future_odometry_clears_request_even_if_clock_catches_up_before_timer(self):
        guard = CommandGuard()
        guard.clock(0.4, 0)
        guard.odometry(0.4)
        guard.request(0.14, 0, 0)
        guard.odometry(0.49)
        guard.clock(0.46, 0.06)
        self.assertEqual(guard.output(0.06).reason, "command_timeout")
        guard.request(0.14, 0, 0.06)
        self.assertEqual(guard.output(0.06).v_mps, 0.14)

    def test_normal_signals_publish_zero_before_teardown_with_live_context(self):
        for stopping_signal in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(stopping_signal=stopping_signal):
                self.check_shutdown(stopping_signal, context_valid=True)

    def test_already_closed_context_skips_publish(self):
        self.check_shutdown(signal.SIGTERM, context_valid=False)

    def check_shutdown(self, stopping_signal, context_valid):
        mocks = {name: MagicMock() for name in (
            "rclpy", "rclpy.clock", "rclpy.executors", "rclpy.node", "rclpy.qos", "rclpy.signals",
            "geometry_msgs.msg", "nav_msgs.msg", "rosgraph_msgs.msg", "std_msgs.msg",
        )}
        mocks["rclpy.node"].Node = object
        mocks["rclpy.executors"].ExternalShutdownException = type("ExternalShutdownException", (Exception,), {})
        twist = object()
        mocks["geometry_msgs.msg"].Twist.return_value = twist
        source = ROOT / "src/arche_experiments/scripts/command_guard"
        loader = SourceFileLoader("review_guard", str(source))
        module = module_from_spec(spec_from_loader(loader.name, loader))
        with patch.dict(sys.modules, mocks):
            loader.exec_module(module)
        lifecycle = Mock()
        module.GuardNode = Mock(return_value=lifecycle.node)
        lifecycle.node.context.ok.return_value = context_valid
        module.rclpy.try_shutdown = lifecycle.shutdown
        module.rclpy.spin.side_effect = KeyboardInterrupt
        handlers = {}

        def register(signum, handler):
            handlers[signum] = handler
            return signal.SIG_DFL

        def spin_once(*args, **kwargs):
            handlers[stopping_signal](stopping_signal, None)

        module.rclpy.spin_once.side_effect = spin_once
        with patch.object(signal, "signal", side_effect=register):
            module.main()
        module.rclpy.init.assert_called_once_with(signal_handler_options=module.SignalHandlerOptions.NO)
        observed = [item for item in lifecycle.mock_calls if item[0] != "node.context.ok"]
        self.assertEqual(observed, ([call.node.command.publish(twist)] if context_valid else []) + [
            call.node.destroy_node(), call.shutdown(),
        ])
