"""Use the repository command guard with deterministic Python fixture inputs."""
import sys

from twin import ROOT

sys.path.insert(0, str(ROOT.parent))
from janus.ros2.src.arche_experiments.arche_experiments import commands


GUARD_SOURCE = commands.__file__


class CommandInput:
    PUBLISH_PERIOD_S = .05
    ODOMETRY_REWIND_S = .10
    RECOVERY_COMMAND_MPS = .11

    def __init__(self, recovery=False, require_odom_reauthorization=False):
        self.guard = commands.CommandGuard(require_recovery_permission=require_odom_reauthorization)
        self.recovery = recovery
        self.require_odom_reauthorization = require_odom_reauthorization
        self.link = self.publisher = True
        self.odometry_present = True
        self.odometry_rewound = False
        self.epoch = 0
        self.authorized_s = -1.
        self.stale_arrival = False
        self.odom_authorization_arrival = False
        self.odom_recovery_command_arrival = False
        self.odom_authorized_s = None

    def handle_event(self, t, event):
        if event == "publisher_cut":
            self.publisher = False
        elif event == "publisher_resume":
            self.publisher = True
        elif event == "stream_operator_cancel":
            self.guard.abort()
        elif event == "odometry_stream_cut":
            self.odometry_present = False
        elif event == "odometry_stream_restore":
            self.odometry_present = True
        elif event == "odometry_recovery_authorize":
            self.odom_authorization_arrival = True
        elif event == "odometry_recovery_command":
            self.odom_recovery_command_arrival = True
        elif event == "odometry_stamp_rewind":
            self.odometry_rewound = True
        elif event == "link_down":
            self.link = False
            self.authorized_s = None
            self.guard.abort()
        elif event == "link_restored":
            self.link = True
        elif event == "guard_restart" and self.link:
            self.guard = commands.CommandGuard()
            self.epoch += 1
            self.authorized_s = None
        elif event == "fresh_authorization" and self.link and self.epoch:
            self.authorized_s = t
        elif event == "stale_epoch_request":
            self.stale_arrival = True

    def budgets(self):
        budgets = {"publisher_period_s": self.PUBLISH_PERIOD_S,
                   "command_timeout_s": self.guard.command_timeout,
                   "odometry_timeout_s": self.guard.odom_timeout,
                   "odometry_rewind_s": self.ODOMETRY_REWIND_S,
                   "clock_timeout_s": self.guard.clock_timeout}
        if self.require_odom_reauthorization:
            budgets.update(require_recovery_permission=True, new_request_v_mps=self.RECOVERY_COMMAND_MPS)
        return budgets

    def update(self, tick, t, dt):
        self.guard.clock(t, t)
        odometry_stamp = None
        if self.link and self.odometry_present:
            odometry_stamp = round(t - self.ODOMETRY_REWIND_S, 9) if self.odometry_rewound else t
            self.odometry_rewound = False
            self.guard.odometry(odometry_stamp)
        authorization = None
        if self.odom_authorization_arrival:
            authorization = self.guard.authorize_recovery(t)
            if authorization:
                self.odom_authorized_s = t
        self.odom_authorization_arrival = False
        explicit, self.odom_recovery_command_arrival = self.odom_recovery_command_arrival, False
        stale, self.stale_arrival = self.stale_arrival, False
        epoch = self.epoch - int(stale)
        supplied = stale or explicit or (self.publisher and self.link and tick % round(self.PUBLISH_PERIOD_S/dt) == 0)
        authorized = self.link and self.authorized_s is not None and t > self.authorized_s
        forwarded = supplied and authorized and epoch == self.epoch
        candidate_v = self.RECOVERY_COMMAND_MPS if explicit else .14
        if forwarded:
            self.guard.request(candidate_v, 0., t)
        output = self.guard.output(t)
        accepted = forwarded and self.guard.request_stamp == t and output.reason == "ready"
        if self.recovery and not authorized:
            output = commands.Command(reason="await_fresh_authorization" if self.link else "link_down")
        metadata = {"stream_epoch": self.epoch, "stream_link": self.link,
                    "stream_odom_frame_stamp_s": odometry_stamp,
                    "stream_odom_last_valid_stamp_s": self.guard.odom_stamp,
                    "stream_authorized_at_s": self.authorized_s,
                    "stream_input_epoch": epoch if supplied else None,
                    "stream_forwarded": forwarded, "stream_accepted": accepted,
                    "stream_last_request_s": self.guard.request_stamp}
        if self.require_odom_reauthorization:
            metadata.update(stream_input_request_stamp_s=t if supplied else None,
                            stream_input_request_v_mps=candidate_v if supplied else None,
                            stream_explicit_recovery_command=explicit,
                            stream_recovery_permission_pending=self.guard.recovery_pending,
                            stream_recovery_authorization_outcome=authorization,
                            stream_recovery_authorized_at_s=self.odom_authorized_s)
        return output, metadata
