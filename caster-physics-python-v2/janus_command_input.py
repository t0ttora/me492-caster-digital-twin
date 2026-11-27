"""Use the repository command guard with deterministic Python fixture inputs."""
import sys

from twin import ROOT

sys.path.insert(0, str(ROOT.parent))
from janus.ros2.src.arche_experiments.arche_experiments import commands


GUARD_SOURCE = commands.__file__


class CommandInput:
    PUBLISH_PERIOD_S = .05

    def __init__(self, recovery=False):
        self.guard = commands.CommandGuard()
        self.recovery = recovery
        self.link = self.publisher = True
        self.epoch = 0
        self.authorized_s = -1.
        self.stale_arrival = False

    def handle_event(self, t, event):
        if event == "publisher_cut":
            self.publisher = False
        elif event == "stream_operator_cancel":
            self.guard.abort()
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
        return {"publisher_period_s": self.PUBLISH_PERIOD_S,
                "command_timeout_s": self.guard.command_timeout,
                "odometry_timeout_s": self.guard.odom_timeout,
                "clock_timeout_s": self.guard.clock_timeout}

    def update(self, tick, t, dt):
        self.guard.clock(t, t)
        if self.link:
            self.guard.odometry(t)
        stale, self.stale_arrival = self.stale_arrival, False
        epoch = self.epoch - int(stale)
        supplied = stale or (self.publisher and self.link and tick % round(self.PUBLISH_PERIOD_S/dt) == 0)
        authorized = self.link and self.authorized_s is not None and t > self.authorized_s
        forwarded = supplied and authorized and epoch == self.epoch
        if forwarded:
            self.guard.request(.14, 0., t)
        output = self.guard.output(t)
        accepted = forwarded and self.guard.request_stamp == t and output.reason == "ready"
        if self.recovery and not authorized:
            output = commands.Command(reason="await_fresh_authorization" if self.link else "link_down")
        metadata = {"stream_epoch": self.epoch, "stream_link": self.link,
                    "stream_authorized_at_s": self.authorized_s,
                    "stream_input_epoch": epoch if supplied else None,
                    "stream_forwarded": forwarded, "stream_accepted": accepted,
                    "stream_last_request_s": self.guard.request_stamp}
        return output, metadata
