"""Stream native Gazebo contacts to length-prefixed protobuf files."""
import argparse
import gzip
import json
from pathlib import Path
from signal import SIGINT, SIGTERM, signal
import struct
from threading import Event, Lock

from gz.transport13 import Node, SubscribeOptions


def save(output):
    stop = Event()
    for sig in (SIGINT, SIGTERM):
        signal(sig, lambda *_: stop.set())
    topics = json.loads((output/"fixture.json").read_text())["contacts"]
    capture = Node()
    streams = {}

    def record(stream, lock):
        def callback(payload, _info):
            with lock:
                if not stream.closed:
                    stream.write(struct.pack("<I", len(payload)) + payload)
        return callback

    try:
        for link, topic in topics.items():
            stream = gzip.open(output/f"contact-{link}.pb.gz", "wb", compresslevel=1)
            lock = Lock()
            streams[topic] = (stream, lock)
            if not capture.subscribe_raw(topic, record(stream, lock),
                                         "gz.msgs.Contacts", SubscribeOptions()):
                raise RuntimeError(f"Could not subscribe to {topic}")
        (output/"contact_recorder_ready.txt").write_text("subscribed\n")
        stop.wait()
    finally:
        for topic, (stream, lock) in streams.items():
            capture.unsubscribe(topic)
            with lock:
                stream.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    save(parser.parse_args().run)
