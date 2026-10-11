#!/usr/bin/env python3
"""Grab one frame per image topic and save it as .npy (used by M1 probes)."""
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

DTYPES = {"mono16": (np.uint16, 1), "16UC1": (np.uint16, 1), "rgb8": (np.uint8, 3),
          "mono8": (np.uint8, 1), "32FC1": (np.float32, 1)}


def main():
    out_dir, skip, topics = sys.argv[1], int(sys.argv[2]), sys.argv[3:]
    rclpy.init()
    node = Node("probe_grab")
    seen, done = {t: 0 for t in topics}, set()

    def make_cb(topic):
        def cb(msg):
            seen[topic] += 1
            if seen[topic] <= skip or topic in done:
                return
            dtype, ch = DTYPES[msg.encoding]
            arr = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(msg.height, msg.width, ch).squeeze()
            np.save(f"{out_dir}/{topic.strip('/').replace('/', '_')}.npy", arr)
            print(topic, msg.encoding, arr.shape, arr.dtype, "min", arr.min(), "max", arr.max(), flush=True)
            done.add(topic)
        return cb

    for t in topics:
        node.create_subscription(Image, t, make_cb(t), 10)
    while rclpy.ok() and len(done) < len(topics):
        rclpy.spin_once(node, timeout_sec=0.5)


if __name__ == "__main__":
    main()
