#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Measure payload bandwidth of the ground-station keyframe stream.

Subscribes to:
  /ground/keyframe         body-frame keyframe cloud, sent once
  /ground/keyframe_poses   one new pose, or every pose after a loop
  /Odometry                vehicle pose for the aircraft icon
"""

import argparse
import io
import time

import rospy
from fast_lio_sam_loop.msg import ground_keyframe, ground_keyframe_poses
from nav_msgs.msg import Odometry

TOPICS = (
    ("/ground/keyframe", ground_keyframe),
    ("/ground/keyframe_poses", ground_keyframe_poses),
    ("/Odometry", Odometry),
)


def serialized_size(msg):
    buf = io.BytesIO()
    msg.serialize(buf)
    return buf.tell()


def fmt_rate(nbytes, seconds):
    if seconds <= 0.0:
        return "0.00 KB/s   0.000 Mbps"
    kbps = nbytes / seconds / 1024.0
    mbps = nbytes * 8.0 / seconds / 1.0e6
    return "{:8.2f} KB/s {:7.3f} Mbps".format(kbps, mbps)


class TopicStat(object):
    def __init__(self):
        self.total_bytes = 0
        self.count = 0
        self.min_size = None
        self.max_size = 0
        self.first_stamp = None
        self.last_stamp = None
        self.samples = []

    def add(self, nbytes, stamp):
        self.total_bytes += nbytes
        self.count += 1
        self.max_size = max(self.max_size, nbytes)
        if self.min_size is None or nbytes < self.min_size:
            self.min_size = nbytes
        self.samples.append((stamp, nbytes))
        if self.first_stamp is None:
            self.first_stamp = stamp
        self.last_stamp = stamp

    def window(self, now, span):
        nbytes = 0
        count = 0
        for stamp, size in self.samples:
            if now - stamp <= span:
                nbytes += size
                count += 1
        return nbytes, count


class BandwidthMonitor(object):
    def __init__(self, window_sec):
        self.window_sec = window_sec
        self.stats = {name: TopicStat() for name, _ in TOPICS}
        self.t0 = time.time()
        self.pose_incremental = 0
        self.pose_full = 0

    def callback(self, msg, topic):
        self.stats[topic].add(serialized_size(msg), time.time())
        if topic == "/ground/keyframe_poses":
            if msg.full_update:
                self.pose_full += 1
            else:
                self.pose_incremental += 1

    def report(self, title):
        now = time.time()
        elapsed = max(now - self.t0, 1e-6)
        print("\n=== {}  elapsed {:.1f} s ===".format(title, elapsed))
        print("{:<28} {:>8} {:>10} {:>10}  {}".format(
            "topic", "msgs", "mean_msg", "max_msg", "while publishing"))
        total_bytes = 0
        window_bytes = 0
        active_bytes = 0
        active_span = 0.0
        for name, _ in TOPICS:
            st = self.stats[name]
            total_bytes += st.total_bytes
            wbytes, _ = st.window(now, self.window_sec)
            window_bytes += wbytes
            span = 0.0
            if st.first_stamp is not None and st.last_stamp > st.first_stamp:
                span = st.last_stamp - st.first_stamp
            active_bytes += st.total_bytes
            active_span = max(active_span, span)
            mean = (st.total_bytes / st.count) if st.count else 0
            print("{:<28} {:8d} {:8.1f} KB {:8.1f} KB  {}".format(
                name, st.count, mean / 1024.0, st.max_size / 1024.0,
                fmt_rate(st.total_bytes, span if span > 0.0 else elapsed)))
        print("-" * 88)
        print("{:<28} {:>8}  while publishing {}".format(
            "TOTAL", "", fmt_rate(active_bytes, active_span if active_span > 0.0 else elapsed)))
        print("{:<28} {:>8}  last {:.0f}s {}".format(
            "TOTAL", "", self.window_sec, fmt_rate(window_bytes, self.window_sec)))
        print("pose messages: incremental {}  full_update {}".format(
            self.pose_incremental, self.pose_full))
        sys_out = sys_stdout()
        if sys_out:
            sys_out.flush()


def sys_stdout():
    import sys
    return sys.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--window", type=float, default=10.0,
                        help="seconds used for the recent-rate column")
    parser.add_argument("--report-period", type=float, default=5.0,
                        help="print a report every N seconds")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="exit after N seconds; 0 keeps running")
    args = parser.parse_args()

    rospy.init_node("measure_ground_bandwidth", anonymous=True)
    monitor = BandwidthMonitor(args.window)
    for name, msg_type in TOPICS:
        rospy.Subscriber(name, msg_type, monitor.callback, callback_args=name,
                         queue_size=50)
    print("Subscribed: {}".format(", ".join(name for name, _ in TOPICS)))
    print("Payload only. /visualization/map_global and /map_path publish only while subscribed.")

    t_end = time.time() + args.duration if args.duration > 0.0 else None
    period = max(args.report_period, 0.5)
    while not rospy.is_shutdown():
        monitor.report("progress")
        if t_end is not None and time.time() >= t_end:
            break
        # Wall clock: rospy.Rate follows /clock and stalls after rosbag --clock ends.
        time.sleep(period)
    monitor.report("final")


if __name__ == "__main__":
    main()
