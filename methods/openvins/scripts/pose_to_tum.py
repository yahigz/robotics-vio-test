#!/usr/bin/env python3
"""Subscribe to OpenVINS /ov_msckf/poseimu and write TUM: t tx ty tz qx qy qz qw (seconds).
Pose = IMU frame expressed in OpenVINS global frame (gravity-aligned, origin at init pose)."""
import sys, rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

topic, out = sys.argv[1], sys.argv[2]
rospy.init_node("tum_recorder", anonymous=True)
f = open(out, "w", buffering=1 << 20)
n = [0]
def cb(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    f.write("%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n" %
            (m.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    n[0] += 1
rospy.Subscriber(topic, PoseWithCovarianceStamped, cb, queue_size=100000, tcp_nodelay=True)
rospy.on_shutdown(lambda: (f.close(), sys.stderr.write("tum_recorder: wrote %d poses\n" % n[0])))
rospy.spin()
