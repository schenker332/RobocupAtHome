#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomTFBroadcaster:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/mobile_base_controller/odom")
        self.br = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber(self.odom_topic, Odometry, self.cb, queue_size=50)
        rospy.loginfo("odom_tf_broadcaster: listening on %s", self.odom_topic)

    def cb(self, msg: Odometry):
        # Use message stamp; if invalid, fallback to now
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = msg.header.frame_id         # "odom"
        t.child_frame_id = msg.child_frame_id           # "base_footprint"

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)

def main():
    rospy.init_node("odom_tf_broadcaster")
    OdomTFBroadcaster()
    rospy.spin()

if __name__ == "__main__":
    main()
