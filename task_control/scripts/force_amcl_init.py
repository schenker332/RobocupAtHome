#!/usr/bin/env python3
import rospy
import yaml
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty

def main():
    rospy.init_node('force_amcl_init')
    
    # Pfad zur manuellen Pose
    pose_file = "/home/leoku/project_ws/manual_pose.yaml"
    
    rospy.loginfo(f"Loading pose from {pose_file}...")
    
    try:
        with open(pose_file, 'r') as f:
            # Simple parsing key: value
            data = {}
            for line in f:
                parts = line.split(':')
                if len(parts) == 2:
                    data[parts[0].strip()] = float(parts[1].strip())
            
            x = data.get('initial_pose_x', 0.0)
            y = data.get('initial_pose_y', 0.0)
            a = data.get('initial_pose_a', 0.0)
            
    except Exception as e:
        rospy.logerr(f"Failed to load yaml: {e}")
        return

    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    
    # Warten auf AMCL
    rospy.sleep(3.0)
    
    rospy.loginfo(f"FORCING AMCL TO: x={x}, y={y}, a={a}")
    
    # Pose Message bauen
    p = PoseWithCovarianceStamped()
    p.header.frame_id = "map"
    p.header.stamp = rospy.Time.now()
    p.pose.pose.position.x = x
    p.pose.pose.position.y = y
    p.pose.pose.position.z = 0.0
    
    # Euler to Quaternion (simple Z rot)
    import math
    from tf.transformations import quaternion_from_euler
    q = quaternion_from_euler(0, 0, a)
    p.pose.pose.orientation.x = q[0]
    p.pose.pose.orientation.y = q[1]
    p.pose.pose.orientation.z = q[2]
    p.pose.pose.orientation.w = q[3]
    
    # Sehr kleine Kovarianz -> "Wir sind uns sicher!"
    # Diagonale Elemente
    p.pose.covariance[0] = 0.05  # x
    p.pose.covariance[7] = 0.05  # y
    p.pose.covariance[35] = 0.05 # yaw
    
    # Sende es mehrfach, um sicherzugehen
    for i in range(5):
        p.header.stamp = rospy.Time.now()
        pub.publish(p)
        rospy.sleep(0.5)
        
    rospy.loginfo("Initial pose sent! Requesting nomotion update...")
    
    # Nomotion update callen, damit AMCL die Partikel sofort resampelt
    try:
        rospy.wait_for_service('/request_nomotion_update', timeout=2.0)
        u = rospy.ServiceProxy('/request_nomotion_update', Empty)
        u()
    except:
        pass

    rospy.loginfo("AMCL Forced Initialization DONE.")

if __name__ == "__main__":
    main()
