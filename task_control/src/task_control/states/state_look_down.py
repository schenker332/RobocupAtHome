#!/usr/bin/env python3
"""
State: Look Down
================
Neigt den Kopf nach unten um auf die Sitze zu schauen.
"""

import rospy
import smach
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class StateLookDown(smach.State):
    """
    Neigt den Kopf nach unten (z.B. pitch=0.8 für Blick auf Stühle).
    """
    
    def __init__(self, pitch=-0.24, pan=0.0, duration=1.5):
        smach.State.__init__(self, outcomes=['done'])
        self.pitch = pitch  # Tilt angle (positive = down)
        self.pan = pan      # Yaw angle (positive = left)
        self.duration = duration
        
        self.pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
    
    def execute(self, userdata):
        rospy.loginfo(f"👀 STATE: Kopf neigen (pitch={self.pitch}, pan={self.pan})...")
        
        # Kurz warten damit Publisher connected ist
        rospy.sleep(0.3)
        
        # Trajectory Message erstellen
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = ["head_1_joint", "head_2_joint"]
        
        point = JointTrajectoryPoint()
        point.positions = [self.pan, self.pitch]
        point.velocities = [0.0, 0.0]
        point.time_from_start = rospy.Duration(self.duration)
        
        traj.points.append(point)
        self.pub.publish(traj)
        
        # Warte bis Bewegung fertig
        rospy.sleep(self.duration + 0.2)
        
        rospy.loginfo(f"✅ Kopf geneigt auf pitch={self.pitch}")
        return 'done'
