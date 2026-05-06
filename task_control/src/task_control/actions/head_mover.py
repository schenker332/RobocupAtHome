#!/usr/bin/env python3
"""
Head Mover
==========
A helper class to move the robot's head slightly and randomly.
Used during conversations to make the robot look more alive.
"""

import rospy
import threading
import random
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class HeadMover:
    def __init__(self):
        # Tiago Head Joints
        self.joint_names = ["head_1_joint", "head_2_joint"]
        
        # Publisher
        self.pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
        
        self.running = False
        self.thread = None
        
        # Center position (where to look generally)
        # 0.0, 0.0 is straight forward
        self.center_pan = 0.0
        self.center_tilt = 0.0 # Slightly up usually looks better for human interaction
        
    def set_center(self, pan, tilt):
        """Update the center position."""
        self.center_pan = pan
        self.center_tilt = tilt

    def look_at(self, yaw, pitch, duration=1.0):
        """
        Move the head to a specific position.
        
        Args:
            yaw: Pan angle in radians (positive = left, negative = right)
            pitch: Tilt angle in radians (positive = down, negative = up)
            duration: Time to reach the position in seconds
        """
        self._move_to(yaw, pitch, duration)
        
    def start(self):
        """Start the background movement thread."""
        if self.running:
            return
            
        rospy.loginfo("🤖 Head Mover started (Natural movement)")
        self.running = True
        self.thread = threading.Thread(target=self._loop)
        self.thread.daemon = True # Kill if main thread dies
        self.thread.start()
        
    def stop(self):
        """Stop the movement thread and reset head to center."""
        if not self.running:
            return
            
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
            
        rospy.loginfo("🤖 Head Mover stopped")
        
        # Reset to center (or 0,0)
        self._move_to(0.0, 0.0, 1.0)

    def _loop(self):
        """Background loop for random movements."""
        rate = rospy.Rate(0.5) # Update every 2 seconds roughly
        
        while self.running and not rospy.is_shutdown():
            # Calculate random target within small box around center
            # Range: +/- 0.15 rad (~8 degrees) for Pan
            # Range: +/- 0.10 rad (~5 degrees) for Tilt
            
            target_pan = self.center_pan + random.uniform(-0.15, 0.15)
            target_tilt = self.center_tilt + random.uniform(-0.10, 0.10)
            
            # Move there slowly
            duration = random.uniform(1.5, 2.5)
            self._move_to(target_pan, target_tilt, duration)
            
            # Sleep for the duration of the movement + random pause
            # We use time.sleep because rate.sleep() is fixed
            time.sleep(duration + random.uniform(0.1, 1.0))

    def _move_to(self, pan, tilt, duration):
        """Publish a trajectory command."""
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.velocities = [0.0, 0.0]
        point.time_from_start = rospy.Duration(duration)
        
        traj.points.append(point)
        self.pub.publish(traj)
