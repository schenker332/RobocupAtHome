#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String 
from messages.msg import PersonPose 

class NoseOnlyTracker(object):
    def __init__(self):
        rospy.init_node('nose_only_tracker')

        # --- KONFIGURATION ---
        self.pose_topic = rospy.get_param('~pose_topic', '/vision/person_poses')
        self.head_command_topic = rospy.get_param('~head_command_topic', '/head_controller/command')
        self.joint_states_topic = rospy.get_param('~joint_states_topic', '/joint_states')
        self.command_topic = rospy.get_param('~command_topic', '/tracking_command')

        self.head_pan_joint = rospy.get_param('~head_pan_joint', 'head_1_joint')
        self.head_tilt_joint = rospy.get_param('~head_tilt_joint', 'head_2_joint')

        self.img_width = 640.0
        self.img_height = 480.0

        # --- TRACKING SETTINGS ---
        
        self.smoothing_alpha = 0.23
        
        self.pan_gain = 0.38
        self.tilt_gain = 0.38

        self.min_confidence = 0.6 

        # Limits
        self.pan_max = 1.5; self.pan_min = -1.5
        self.tilt_max = 0.9; self.tilt_min = -0.6

        # --- STATUS ---
        self.tracking_enabled = False
        self.last_update_time = rospy.Time(0)
        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.joints_received = False
        
        self.smoothed_error_x = 0.0
        self.smoothed_error_y = 0.0

        # --- ROS ---
        self.pub = rospy.Publisher(self.head_command_topic, JointTrajectory, queue_size=1)
        
        rospy.Subscriber(self.pose_topic, PersonPose, self.pose_callback, queue_size=1)
        rospy.Subscriber(self.joint_states_topic, JointState, self.joint_states_callback, queue_size=1)
        rospy.Subscriber(self.command_topic, String, self.command_callback, queue_size=1)

        self.enable_srv = rospy.Service('~enable_tracking', Empty, self.enable_cb)
        self.disable_srv = rospy.Service('~disable_tracking', Empty, self.disable_cb)
        
        rospy.Timer(rospy.Duration(0.1), self.control_loop)
        rospy.loginfo("NOSE ONLY Tracker bereit. Sende 'an' oder 'aus' an " + self.command_topic)

    def command_callback(self, msg):
        befehl = msg.data.lower().strip()
        if befehl in ['an', 'on', 'start', 'true']:
            if not self.tracking_enabled:
                self.tracking_enabled = True
                rospy.loginfo("Befehl empfangen: Tracking AN")
        
        elif befehl in ['aus', 'off', 'stop', 'false']:
            if self.tracking_enabled:
                self.tracking_enabled = False
                rospy.loginfo("Befehl empfangen: Tracking AUS -> Fahre Home")
                self.move_to_home()

    def move_to_home(self):
        traj = JointTrajectory()
        traj.joint_names = [self.head_pan_joint, self.head_tilt_joint]
        p = JointTrajectoryPoint()
        p.positions = [0.0, 0.0]
        p.time_from_start = rospy.Duration(1.5)
        traj.points.append(p)
        self.pub.publish(traj)

    def enable_cb(self, req): self.tracking_enabled = True; return EmptyResponse()
    def disable_cb(self, req): 
        self.tracking_enabled = False
        self.move_to_home()
        return EmptyResponse()

    def joint_states_callback(self, msg):
        try:
            if self.head_pan_joint in msg.name and self.head_tilt_joint in msg.name:
                self.current_pan = msg.position[msg.name.index(self.head_pan_joint)]
                self.current_tilt = msg.position[msg.name.index(self.head_tilt_joint)]
                self.joints_received = True
        except ValueError: pass

    def pose_callback(self, msg):
        if not self.tracking_enabled: return
        if not msg.keypoints: return

        kps = msg.keypoints
        if len(kps) < 3: return

        nose_x = kps[0]
        nose_y = kps[1]
        nose_conf = kps[2]

        if nose_conf > self.min_confidence:
            self.last_update_time = rospy.Time.now()

            raw_x = (nose_x / self.img_width) - 0.5
            raw_y = (nose_y / self.img_height) - 0.4 

            self.smoothed_error_x = (self.smoothing_alpha * raw_x) + \
                                    ((1.0 - self.smoothing_alpha) * self.smoothed_error_x)
            self.smoothed_error_y = (self.smoothing_alpha * raw_y) + \
                                    ((1.0 - self.smoothing_alpha) * self.smoothed_error_y)

    def control_loop(self, event):
        if not self.tracking_enabled or not self.joints_received: return
        
        time_since = (rospy.Time.now() - self.last_update_time).to_sec()

        
        if time_since < 1.0:
            if abs(self.smoothed_error_x) < 0.03 and abs(self.smoothed_error_y) < 0.03: return

            
            delta_pan = -1.0 * self.smoothed_error_x * self.pan_gain
            delta_tilt = -1.0 * self.smoothed_error_y * self.tilt_gain 
            
            target_pan = self.current_pan + delta_pan
            target_tilt = self.current_tilt + delta_tilt
            
        
            duration = 0.2
        
    
        else:
            target_pan = 0.0
            target_tilt = 0.0
            if abs(self.current_pan) < 0.05 and abs(self.current_tilt) < 0.05: return
            duration = 1.5

        target_pan = max(min(target_pan, self.pan_max), self.pan_min)
        target_tilt = max(min(target_tilt, self.tilt_max), self.tilt_min)

        traj = JointTrajectory()
        traj.joint_names = [self.head_pan_joint, self.head_tilt_joint]
        p = JointTrajectoryPoint()
        p.positions = [target_pan, target_tilt]
        p.time_from_start = rospy.Duration(duration)
        traj.points.append(p)
        self.pub.publish(traj)

if __name__ == '__main__':
    NoseOnlyTracker()
    rospy.spin()