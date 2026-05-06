#!/usr/bin/env python3
"""
State: Turn Back
================
Dreht den Roboter zurück in die Ausgangsposition (nach links).
"""

import rospy
import smach
from geometry_msgs.msg import Twist

class StateTurnBack(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("\u2163 STATE: TURN BACK")
        rospy.loginfo("="*60)
        
        # Parameter müssen mit StateWaitForNextGuest übereinstimmen, aber invertiert
        speed = 0.8  # Positiv = nach links
        duration = 3.2 # Gleiche Dauer wie beim Hindrehen
        
        rospy.loginfo(f"\u2163 Drehe mich ZURÜCK (Speed: {speed}, Duration: {duration}s)...")
        
        twist = Twist()
        twist.angular.z = speed
        
        start_time = rospy.Time.now()
        end_time = start_time + rospy.Duration(duration)
        rate = rospy.Rate(20)
        
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
            
        # Stoppen
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("\u2705 Zurückdrehen beendet.")
        
        return 'done'
