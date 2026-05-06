#!/usr/bin/env python3
"""
State: Wait For Next Guest
==========================
State 10: Drehe zur Tür und warte auf den nächsten Gast.

ABLAUF:
1. Roboter dreht sich um ca. 120 Grad (Richtung Tür).
2. Wartet, bis eine Person/Gesicht erkannt wird (via /yolo/face_crop).
3. Transition zu FINISHED (oder nächster Aktion).
"""

import rospy
import smach
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class StateWaitForNextGuest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'failed'])
        # Publisher für Bewegung
        self.cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        self.face_detected = False

    def face_callback(self, msg):
        self.face_detected = True

    def execute(self, userdata):
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("🔄 STATE 10: WAIT_FOR_NEXT_GUEST")
        rospy.loginfo("="*60)
        
        # Kurz warten für Connections
        rospy.sleep(1.0)
        
        # 1. Drehung um ca. 160 Grad (~2.8 rad)
        # -0.8 rad/s * 3.5s ~= -2.8 rad (nach rechts)
        speed = -0.8
        duration = 3.2
        
        rospy.loginfo(f"🔄 Drehe mich zur Tür (Speed: {speed}, Duration: {duration}s)...")
        
        twist = Twist()
        twist.angular.z = speed
        
        # Sicherstellen, dass wir gültige Zeit haben
        while rospy.Time.now().to_sec() == 0:
            rospy.sleep(0.1)
            
        start_time = rospy.Time.now()
        end_time = start_time + rospy.Duration(duration)
        rate = rospy.Rate(20) # 20Hz Senden
        
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
            
        # Stoppen
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("✅ Drehung beendet.")
        
        # 2. Warten auf Gesicht (Crops)
        rospy.loginfo("👀 Warte auf visuelle Erkennung (Crops)...")
        self.face_detected = False
        
        # Subscriber temporär anlegen
        sub = rospy.Subscriber('/yolo/face_crop', Image, self.face_callback)
        
        # Warte-Schleife (max 60s)
        timeout = rospy.Time.now() + rospy.Duration(60.0)
        
        while not rospy.is_shutdown():
            if self.face_detected:
                rospy.loginfo("✅ Gesicht erkannt! (Crop empfangen)")
                sub.unregister()
                # Wir bleiben zur Tür gedreht für die Interaktion
                return 'done'
            
            if rospy.Time.now() > timeout:
                rospy.logwarn("⚠️ Timeout beim Warten auf Gast!")
                sub.unregister()
                # Wir gehen trotzdem weiter
                return 'done' 
                
            rate.sleep()
            
        return 'failed'
