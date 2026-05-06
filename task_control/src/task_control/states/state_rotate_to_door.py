#!/usr/bin/env python3
"""
State: Rotate To Door
=====================
Dreht den Roboter um einen bestimmten Winkel (z.B. 135°) zur Tür und
verifiziert die Rotation über die AMCL Pose.
"""

import rospy
import smach
import math
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class StateRotateToDoor(smach.State):
    """
    Dreht den Roboter um 135° zur Tür und prüft die Rotation.
    """
    
    def __init__(self, target_angle_deg=135.0, tolerance_deg=10.0):
        smach.State.__init__(self, outcomes=['done', 'failed'])
        self.target_angle_deg = target_angle_deg
        self.target_angle_rad = math.radians(target_angle_deg)
        self.tolerance_rad = math.radians(tolerance_deg)
        
        self.cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        self.current_pose = None
        self.pose_sub = None
    
    def get_current_yaw(self):
        """Holt den aktuellen Yaw-Winkel aus AMCL Pose."""
        if self.current_pose is None:
            return None
        
        orientation = self.current_pose.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])
        return yaw
    
    def pose_callback(self, msg):
        """Callback für AMCL Pose Updates."""
        self.current_pose = msg
    
    def normalize_angle(self, angle):
        """Normalisiert Winkel auf [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def execute(self, userdata):
        rospy.loginfo(f"🔄 STATE: Drehe um {self.target_angle_deg}° zur Tür...")
        
        # Subscribiere zu AMCL Pose
        self.current_pose = None
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        
        # Warte auf erste Pose
        rospy.loginfo("   ⏳ Warte auf AMCL Pose...")
        timeout = 5.0
        start_time = rospy.Time.now()
        while self.current_pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logerr("❌ Timeout: Keine AMCL Pose erhalten!")
                self.pose_sub.unregister()
                return 'failed'
        
        # Start-Yaw speichern
        start_yaw = self.get_current_yaw()
        target_yaw = self.normalize_angle(start_yaw + self.target_angle_rad)
        
        rospy.loginfo(f"   📍 Start Yaw: {math.degrees(start_yaw):.1f}°")
        rospy.loginfo(f"   🎯 Ziel Yaw: {math.degrees(target_yaw):.1f}°")
        
        # Rotation starten
        twist = Twist()
        rotation_speed = 0.3  # Rotationsgeschwindigkeit (rad/s)
        
        rate = rospy.Rate(10)  # 10 Hz
        max_time = 15.0  # Max 15 Sekunden
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            # Aktuelle Position
            current_yaw = self.get_current_yaw()
            if current_yaw is None:
                continue
            
            # Berechne Differenz zum Ziel
            angle_diff = self.normalize_angle(target_yaw - current_yaw)
            
            # Drehrichtung basierend auf angle_diff wählen (kürzester Weg)
            if angle_diff > 0:
                twist.angular.z = rotation_speed  # Gegen Uhrzeigersinn (CCW)
            else:
                twist.angular.z = -rotation_speed  # Im Uhrzeigersinn (CW)
            
            rospy.loginfo_throttle(1.0, f"   🔄 Aktuell: {math.degrees(current_yaw):.1f}°, Diff: {math.degrees(angle_diff):.1f}°")
            
            # Ziel erreicht?
            if abs(angle_diff) < self.tolerance_rad:
                rospy.loginfo(f"✅ Ziel erreicht! Finale Pose: {math.degrees(current_yaw):.1f}°")
                break
            
            # Timeout?
            if (rospy.Time.now() - start_time).to_sec() > max_time:
                rospy.logwarn(f"⚠️ Timeout! Konnte Ziel nicht exakt erreichen. Diff: {math.degrees(angle_diff):.1f}°")
                break
            
            # Weiter drehen
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # Stoppen
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.pose_sub.unregister()
        
        # Finale Prüfung
        final_yaw = self.get_current_yaw()
        final_diff = abs(self.normalize_angle(target_yaw - final_yaw))
        
        rospy.loginfo(f"   📊 Finale Differenz: {math.degrees(final_diff):.1f}°")
        
        if final_diff < self.tolerance_rad * 2:  # Etwas toleranter für finalen Check
            rospy.loginfo("✅ Rotation erfolgreich!")
            return 'done'
        else:
            rospy.logwarn(f"⚠️ Rotation ungenau, aber weiter. Diff: {math.degrees(final_diff):.1f}°")
            return 'done'  # Trotzdem weitermachen (nicht kritisch)
