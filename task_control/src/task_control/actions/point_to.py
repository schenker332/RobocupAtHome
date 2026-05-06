#!/usr/bin/env python3
"""
Point To Action
===============
Zeigt auf einen bestimmten Punkt/Stuhl mit dem Arm.

VERWENDUNG:
    from task_control.actions.point_to import PointTo
    
    point_to = PointTo()
    point_to.point_at_seat(seat_id=2)  # Zeige auf Stuhl 2
"""

import rospy
from point_to.msg import PointToActionGoal
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID

class PointTo:
    """
    Sendet Point-To Befehle an den Roboter.
    """
    
    def __init__(self):
        self.pub = rospy.Publisher('/point_to/goal', PointToActionGoal, queue_size=1)
        rospy.sleep(0.3)  # Warte bis Publisher bereit
    
    def point_at_seat(self, seat_id, wait=True):
        """
        Zeigt auf einen Stuhl.
        
        Args:
            seat_id: Stuhl-Nummer (1-4 oder pose_id für den Roboter)
            wait: Ob auf das Ende der Bewegung gewartet werden soll
        """
        rospy.loginfo(f"👉 Zeige auf Platz {seat_id}...")
        
        # Erstelle Goal Message
        msg = PointToActionGoal()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = ''
        
        msg.goal_id = GoalID()
        msg.goal_id.stamp = rospy.Time(0)
        msg.goal_id.id = ''
        
        # pose_id entspricht dem Stuhl
        msg.goal.pose_id = seat_id
        
        # Sende Befehl
        self.pub.publish(msg)
        rospy.loginfo(f"   📤 PointTo Command gesendet (pose_id={seat_id})")
        
        if wait:
            # Warte ein bisschen damit die Bewegung ausgeführt wird
            rospy.sleep(2.0)
            rospy.loginfo(f"   ✅ Zeige-Geste sollte fertig sein")
    
    def stop_pointing(self):
        """Setzt den Arm zurück (optional)."""
        # Könnte eine neutrale Pose senden
        pass
