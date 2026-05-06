#!/usr/bin/env python3
"""
State: Look At Door
===================
Dreht den Kopf zur Tür.
"""

import rospy
import smach
from task_control.actions.head_mover import HeadMover

class StateLookAtDoor(smach.State):
    """
    Dreht den Kopf schnell zur Tür (nach rechts).
    """
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.head_mover = HeadMover()
    
    def execute(self, userdata):
        rospy.loginfo("👀 Drehe Kopf zur Tür...")
        
        # Kopf nach LINKS zur Tür drehen (-90 Grad)
        self.head_mover.look_at(yaw=-1.57, pitch=0.0, duration=1.0)  # -90° = -1.57 rad
        rospy.sleep(1)  # Warte kurz, damit man sieht, dass er hinschaut
        
        rospy.loginfo("👀 Drehe Kopf zurück zur Mitte...")
        # Kopf wieder zur MITTE drehen
        self.head_mover.look_at(yaw=0.0, pitch=0.0, duration=1.0)
        rospy.sleep(1.0) # Warte bis Bewegung fertig
        
        rospy.loginfo("✅ Kopf-Blick fertig!")
        return 'done'
