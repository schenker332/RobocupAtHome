#!/usr/bin/env python3
"""
State: Guest Compliment
=======================
State 25: Macht dem Gast ein Kompliment basierend auf visueller Analyse.
"""

import rospy
import smach
import time
from std_msgs.msg import String
from task_control.actions.conversation import Conversation

class StateGuestCompliment(smach.State):
    def __init__(self, guest_manager, target_role="guest1"):
        smach.State.__init__(self, outcomes=['done', 'failed'])
        self.guest_manager = guest_manager
        self.target_role = target_role
        self.target_state = 25 # Fixer State für Compliment
        #rospy.loginfo(f"[StateGuestCompliment] Initialisiert für {self.target_role}")

    def execute(self, userdata):
        rospy.loginfo(f"✨ STATE {self.target_state}: COMPLIMENT ({self.target_role})")
        
        # 1. Visuelle Beschreibung holen
        desc = "No visual info available."
        guest = self.guest_manager.get_guest_by_role(self.target_role)
        
        if guest and guest.get("visual_description"):
            raw_desc = guest["visual_description"]
            if raw_desc:
                desc = raw_desc
                rospy.loginfo(f"   Nutze Beschreibung: {desc}")
        
        # 2. Kontext an Smart Brain senden
        ctx_pub = rospy.Publisher('/smart_brain/scene_context', String, queue_size=1, latch=True)
        ctx_pub.publish(String(desc))
        
        # 3. Conversation starten (Auto-Start ist im Prompt konfiguriert)
        conversation = Conversation(
            role=self.target_role,
            state_number=self.target_state,
            guest_manager=self.guest_manager
        )
        
        rospy.loginfo("   Starte Smart Brain für Kompliment...")
        conversation.start()
        
        # Warte bis fertig
        while not conversation.is_done() and not rospy.is_shutdown():
            time.sleep(0.1)
            
        conversation.stop()
        rospy.loginfo("✅ Kompliment erledigt.")
        return 'done'
