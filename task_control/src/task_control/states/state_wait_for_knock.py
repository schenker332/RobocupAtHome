#!/usr/bin/env python3
"""
State: Wait For Knock
=====================
Wartet auf ein Klopfen oder Klingel-Event von der Audio Detection.
Kann optional einen Text sprechen bevor er wartet.
"""

import rospy
import smach
import rospkg
import os
import random
from std_msgs.msg import String, Header
from pal_interaction_msgs.msg import TtsActionGoal
from actionlib_msgs.msg import GoalID


class StateWaitForKnock(smach.State):
    """
    Wartet auf Audio Event (KNOCK oder DOORBELL).
    Gibt 'done' zurück sobald ein Event erkannt wurde.
    """
    
    def __init__(self, timeout=60.0, category=None):
        """
        Args:
            timeout: Maximale Wartezeit in Sekunden (Standard: 60s)
            category: Optionaler Text aus random_talks.txt der vorher gesprochen wird.
        """
        smach.State.__init__(self, outcomes=['done', 'timeout'])
        self.timeout = timeout
        self.category = category
        self.event_received = False
        self.event_type = None
        
        # TTS Setup (falls category genutzt wird)
        self.tts_pub = rospy.Publisher('/tts/goal', TtsActionGoal, queue_size=1)
        
    def audio_event_callback(self, msg):
        """Callback für Audio Events."""
        if msg.data in ["KNOCK", "DOORBELL"]:
            rospy.loginfo(f"🚪 Audio Event empfangen: {msg.data}")
            self.event_type = msg.data
            self.event_received = True

    def _load_random_text(self, category):
        """Lädt eine zufällige Zeile aus random_talks.txt für die gegebene Kategorie."""
        try:
            rospack = rospkg.RosPack()
            path = os.path.join(rospack.get_path('audio_capture'), 'scripts', 'prompts', 'random_talks.txt')
            
            if not os.path.exists(path):
                return None

            candidates = []
            in_section = False
            
            with open(path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line: continue
                    if line.startswith('[') and line.endswith(']'):
                        in_section = (line[1:-1] == category)
                        continue
                    if in_section:
                        candidates.append(line)
            
            return random.choice(candidates) if candidates else None
        except:
            return None
    
    def execute(self, userdata):
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("🚪 STATE: WAIT FOR KNOCK")
        
        # 1. Optional sprechen
        if self.category:
            text = self._load_random_text(self.category)
            if text:
                rospy.loginfo(f"   💬 Spreche: '{text}'")
                msg = TtsActionGoal()
                msg.header = Header()
                msg.header.stamp = rospy.Time.now()
                msg.goal_id = GoalID()
                msg.goal.rawtext.text = text
                msg.goal.rawtext.lang_id = 'en_GB'
                self.tts_pub.publish(msg)
                # Wir warten NICHT bis er fertig ist, damit wir sofort hören können!
                # Aber wir geben dem Publisher eine winzige Zeit
                rospy.sleep(0.5)

        rospy.loginfo("   Warte auf Klopfen oder Klingel...")
        rospy.loginfo(f"   Timeout: {self.timeout}s")
        rospy.loginfo("="*60 + "\n")
        
        # Reset
        self.event_received = False
        self.event_type = None
        
        # Subscriber für Audio Events
        sub = rospy.Subscriber('/audio/event', String, self.audio_event_callback)
        
        # Warte auf Event oder Timeout
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Event empfangen?
            if self.event_received:
                rospy.loginfo(f"✅ Event erkannt: {self.event_type}")
                rospy.loginfo("   Fortfahren mit nächstem State...\n")
                sub.unregister()
                return 'done'
            
            # Timeout erreicht?
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > self.timeout:
                rospy.logwarn(f"⏰ TIMEOUT nach {self.timeout}s ohne Audio Event!")
                sub.unregister()
                return 'timeout'
            
            # Status alle 5 Sekunden
            if int(elapsed) % 5 == 0:
                remaining = self.timeout - elapsed
                rospy.loginfo_throttle(5, f"   ⏳ Warte noch {remaining:.0f}s auf Klopfen...")
            
            rate.sleep()
        
        sub.unregister()
        return 'timeout'
