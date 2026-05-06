#!/usr/bin/env python3
"""
State: Speak
============
Sagt einfach nur einen Text (oder zufällig aus einer Kategorie).
"""

import rospy
import smach
import rospkg
import os
import random
from pal_interaction_msgs.msg import TtsActionGoal
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Header

class StateSpeak(smach.State):
    """
    Sagt einen Text.
    Kann Text entweder direkt bekommen oder aus random_talks.txt laden.
    """
    
    def __init__(self, text=None, category=None, delay=0.0):
        smach.State.__init__(self, outcomes=['done'])
        self.tts_pub = rospy.Publisher('/tts/goal', TtsActionGoal, queue_size=1)
        self.delay = delay
        self.category = category
        self.fixed_text = text

    def _load_random_text(self, category):
        """Lädt eine zufällige Zeile aus random_talks.txt für die gegebene Kategorie."""
        try:
            rospack = rospkg.RosPack()
            path = os.path.join(rospack.get_path('audio_capture'), 'scripts', 'prompts', 'random_talks.txt')
            
            if not os.path.exists(path):
                rospy.logerr(f"Prompts file not found: {path}")
                return "I have nothing to say."

            candidates = []
            in_section = False
            
            with open(path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                        
                    if line.startswith('[') and line.endswith(']'):
                        section = line[1:-1]
                        if section == category:
                            in_section = True
                        else:
                            in_section = False
                        continue
                    
                    if in_section:
                        candidates.append(line)
            
            if candidates:
                return random.choice(candidates)
            else:
                rospy.logwarn(f"Keine Texte für Kategorie '{category}' gefunden.")
                return "I have nothing to say."
                
        except Exception as e:
            rospy.logerr(f"Fehler beim Laden von random_talks: {e}")
            return "I have nothing to say."
    
    def execute(self, userdata):
        # Text bestimmen (jedes Mal neu laden für Variation)
        text_to_speak = ""
        if self.fixed_text:
            text_to_speak = self.fixed_text
        elif self.category:
            text_to_speak = self._load_random_text(self.category)
        else:
            rospy.logwarn("StateSpeak: Weder Text noch Category angegeben.")
            return 'done'

        rospy.loginfo(f"💬 Speak State: '{text_to_speak}'")
        
        # Warte auf Subscriber (wichtig beim ersten Start!)
        timeout = 2.0
        start_wait = rospy.Time.now()
        while self.tts_pub.get_num_connections() == 0:
            if (rospy.Time.now() - start_wait).to_sec() > timeout:
                rospy.logwarn("⚠️ TTS Publisher hat keine Subscriber gefunden!")
                break
            rospy.sleep(0.1)
        
        if self.delay > 0:
            rospy.sleep(self.delay)
            
        msg = TtsActionGoal()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.goal_id = GoalID()
        msg.goal.rawtext.text = text_to_speak
        msg.goal.rawtext.lang_id = 'en_GB'
        self.tts_pub.publish(msg)
        
        # Geben wir dem TTS kurz Zeit zum Starten/Sprechen?
        # Da wir kein Feedback haben, warten wir pauschal kurz basierend auf Textlänge
        # ca 0.1s pro Zeichen ist grob geschätzt + Puffer
        wait_time = len(text_to_speak) * 0.1 + 2.0
        rospy.sleep(wait_time)
        
        return 'done'
