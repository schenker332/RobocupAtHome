#!/usr/bin/env python3
"""
State: Drive And Speak
======================
Fährt zu einem Ort UND spricht gleichzeitig etwas (parallel).
"""

import rospy
import smach
import threading
import rospkg
import os
import random
from pal_interaction_msgs.msg import TtsActionGoal
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Header
from task_control.actions.navigation_controller import NavigationController
from task_control.actions.head_mover import HeadMover
from task_control.config import WAYPOINTS

class StateDriveAndSpeak(smach.State):
    """
    Fährt zu einem Ort und spricht parallel dazu einen Text.
    Kann Text entweder direkt bekommen oder aus random_talks.txt laden.
    """
    
    def __init__(self, location_name, text=None, category=None):
        smach.State.__init__(self, outcomes=['done', 'failed'])
        self.location_name = location_name
        self.nav = NavigationController()
        self.head_mover = HeadMover()
        self.tts_pub = rospy.Publisher('/tts/goal', TtsActionGoal, queue_size=1)
        
        # Text bestimmen
        if text:
            self.text = text
        elif category:
            self.text = self._load_random_text(category)
        else:
            self.text = "I have nothing to say."
            rospy.logwarn(f"StateDriveAndSpeak: Weder Text noch Category angegeben für {location_name}")

    def _load_random_text(self, category):
        """Lädt eine zufällige Zeile aus random_talks.txt für die gegebene Kategorie."""
        try:
            rospack = rospkg.RosPack()
            path = os.path.join(rospack.get_path('audio_capture'), 'scripts', 'prompts', 'random_talks.txt')
            
            if not os.path.exists(path):
                rospy.logerr(f"Prompts file not found: {path}")
                return f"I am driving to {self.location_name}"

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
                return f"I am driving to {self.location_name}"
                
        except Exception as e:
            rospy.logerr(f"Fehler beim Laden von random_talks: {e}")
            return f"I am driving to {self.location_name}"
    
    def execute(self, userdata):
        rospy.loginfo(f"🚗💬 STATE: Fahre zu '{self.location_name}' und spreche...")
        
        # 1. Hole Wegpunkt
        target = WAYPOINTS.get(self.location_name)
        if not target:
            rospy.logerr(f"❌ Unbekannter Ort: {self.location_name}")
            return 'failed'
        
        # 2. PARALLEL: Navigation starten (in eigenem Thread)
        nav_done = threading.Event()
        nav_success = [False]
        
        def navigate():
            rospy.loginfo("   🚗 [Thread] Navigation gestartet...")
            success = self.nav.go_to(target)
            nav_success[0] = success
            nav_done.set()
            rospy.loginfo(f"   🚗 [Thread] Navigation beendet: {'✅' if success else '❌'}")
        
        nav_thread = threading.Thread(target=navigate)
        nav_thread.daemon = True
        nav_thread.start()
        
        # 3. Kopf wieder geradeaus drehen (während Navigation startet)
        rospy.loginfo("   👀 Drehe Kopf geradeaus...")
        self.head_mover.look_at(yaw=0.0, pitch=0.0, duration=0.5)
        
        # 4. Kurz warten bis Navigation läuft
        rospy.sleep(1.0)
        
        # 4. PARALLEL: TTS senden (während Navigation läuft!)
        rospy.loginfo(f"   💬 Spreche: '{self.text}'")
        msg = TtsActionGoal()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.goal_id = GoalID()
        msg.goal.rawtext.text = self.text
        msg.goal.rawtext.lang_id = 'en_GB'
        self.tts_pub.publish(msg)
        
        # 5. Warte auf Navigation
        nav_done.wait()
        
        # 6. Prüfe Ergebnis
        if nav_success[0]:
            rospy.loginfo("✅ Drive & Speak erfolgreich!")
            return 'done'
        else:
            rospy.logwarn("⚠️ Navigation fehlgeschlagen, aber weiter...")
            return 'done'  # Trotzdem weitermachen (Navigation ist nicht kritisch)
