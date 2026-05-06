#!/usr/bin/env python3
"""
State: Guest Welcome
====================
State 2 der Receptionist Task: Begrüße den ersten Gast.

AUFGABE:
1. Gast registrieren (mit Platzhaltern)
2. Face Learning & Conversation PARALLEL starten
3. Warten bis BEIDE fertig sind
4. Cleanup

ABLAUF:
- Face Learning sammelt 10 Face Samples im Hintergrund
- Conversation führt Gespräch mit Smart Brain (Name/Drink)
- GuestManager wird SOFORT aktualisiert wenn Daten ankommen
- State koordiniert beide Actions

OUTCOME:
- 'done': Gast erfolgreich registriert (Face + Name/Drink)
- 'failed': Fehler beim Face Learning
"""

import rospy
import smach
import time
import threading
from sensor_msgs.msg import Image
from std_msgs.msg import String
from task_control.actions.face_learning import FaceLearning
from task_control.actions.conversation import Conversation
# Dynamic import to avoid build errors if package not ready yet
try:
    from guest_stylist.srv import AnalyzeSnapshot
    STYLIST_AVAILABLE = True
except ImportError:
    STYLIST_AVAILABLE = False


class StateGuestWelcome(smach.State):
    """
    SMACH State für Gast Begrüßung.
    Führt Face Learning für einen Gast durch.
    Flexibel einsetzbar für verschiedene Gäste (Roles).
    """
    
    def __init__(self, guest_manager, target_role="guest1", target_state=2):
        """
        Args:
            guest_manager: GuestManager Instanz
            target_role: Rolle des Gasts (z.B. "guest1", "guest2")
            target_state: Nummer des States für Smart Brain Prompt (z.B. 2, 12)
        """
        smach.State.__init__(
            self,
            outcomes=['done', 'failed'],
            input_keys=[],
            output_keys=[]
        )
        self.guest_manager = guest_manager
        self.target_role = target_role
        self.target_state = target_state
        #rospy.loginfo(f"[StateGuestWelcome] Initialisiert für {self.target_role} (State {self.target_state})")
    
    def trigger_analysis(self):
        """Triggers vision description in a background thread."""
        if not STYLIST_AVAILABLE:
            return

        def run_analysis():
            rospy.loginfo("[Async] Asking Guest Stylist...")
            try:
                rospy.wait_for_service('/guest_stylist/analyze', timeout=2.0)
                analyze = rospy.ServiceProxy('/guest_stylist/analyze', AnalyzeSnapshot)
                
                # Empty image triggers snapshot in node
                resp = analyze(Image()) 
                
                if resp.description and "Error" not in resp.description:
                    rospy.loginfo(f"[Async] Style: {resp.description}")
                    self.guest_manager.update_guest(self.target_role, visual_description=resp.description)
                
            except Exception as e:
                rospy.logwarn(f"[Async] Stylist failed: {e}")

        t = threading.Thread(target=run_analysis)
        t.daemon = True
        t.start()

    def execute(self, userdata):
        """
        Hauptlogik: Gast begrüßen und Daten sammeln.
        """
        rospy.loginfo(f"▶STATE {self.target_state}: GUEST_WELCOME ({self.target_role})")

        # === Face Tracking aktivieren ===
        face_tracking_pub = rospy.Publisher('/tracking_command', String, queue_size=1, latch=True)
        rospy.sleep(0.5)  # Warte kurz bis Publisher bereit ist
        
        rospy.loginfo("👁️  Face Tracking aktivieren...")
        face_tracking_pub.publish(String(data='on'))
        rospy.sleep(0.5)
        
        # === PHASE 1: Gast mit Platzhaltern registrieren === 
        # Erstelle dummy GuestInfo für Registrierung
        # (Die msg wird hier nicht direkt gebraucht, da wir GuestManager nutzen)
        
        # Registriere Gast
        self.guest_manager.register_guest_profile(
            role=self.target_role,
            name="Unknown",
            drink="Unknown"
        )
        
        # Guest ID ist die Role
        guest_id = self.target_role
        
        #rospy.loginfo(f"✅ Gast registriert mit ID: {guest_id}")
        
        # === PHASE 2: Actions initialisieren === 
        # Face Learning Action
        face_learning = FaceLearning(
            samples_required=10,
            timeout=60.0
        )

        # Conversation Action
        # Signatur: Conversation(role, state_number, guest_manager)
        conversation = Conversation(
            role=self.target_role,
            state_number=self.target_state,
            guest_manager=self.guest_manager
        )
        
        # === PHASE 3: Actions PARALLEL starten ===
        rospy.loginfo(f"\n🚀 Phase 3: Starte Face Learning + Conversation (State {self.target_state})...")
        
        # Trigger Style Analysis (Async)
        self.trigger_analysis()

        # Face Learning starten
        #rospy.loginfo("▶Starte Face Learning...")
        if not face_learning.start():
            rospy.logerr("❌ Face Learning konnte nicht gestartet werden!")
            return 'failed'
        rospy.loginfo("Face Learning läuft im Hintergrund")
        
        # Conversation starten (aktiviert Smart Brain)
        #rospy.loginfo(f"▶️  Starte Conversation (aktiviert Smart Brain State {self.target_state})...")
        conversation.start()
        rospy.loginfo(f"Conversation gestartet - Smart Brain aktiv (State {self.target_state})")
        
        # === PHASE 4: Warte auf BEIDE Actions ===
        rospy.loginfo("\n⏳ Phase 4: Warte auf Face Learning + Conversation...")
        
        face_learning_done = False
        conversation_done = False
        
        timeout = rospy.Time.now() + rospy.Duration(120.0)  # 2 Minuten Timeout
        
        while not rospy.is_shutdown():
            # Check Timeout
            if rospy.Time.now() > timeout:
                rospy.logerr("❌ TIMEOUT: Actions haben zu lange gedauert!")
                conversation.stop()
                return 'failed'
            
            # Check Face Learning Status
            if not face_learning_done and face_learning.is_done():
                rospy.loginfo("Face Learning abgeschlossen!")
                
                # Hole gesammelte Daten von Face Learning
                samples = face_learning.get_samples()
                track_id = face_learning.get_track_id()
                
                rospy.loginfo(f"   → {len(samples)} Face Samples gesammelt")
                rospy.loginfo(f"   → Track ID: {track_id}")
                
                # Face Samples einzeln zum GuestManager hinzufügen
                for sample in samples:
                    self.guest_manager.add_face_sample(self.target_role, sample)
                
                face_learning_done = True
            
            # Check Conversation Status
            if not conversation_done and conversation.is_done():
                rospy.loginfo("✅ Conversation abgeschlossen!")
                conversation_done = True
            
            # BEIDE fertig? → Beende Loop
            if face_learning_done and conversation_done:
                rospy.loginfo("\n🎉 BEIDE Actions abgeschlossen!")
                break
            
            # Zeige Fortschritt
            if not face_learning_done:
                progress = face_learning.get_progress()
                rospy.loginfo_throttle(5, f"   Face Learning: {progress['collected']}/{progress['target']} Samples")
            
            time.sleep(0.1)  # Verwende time.sleep statt rospy.sleep wegen use_sim_time
        
        # === PHASE 5: Cleanup ===
        rospy.loginfo("\n🧹 Phase 5: Cleanup...")
        
        # Face Tracking deaktivieren
        rospy.loginfo("👁️  Face Tracking deaktivieren...")
        face_tracking_pub.publish(String(data='off'))
        rospy.sleep(0.3)
        
        conversation.stop()
        rospy.loginfo("✅ Smart Brain deaktiviert (State 0: IDLE)")
        
        # === PHASE 6: Zeige Ergebnis ===
        rospy.loginfo("\n📊 Ergebnis:")
        guest = self.guest_manager.get_guest_by_role(guest_id)
        if guest:
            rospy.loginfo(f"   Name: {guest['name']}")
            rospy.loginfo(f"   Drink: {guest['favorite_drink']}")
            rospy.loginfo(f"   Face Samples: {len(guest['face_encodings'])}")
        
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo(f"✅ STATE {self.target_state}: GUEST_WELCOME ({self.target_role}) - ABGESCHLOSSEN")
        rospy.loginfo("="*60 + "\n")
        
        return 'done'
