#!/usr/bin/env python3
"""
State: Host Greeting
====================
State 1 der Receptionist Task: Begrüße den Host und lerne sein Gesicht.

AUFGABE:
1. Host registrieren (mit Platzhaltern)
2. Face Learning & Conversation PARALLEL starten
3. Warten bis BEIDE fertig sind
4. Cleanup


OUTCOME:
- 'done': Host erfolgreich registriert (Face + Name/Drink)
- 'failed': Fehler beim Face Learning
"""

import rospy
import smach
import time
from std_msgs.msg import String
from task_control.actions.face_learning import FaceLearning
from task_control.actions.conversation import Conversation
from task_control.actions.head_mover import HeadMover


class StateHostGreeting(smach.State):
    """
    SMACH State für Host Begrüßung.
    Führt Face Learning für den Host durch.
    """
    
    def __init__(self, guest_manager):
        """
        Args:
            guest_manager: GuestManager Instanz (direkt übergeben, nicht via userdata!)
        """
        smach.State.__init__(
            self,
            outcomes=['done', 'failed'],
            input_keys=[],   # Keine input_keys mehr - wir übergeben direkt!
            output_keys=[]
        )
        # Speichere GuestManager direkt als Instanzvariable
        self.guest_manager = guest_manager
        
        # Head Mover für natürliche Kopfbewegungen
        self.head_mover = HeadMover()
        


    def execute(self, userdata):
        """
        Führt State aus.
        
        Parallel:
        - Face Learning sammelt Samples
        - Conversation sammelt Name/Drink via Smart Brain
        
        Beide Actions updated GuestManager direkt sobald Daten ankommen!
            
        Returns:
            'done' wenn erfolgreich
            'failed' bei Fehler
        """
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("🤖 STATE 1: HOST GREETING")
        rospy.loginfo("="*60)
        
        guest_manager = self.guest_manager
        
        # === Face Tracking aktivieren ===
        face_tracking_pub = rospy.Publisher('/tracking_command', String, queue_size=1, latch=True)
        rospy.sleep(0.5)  # Warte kurz bis Publisher bereit ist
        
        rospy.loginfo("👁️  Face Tracking aktivieren...")
        face_tracking_pub.publish(String(data='on'))
        rospy.sleep(0.5)
        
        # Kopfbewegungen NICHT starten - Face Tracking übernimmt!
        # self.head_mover.start()  # DEAKTIVIERT - würde Face Tracking stören
        
        # === PHASE 1: Host registrieren (mit Platzhaltern) ===
        rospy.loginfo("\n💾 Phase 1: Host registrieren (Platzhalter)")
        rospy.loginfo("-" * 60)
        
        guest_manager.register_guest_profile(
            role="host",
            name="[host]",  # Platzhalter - wird durch Conversation aktualisiert
            drink="unknown"  # Platzhalter - wird durch Conversation aktualisiert
        )
        
        
        # === PHASE 2: Starte BEIDE Actions parallel ===
        rospy.loginfo("\n🚀 Phase 2: Starte Face Learning + Conversation")
        rospy.loginfo("-" * 60)
        
        # Face Learning starten
        face_learner = FaceLearning(
            samples_required=10,
            timeout=60.0
        )
        face_learner.start()
        rospy.loginfo("Face Learning gestartet (sammelt 10 Samples)")
        
        # Conversation starten
        conversation = Conversation(
            role="host",
            state_number=1,  # Smart Brain State 1 = Host Greeting
            guest_manager=guest_manager
        )
        conversation.start()
        rospy.loginfo("💬 Conversation gestartet (Smart Brain aktiviert)")
        
        # === PHASE 3: Warte auf BEIDE ===
        rospy.loginfo("\n⏳ Phase 3: Warte auf Face Learning + Conversation")
        rospy.loginfo("-" * 60)
        
        face_done = False
        conversation_done = False
        loop_count = 0
        
        while not rospy.is_shutdown() and not (face_done and conversation_done):
            # === Check Face Learning ===
            if not face_done and face_learner.is_done():
                rospy.loginfo("\nFace Learning abgeschlossen!")
                
                if not face_learner.get_result():
                    rospy.logerr("❌ Face Learning fehlgeschlagen!")
                    
                    # Face Tracking ausschalten bei Fehler
                    rospy.loginfo("👁️  Face Tracking deaktivieren (Fehler)...")
                    face_tracking_pub.publish(String(data='off'))
                    
                    conversation.stop()
                    # self.head_mover.stop()  # War nicht gestartet
                    return 'failed'
                
                # Hole Daten
                samples = face_learner.get_samples()
                track_id = face_learner.get_track_id()
                
                rospy.loginfo(f"   Samples: {len(samples)}")
                rospy.loginfo(f"   Track ID: {track_id}")
                
                # Update GuestManager: Samples hinzufügen
                for sample in samples:
                    guest_manager.add_face_sample("host", sample)
                
                rospy.loginfo(f"✅ {len(samples)} Face Samples zu GuestManager hinzugefügt!\n")
                face_done = True
            
            # === Check Conversation ===
            if not conversation_done and conversation.is_done():
                rospy.loginfo("\n💬 Conversation abgeschlossen!")
                
                # Name/Drink wurden bereits während Conversation updated!
                # (via conversation._on_guest_data Callback)
                
                name = conversation.get_name()
                drink = conversation.get_drink()
                rospy.loginfo(f"   Name: {name}")
                rospy.loginfo(f"   Drink: {drink}")
                rospy.loginfo(f"✅ Name/Drink bereits in GuestManager!\n")
                
                conversation_done = True
            
            # Status-Update alle 2 Sekunden
            if loop_count % 20 == 0:
                progress = face_learner.get_progress()
                face_status = "✅" if face_done else f"⏳ ({progress['collected']}/{progress['target']})"
                conv_status = "✅" if conversation_done else "⏳"
                rospy.loginfo(f"   Status: Face {face_status} | Conversation {conv_status}")
            
            time.sleep(0.1)  # Wall-clock time (nicht rospy.sleep!)
            loop_count += 1
        
        # === PHASE 4: Cleanup ===
        rospy.loginfo("\n🧹 Phase 4: Cleanup")
        rospy.loginfo("-" * 60)
        
        # Face Tracking deaktivieren
        rospy.loginfo("👁️  Face Tracking deaktivieren...")
        face_tracking_pub.publish(String(data='off'))
        rospy.sleep(0.3)
        
        # self.head_mover.stop() # War nicht gestartet - Face Tracking hat Kopf bewegt
        conversation.stop()
        rospy.loginfo("✅ Conversation gestoppt (Smart Brain → idle)")
        
        # === FERTIG ===
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("🎉 STATE 1 ABGESCHLOSSEN!")
        rospy.loginfo("="*60)
        
        # Zeige finales Host-Profil
        host = guest_manager.get_guest_by_role("host")
        rospy.loginfo(f"📋 Host-Profil:")
        rospy.loginfo(f"   Name: {host['name']}")
        rospy.loginfo(f"   Drink: {host['favorite_drink']}")
        rospy.loginfo(f"   Face Samples: {len(host['face_encodings'])}")
        rospy.loginfo("="*60 + "\n")
        
        return 'done'


# === TEST FUNKTION ===
if __name__ == '__main__':
    """Test: State Host Greeting standalone"""
    rospy.init_node('test_state_host_greeting')
    
    from task_control.guest_manager import GuestManager
    
    rospy.loginfo("=== TEST: State Host Greeting ===\n")
    
    # Erstelle Guest Manager
    gm = GuestManager()
    
    # Erstelle State Machine - GuestManager direkt übergeben!
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    
    with sm:
        smach.StateMachine.add(
            'HOST_GREETING',
            StateHostGreeting(gm),  # Direkte Übergabe!
            transitions={
                'done': 'succeeded',
                'failed': 'failed'
            }
        )
    
    # Führe State Machine aus
    outcome = sm.execute()
    
    rospy.loginfo(f"\n=== TEST BEENDET: {outcome} ===")
