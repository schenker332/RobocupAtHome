#!/usr/bin/env python3
"""
Receptionist State Machine
===========================
Hauptprogramm für die Receptionist Task.

STRUKTUR:
- State 1: HOST_GREETING - Begrüße Host, lerne Gesicht
- State 2: GUEST_WELCOME - Begrüße ersten Gast
- State 3: FINISHED - Task abgeschlossen

VERWENDUNG:
    rosrun task_control receptionist_sm.py
"""

import rospy
import smach
import smach_ros
from messages.msg import GuestList
from task_control.guest_manager import GuestManager
from task_control.states.state_host_greeting import StateHostGreeting
from task_control.states.state_guest_welcome import StateGuestWelcome
from task_control.states.state_introduce_guest import StateIntroduceGuest
from task_control.states.state_verify_seats import StateVerifySeats
from task_control.states.state_wait_for_next_guest import StateWaitForNextGuest
from task_control.states.state_verify_changes import StateVerifyChanges
from task_control.states.state_guest_compliment import StateGuestCompliment
from task_control.states.state_drive_and_compliment import StateDriveAndCompliment
from task_control.states.state_wait_for_knock import StateWaitForKnock
from task_control.states.state_look_at_door import StateLookAtDoor
from task_control.states.state_drive_and_speak import StateDriveAndSpeak
from task_control.states.state_speak import StateSpeak
from task_control.states.state_open_door import StateOpenDoor
from task_control.states.state_rotate_to_door import StateRotateToDoor
from task_control.states.state_look_down import StateLookDown
from task_control.actions.navigation_controller import NavigationController
from task_control.config import WAYPOINTS

class StateMoveTo(smach.State):
    """
    Fährt zu einem definierten Wegpunkt (aus WAYPOINTS).
    """
    def __init__(self, location_name):
        smach.State.__init__(self, outcomes=['done', 'failed'])
        self.location_name = location_name
        self.nav = None  # Wird erst bei execute erstellt (Lazy Loading)

    def execute(self, userdata):
        rospy.loginfo(f"🚗 STATE: Fahre zu '{self.location_name}'...")
        
        # NavigationController erst JETZT initialisieren (beim ersten Ausführen)
        if self.nav is None:
            self.nav = NavigationController()
        
        target = WAYPOINTS.get(self.location_name)
        if not target:
            rospy.logerr(f"❌ Unbekannter Ort: {self.location_name}")
            return 'failed'
            
        success = self.nav.go_to(target)
        
        if success:
            return 'done'
        else:
            return 'failed'


def scan_callback(msg, guest_manager):
    """
    Dauerhafter Scanner: Prüft jedes Gesicht auf Bekannte.
    """
    for guest in msg.guests:
        track_id = guest.current_track_id
        
        # Nur prüfen, wenn wir ein Face Encoding haben
        if len(guest.face_encoding) == 0:
            # DEBUG: Sehen wir überhaupt ein Gesicht?
            # rospy.loginfo_throttle(2.0, f"👀 [ID {track_id}] Kein Face Encoding (Gesicht zu klein/unscharf?)")
            continue
            
        # Hole detaillierte Liste aller Vergleiche
        results = guest_manager.identify_guest_detailed(guest.face_encoding, track_id)
        
        if not results:
            rospy.loginfo_throttle(2.0, f"👀 [ID {track_id}] Keine Vergleichsdaten im Speicher!")
            continue

        # Baue Log-String
        log_str = f"👀 [ID {track_id}] "
        best_match = None
        best_dist = 100.0
        
        # Details anhängen
        details = []
        for res in results:
            match_mark = "★" if res["is_match"] else " "
            # Zeige IMMER die Distanz an, auch wenn kein Match!
            details.append(f"{match_mark}{res['name']}:{res['dist']:.3f}")
            
            # Besten finden für Zusammenfassung
            if res["dist"] < best_dist:
                best_dist = res["dist"]
                if res["is_match"]:
                    best_match = res["name"]

        log_str += " | ".join(details)
        
        # Fazit
        if best_match:
            log_str += f" -> ✅ {best_match}"
            
            # NEU: Aktualisiere das Gedächtnis mit der aktuellen Track ID
            for res in results:
                if res["name"] == best_match and res["is_match"]:
                    guest_manager.update_track_id(res["role"], track_id)
                    break
        else:
            log_str += f" -> ❓ UNBEKANNT (Beste Dist: {best_dist:.3f})"
            
        rospy.loginfo(log_str)


class WaitState(smach.State):
    """Einfacher State für Pausen zwischen den Aktionen mit Countdown."""
    def __init__(self, seconds=5):
        smach.State.__init__(self, outcomes=['done'])
        self.seconds = seconds
    
    def execute(self, userdata):
        rospy.loginfo(f"⏳ Puffer-Pause: {self.seconds} Sekunden...")
        for i in range(self.seconds, 0, -1):
            rospy.loginfo(f"   [{i}s rest...]")
            rospy.sleep(1.0)
        return 'done'


class StateScanning(smach.State):
    """
    State 3: Scanning
    Einfach nur da sein und scannen (Endlosschleife).
    Der scan_callback macht die eigentliche Arbeit.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['aborted'])
    
    def execute(self, userdata):
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("👀 STATE 3: SCANNING MODE")
        rospy.loginfo("   (Ich laufe jetzt endlos und melde, wen ich sehe...)")
        rospy.loginfo("="*60 + "\n")
        
        # Endlosschleife - wir warten einfach
        # Der Scanner läuft im Hintergrund weiter!
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            
        return 'aborted'


class StateFinished(smach.State):
    """Finaler State - Task abgeschlossen."""
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
    
    def execute(self, userdata):
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("🏁 TASK ABGESCHLOSSEN")
        rospy.loginfo("="*60 + "\n")
        return 'done'


# --- MAIN ---

def main():
    """Hauptfunktion - Erstellt und startet State Machine."""
    rospy.init_node('receptionist_state_machine')
    
    # Unterdrücke SMACH Debug-Outputs
    import logging
    logging.getLogger('smach').setLevel(logging.WARNING)
    
    # === Start-State (fest definiert) ===
    start_state = 'MOVE_TO_HOST'  # Fest - nicht mehr per Parameter änderbar

    rospy.loginfo(f"Start-State: {start_state}")
    rospy.loginfo("="*60)
    
    # === Guest Manager erstellen ===
    guest_manager = GuestManager()
    
    # === HINTERGRUND SCANNER STARTEN ===
    # Lauscht dauerhaft auf /vision/guest_info und prüft auf Bekannte
    rospy.Subscriber('/vision/guest_info', GuestList, scan_callback, callback_args=guest_manager)


    # === State Machine erstellen ===
    sm = smach.StateMachine(outcomes=['task_complete', 'task_failed'])
    
    # Setze Initial State
    sm.set_initial_state([start_state])
    
    with sm:
        # STATE 0: Fahre zur Host-Position (Startpunkt)
        smach.StateMachine.add('MOVE_TO_HOST', StateMoveTo('host'), transitions={'done': 'HOST_GREETING', 'failed': 'task_failed'})

        # STATE 1: Host Greeting
        smach.StateMachine.add('HOST_GREETING', StateHostGreeting(guest_manager), transitions={'done': 'MOVE_TO_SEATS', 'failed': 'task_failed'})
        
        # NAVIGATION: Zu den Stühlen fahren (mit modularer Konversation)
        smach.StateMachine.add('MOVE_TO_SEATS', StateDriveAndSpeak('seats', category='MOVE_TO_SEATS'), transitions={'done': 'WAIT_FOR_KNOCK', 'failed': 'task_failed'})

        # Warte auf Klopfen/Klingel (spricht vorher einen Satz aus der Kategorie)
        smach.StateMachine.add('WAIT_FOR_KNOCK', StateWaitForKnock(timeout=120.0, category='WAITING_FOR_KNOCK'), transitions={'done': 'LOOK_AT_DOOR_1', 'timeout': 'LOOK_AT_DOOR_1'})
        
        # Drehe Kopf zur Tür
        smach.StateMachine.add('LOOK_AT_DOOR_1', StateLookAtDoor(), transitions={'done': 'DRIVE_AND_SPEAK_1'})

        # NAVIGATION + SPRECHEN: Fahre zur Tür und sage etwas Nettes
        smach.StateMachine.add('DRIVE_AND_SPEAK_1', StateDriveAndSpeak('door', "Oh, someone is at the door! How wonderful! I will go and open it right away."), transitions={'done': 'CALCULATE_DOOR', 'failed': 'task_failed'})

        # ZWISCHEN-STATE: Lustige Berechnung ankündigen (dient als Delay)
        smach.StateMachine.add('CALCULATE_DOOR', StateSpeak(text="Let me check the door. I am calculating how to open it best. It looks quite difficult, but I will manage."), transitions={'done': 'OPEN_DOOR_1'})

        # Öffne die Tür (wartet auf success vom Action Server, max 120s)
        # pre_delay ist 0, da wir ja schon während des Sprechens gewartet haben
        smach.StateMachine.add('OPEN_DOOR_1', StateOpenDoor(timeout=120.0, pre_delay=0.0), transitions={'done': 'MOVE_TO_GREETING', 'timeout': 'MOVE_TO_GREETING'})

        # Fahre zur greeting Position (besser vor dem Gast)
        smach.StateMachine.add('MOVE_TO_GREETING', StateDriveAndSpeak('greeting', "Let me position myself so I can see you properly. I don't have my glasses on, so I have to get a bit closer to you."), transitions={'done': 'GUEST_WELCOME', 'failed': 'task_failed'})

        # STATE 2: Guest Welcome & Bild machen am anfang
        smach.StateMachine.add('GUEST_WELCOME', StateGuestWelcome(guest_manager), transitions={'done': 'REQUEST_CLOSE_DOOR_1', 'failed': 'task_failed'})
        
        # Bitte den Gast, die Tür zu schließen
        smach.StateMachine.add('REQUEST_CLOSE_DOOR_1', StateSpeak(text="Could you please close the door behind you? That would be very helpful."), transitions={'done': 'DRIVE_AND_COMPLIMENT_1'})

        # NAVIGATION + KOMPLIMENT: Fahre zum Wohnzimmer und mache gleichzeitig Kompliment (Gast 1)
        smach.StateMachine.add('DRIVE_AND_COMPLIMENT_1', StateDriveAndCompliment(guest_manager, 'seats', target_role="guest1"), transitions={'done': 'LOOK_DOWN_FOR_VERIFY', 'failed': 'task_failed'})
        

        # Kopf nach unten neigen um Stühle zu sehen (VOR Introduce)
        smach.StateMachine.add('LOOK_DOWN_FOR_VERIFY', StateLookDown(pitch=-0.3), transitions={'done': 'INTRODUCE_GUEST'})

        # STATE 8: Introduce Guest (Vorstellung & Sitzplatz)
        smach.StateMachine.add('INTRODUCE_GUEST', StateIntroduceGuest(guest_manager), transitions={'done': 'VERIFY_SEATS', 'failed': 'task_failed'})

        # STATE 9: Verify Seats (Speichern wer wo sitzt)
        smach.StateMachine.add('VERIFY_SEATS', StateVerifySeats(guest_manager), transitions={'done': 'LOOK_UP_AFTER_VERIFY', 'failed': 'task_failed'})

        # Kopf wieder hoch (nach dem Verifizieren)
        smach.StateMachine.add('LOOK_UP_AFTER_VERIFY', StateLookDown(pitch=0.0), transitions={'done': 'MOVE_TO_DOOR_2'})

        # === GAST 2: Gleicher Ablauf wie Gast 1 ===
        
        # Fahre zurück zur Tür und rede über die Aufregung
        smach.StateMachine.add('MOVE_TO_DOOR_2', StateDriveAndSpeak('door', "I have a feeling that a second guest is coming soon! I am so excited. "), transitions={'done': 'WAIT_FOR_KNOCK_2', 'failed': 'task_failed'})

        # Warte auf Klopfen (Gast 2)
        smach.StateMachine.add('WAIT_FOR_KNOCK_2', StateWaitForKnock(timeout=120.0), transitions={'done': 'OPEN_DOOR_2', 'timeout': 'OPEN_DOOR_2'})

        # Öffne die Tür (Gast 2)
        smach.StateMachine.add('OPEN_DOOR_2', StateOpenDoor(timeout=120.0, pre_delay=0.0), transitions={'done': 'MOVE_TO_GREETING_2', 'timeout': 'MOVE_TO_GREETING_2'})

        # Fahre zur greeting Position (Gast 2)
        smach.StateMachine.add('MOVE_TO_GREETING_2', StateDriveAndSpeak('greeting', "Great that you are here! The party is in full swing, but I have to reposition myself so I can see you better."), transitions={'done': 'GUEST_WELCOME_2', 'failed': 'task_failed'})

        # STATE 12: Guest 2 Welcome (Begrüßen & Scannen)
        smach.StateMachine.add('GUEST_WELCOME_2', StateGuestWelcome(guest_manager, target_role="guest2", target_state=12), transitions={'done': 'REQUEST_CLOSE_DOOR_2', 'failed': 'task_failed'})
        
        # Bitte den zweiten Gast, die Tür zu schließen
        smach.StateMachine.add('REQUEST_CLOSE_DOOR_2', StateSpeak(text="Could you please close the door behind you as well? Thank you!"), transitions={'done': 'DRIVE_AND_COMPLIMENT_2'})

        # Navigation zum Wohnzimmer & Kompliment machen (Guest 2)
        smach.StateMachine.add('DRIVE_AND_COMPLIMENT_2', StateDriveAndCompliment(guest_manager, 'seats', target_role="guest2"), transitions={'done': 'LOOK_DOWN_FOR_VERIFY_2', 'failed': 'task_failed'})

        # Kopf nach unten neigen um Stühle zu sehen (VOR Verify Changes)
        smach.StateMachine.add('LOOK_DOWN_FOR_VERIFY_2', StateLookDown(pitch=-0.3), transitions={'done': 'VERIFY_CHANGES'})

        # STATE 13: Verify Changes (Hat sich wer umgesetzt?)
        smach.StateMachine.add('VERIFY_CHANGES', StateVerifyChanges(guest_manager), transitions={'done': 'INTRODUCE_GUEST_2', 'failed': 'task_failed'})
        
        # STATE 14: Introduce Guest 2
        smach.StateMachine.add('INTRODUCE_GUEST_2', StateIntroduceGuest(guest_manager, target_role="guest2", target_state=14), transitions={'done': 'FINAL_VERIFY', 'failed': 'task_failed'})

        # STATE 15: Final Verify (Alle 3 Personen sitzen)
        smach.StateMachine.add('FINAL_VERIFY', StateVerifySeats(guest_manager, target_state=15), transitions={'done': 'FINISHED', 'failed': 'task_failed'})









        # ---------------------------
        
        # STATE 3: Scanning (Endlos) - aktuell nicht im Flow, aber verfügbar
        smach.StateMachine.add('SCANNING', StateScanning(), transitions={'aborted': 'task_failed'})
        
        # STATE 4: Finished (Task abgeschlossen)
        smach.StateMachine.add('FINISHED', StateFinished(), transitions={'done': 'task_complete'})
    


    
    # === Introspection Server (optional - für Visualization) ===
    try:
        sis = smach_ros.IntrospectionServer('receptionist_sm', sm, '/SM_ROOT')
        sis.start()
    except:
        rospy.logwarn("⚠️ Introspection Server konnte nicht gestartet werden")
    
    # === State Machine ausführen ===
    outcome = sm.execute()
    

    rospy.loginfo(f"🏁 State Machine beendet: {outcome}")
    



















    # === Zeige Ergebnis ===
    if outcome == 'task_complete':
        rospy.loginfo("\n📊 ENDERGEBNIS:")
        rospy.loginfo("-" * 60)
        
        # Zeige Host
        host = guest_manager.get_guest_by_role("host")
        if host:
            rospy.loginfo(f"✅ Host:")
            rospy.loginfo(f"   Name: {host['name']}")
            rospy.loginfo(f"   Drink: {host['favorite_drink']}")
            rospy.loginfo(f"   Face Samples: {len(host['face_encodings'])}")
        
        # Zeige Gast 1
        guest1 = guest_manager.get_guest_by_role("guest1")
        if guest1:
            rospy.loginfo(f"\n✅ Gast 1:")
            rospy.loginfo(f"   Name: {guest1['name']}")
            rospy.loginfo(f"   Drink: {guest1['favorite_drink']}")
            rospy.loginfo(f"   Face Samples: {len(guest1['face_encodings'])}")
        
        # Zeige Übersicht
        rospy.loginfo(f"\n📋 Übersicht - Alle registrierten Personen:")
        for guest in guest_manager.known_guests:
            rospy.loginfo(f"   - {guest['role']}: {guest['name']} ({guest['favorite_drink']}, {len(guest['face_encodings'])} Samples)")
        
        rospy.loginfo("="*60 + "\n")
    
    # Stop Introspection Server
    try:
        sis.stop()
    except:
        pass


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("State Machine unterbrochen")
