#!/usr/bin/env python3
"""
State: Introduce Guest
======================
State 8: Stellt den Gast dem Host vor und weist einen Platz zu (4 Stühle fix).

LOGIK:
1. Suche Host in der Sitzplatzliste (via Track ID)
2. Schlage Platz rechts vom Host vor (Host Index + 1)
3. Falls belegt oder Ende, suche nächsten freien Platz.
"""

import rospy
import smach
from messages.msg import SeatArray, Seat, GuestList
from std_msgs.msg import String # Für den Context Publisher brauchen wir es noch
from task_control.actions.conversation import Conversation
# PointTo Import entfernt - wird jetzt via smart_brain's /smart_brain/point_to_seat Topic gemacht
from collections import Counter

class StateIntroduceGuest(smach.State):
    """
    SMACH State für das Vorstellen eines Gastes.
    Kann für Gast 1 (State 8) und Gast 2 (State ? - z.B. 18) verwendet werden.
    """
    
    def __init__(self, guest_manager, target_role="guest1", target_state=8):
        smach.State.__init__(self, outcomes=['done', 'failed'])
        self.guest_manager = guest_manager
        self.target_role = target_role
        self.target_state = target_state
        #rospy.loginfo(f"[StateIntroduceGuest] Initialisiert für {self.target_role} (State {self.target_state})")

    def execute(self, userdata):
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo(f"🎩 STATE {self.target_state}: INTRODUCE GUEST ({self.target_role})")
        rospy.loginfo("="*60)
        
        # Namen des aktuellen Gastes holen
        guest_name = "the guest"
        current_guest = self.guest_manager.get_guest_by_role(self.target_role)
        if current_guest and current_guest.get("name"):
            guest_name = current_guest["name"]
        
        # 1. WARTESCHLEIFE: Warte bis Host (und ggf. Gast 1) erkannt wird
        timeout_duration = 5.0  # Reduziert von 30.0s - kurze Wartezeit
        start_time = rospy.Time.now()
        
        # Wen suchen wir?
        required_roles = ["host"]
        if self.target_role == "guest2":
            required_roles.append("guest1")
            
        rospy.loginfo(f"⏳ WAITING: Need {len(required_roles)} identified persons ({required_roles}) before generating context...")
        rospy.loginfo(f"⏳ Scanne Szene (max {timeout_duration}s)...")
        
        final_seats = []
        host_seat_idx = -1
        guest1_seat_idx = -1 # Neu für Gast 1
        
        found_roles_map = {} # role -> seat_id

        while (rospy.Time.now() - start_time).to_sec() < timeout_duration:
            try:
                msg = rospy.wait_for_message('/seat_status_list', SeatArray, timeout=1.0)
                current_seats = msg.seats
                
                # DEBUG: Zeige empfangene Sitzpositionen
                rospy.loginfo("="*60)
                rospy.loginfo("🪑 DEBUG: EMPFANGENE SITZPOSITIONEN:")
                for seat in current_seats:
                    rospy.loginfo(f"   Stuhl {seat.id}: status='{seat.status}', person_id={seat.person_id}, type='{seat.type}'")
                rospy.loginfo("="*60)
                
                # Hole auch aktuelle Gesichtsdaten
                try:
                    guest_msg = rospy.wait_for_message('/vision/guest_info', GuestList, timeout=0.5)
                except:
                    guest_msg = None
                
                # Checke jeden Sitz
                current_found = {}
                
                # DEBUG: Zeige alle bekannten Gäste mit ihren Track IDs
                rospy.loginfo("👥 DEBUG: BEKANNTE GÄSTE IM GUEST_MANAGER:")
                for guest in self.guest_manager.known_guests:
                    rospy.loginfo(f"   {guest['role']}: {guest['name']}, Track ID: {guest.get('last_track_id', 'KEINE')}")
                rospy.loginfo("="*60)
                
                for seat in current_seats:
                    if seat.status == 'OCCUPIED':
                        track_id = seat.person_id
                        rospy.loginfo(f"🔍 DEBUG: Prüfe Stuhl {seat.id} mit Track ID {track_id}...")
                        
                        # Methode 1: Direkt aus guest_manager (falls scan_callback schon geupdated hat)
                        guest = self.guest_manager.get_guest_by_track_id(track_id)
                        if guest:
                            rospy.loginfo(f"   ✅ Match gefunden: {guest['name']} ({guest['role']}) -> Stuhl {seat.id}")
                            current_found[guest['role']] = seat.id
                            continue
                        else:
                            rospy.loginfo(f"   ❌ Keine direkte Match für Track ID {track_id}")
                    else:
                        # DEBUG: Warnung wenn leerer Stuhl geprüft wird
                        rospy.loginfo(f"   ⚠️ Stuhl {seat.id} ist {seat.status}, überspringe...")
                        
                        # Methode 2: Live Face Encoding vergleichen (falls Track ID nicht passt)
                        if guest_msg:
                            rospy.loginfo(f"   🔄 Versuche Live-Gesichtserkennung für Track ID {track_id}...")
                            for g in guest_msg.guests:
                                rospy.loginfo(f"      Prüfe Guest aus Vision: Track ID {g.current_track_id}, hat Encoding: {len(g.face_encoding) > 0}")
                                if g.current_track_id == track_id and len(g.face_encoding) > 0:
                                    # Versuche zu identifizieren
                                    matched_guest, dist = self.guest_manager.identify_guest(g.face_encoding, track_id)
                                    if matched_guest:
                                        rospy.loginfo(f"   ✅ Live-Match: {matched_guest['name']} (dist={dist:.3f}) -> Stuhl {seat.id}")
                                        current_found[matched_guest['role']] = seat.id
                                        # Update Track ID im Manager
                                        self.guest_manager.update_track_id(matched_guest['role'], track_id)
                                    else:
                                        rospy.loginfo(f"   ❌ Kein Match bei Gesichtserkennung (beste Distanz zu groß)")
                                    break
                        else:
                            rospy.loginfo(f"   ⚠️ Keine Guest-Vision-Daten verfügbar für Live-Check")
                
                # Haben wir alle?
                all_found = True
                for r in required_roles:
                    if r not in current_found:
                        all_found = False
                        break
                
                # DEBUG: Zeige was gefunden wurde
                rospy.loginfo(f"📊 DEBUG: Gefundene Rollen: {current_found}")
                rospy.loginfo(f"📊 DEBUG: Benötigte Rollen: {required_roles}")
                rospy.loginfo(f"📊 DEBUG: Alle gefunden? {all_found}")
                
                if all_found:
                    rospy.loginfo(f"✅ Alle benötigten Personen gefunden: {current_found}")
                    found_roles_map = current_found
                    final_seats = current_seats
                    break
                
                # Fallback: Speichere was wir haben (falls Timeout)
                if len(current_found) > len(found_roles_map):
                    found_roles_map = current_found
                    final_seats = current_seats
                
                rospy.loginfo_throttle(2, f"   ... gefunden: {list(current_found.keys())} / benötigt: {required_roles} ...")
                
            except Exception as e:
                rospy.logdebug(f"Exception in scan loop: {e}")
                pass
            
            if rospy.is_shutdown():
                return 'failed'

        # 2. Auswertung nach der Schleife
        if len(found_roles_map) < len(required_roles):
            rospy.logwarn(f"⚠️ Timeout! Nicht alle Personen gefunden. Habe: {found_roles_map}")
            # Wir nehmen trotzdem den letzten Snapshot
            if not final_seats:
                 try:
                    msg = rospy.wait_for_message('/seat_status_list', SeatArray, timeout=1.0)
                    final_seats = msg.seats
                 except: pass

        # Daten extrahieren
        host_seat_idx = found_roles_map.get("host", -1)
        guest1_seat_idx = found_roles_map.get("guest1", -1)
        
        # Namen holen
        host_name = "Host"
        h = self.guest_manager.get_guest_by_role("host")
        if h: host_name = h["name"]
        
        guest1_name = "Guest 1"
        g1 = self.guest_manager.get_guest_by_role("guest1")
        if g1: guest1_name = g1["name"]

        # 3. Besetzte Plätze markieren (für Platzwahl)
        occupied_indices = []
        for seat in final_seats:
             if seat.status == 'OCCUPIED':
                 occupied_indices.append(seat.id)

        if host_seat_idx == -1:
             host_seat_idx = 0 # Fallback für Prompt

        # 4. Ziel-Platz berechnen (Logik: Rechts vom Host, sonst irgendwo frei)
        target_seat = -1
        
        # Versuche Platz rechts vom Host (idx + 1)
        ideal_seat = host_seat_idx + 1
        
        # Sicherheitscheck: Sitz <= 3 und nicht belegt (3 Stühle!)
        if ideal_seat <= 3 and ideal_seat not in occupied_indices:
            target_seat = ideal_seat
            rospy.loginfo(f"🎯 Vorschlag: Platz {target_seat} (rechts vom Host)")
        else:
            # Suche irgendeinen anderen freien Platz (1-3)
            for s in [1, 2, 3]:
                if s not in occupied_indices:
                    target_seat = s
                    rospy.loginfo(f"🎯 Vorschlag: Platz {target_seat} (nächster freier)")
                    break
        
        if target_seat == -1:
            rospy.logerr("❌ Alle Stühle belegt!")
            target_seat = 1 # Not-Fallback

        # 5. Prompt für Smart Brain bauen
        # Wir nutzen strikt Nummern (Chair 1 - Chair 3)
        scene_info = ""
        if host_seat_idx > 0:
            scene_info += f"Host ({host_name}) is sitting on Chair {host_seat_idx}. "
        else:
            scene_info += f"Host ({host_name}) is currently standing or not visible. "
            
        if guest1_seat_idx > 0:
            scene_info += f"First Guest ({guest1_name}) is sitting on Chair {guest1_seat_idx}. "
            
        scene_info += f"The calculated best seat for {guest_name} ({self.target_role}) is Chair {target_seat}. "
        scene_info += "The chairs are numbered 1 to 3 from left to right."
        
        rospy.loginfo(f"📝 Kontext für LLM: {scene_info}")

        # 6. Information an Smart Brain publishen (via Context Topic)
        ctx_pub = rospy.Publisher('/smart_brain/scene_context', String, queue_size=1, latch=True)
        ctx_pub.publish(String(scene_info))
        
        # Pointing wird jetzt von smart_brain.py gemacht (parsed SEAT: und sendet /smart_brain/point_to_seat)
        
        # 7. Conversation starten (State 8 = Introduce)
        conversation = Conversation(
            role=self.target_role,
            state_number=self.target_state,
            guest_manager=self.guest_manager
        )
        conversation.start()
        
        while not conversation.is_done() and not rospy.is_shutdown():
            rospy.sleep(0.5)
            
        conversation.stop()
        
        rospy.loginfo("✅ Vorstellung beendet.")
        return 'done'

