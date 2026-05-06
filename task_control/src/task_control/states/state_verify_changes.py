#!/usr/bin/env python3
"""
State: Verify Changes (State 13)
================================
Prüft, ob sich Personen (Host, Gast 1) umgesetzt haben, während der Roboter weg war.

LOGIK:
1. Lese aktuelle Sitzbelegung (/seat_status_list).
2. Vergleiche mit gespeicherten Sitzen im GuestManager.
3. Erstelle Kontext für LLM ("Host moved from 1 to 2").
4. Starte Smart Brain Conversation (Kommentar abgeben).
5. Update GuestManager mit neuer Belegung.
"""

import rospy
import smach
from messages.msg import SeatArray
from std_msgs.msg import String
from task_control.actions.conversation import Conversation

class StateVerifyChanges(smach.State):
    def __init__(self, guest_manager):
        smach.State.__init__(self, outcomes=['done', 'failed'])
        self.guest_manager = guest_manager

    def execute(self, userdata):
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("🔎 STATE 13: VERIFY SEAT CHANGES")
        rospy.loginfo("="*60)
        
        # 1. WARTESCHLEIFE: Warte bis wir Host und Gast 1 sehen
        timeout_duration = 30.0
        start_time = rospy.Time.now()
        
        rospy.loginfo("⏳ WAITING: Need both 'host' and 'guest1' to be identified on chairs before verifying changes...")
        rospy.loginfo(f"⏳ Scanne Szene (max {timeout_duration}s)...")
        
        current_seats = []
        people_found_count = 0
        
        while (rospy.Time.now() - start_time).to_sec() < timeout_duration:
            try:
                msg = rospy.wait_for_message('/seat_status_list', SeatArray, timeout=1.0)
                current_seats = msg.seats
                
                # Zähle bekannte Gesichter
                found_roles = []
                temp_found_list = []
                
                # --- NEUES LOGGING ---
                seat_debug_str = " | ".join([f"S{s.id}:{s.status[:3]} ID:{s.person_id}" for s in current_seats])
                rospy.loginfo(f"🪑 SEAT STATUS: {seat_debug_str}")
                # ---------------------

                for seat in current_seats:
                    if seat.status == 'OCCUPIED':
                        track_id = seat.person_id
                        guest = self.guest_manager.get_guest_by_track_id(track_id)
                        if guest:
                            found_roles.append(guest['role'])
                
                # Check: Haben wir Host und Guest1?
                has_host = "host" in found_roles
                has_guest1 = "guest1" in found_roles
                
                if has_host and has_guest1:
                    rospy.loginfo("✅ Host und Gast 1 erkannt!")
                    people_found_count = 2
                    break
                
                if len(found_roles) >= 2:
                     # Fallback: Zumindest 2 bekannte Leute gesehen
                     rospy.loginfo(f"✅ 2 bekannte Personen erkannt: {found_roles}")
                     people_found_count = len(found_roles)
                     break
                     
                rospy.loginfo_throttle(2, f"   ... sehe {len(found_roles)} bekannte Personen {found_roles} ...")
                
            except Exception:
                pass
                
            if rospy.is_shutdown():
                return 'failed'

        # 2. Vergleich: Alt vs Neu (Mit dem letzten Stand)
        if people_found_count < 2:
            rospy.logwarn("⚠️ Timeout! Konnte nicht alle Personen sicher wiederfinden.")
            
        changes_detected = []
        updates_to_save = [] # (role, new_seat_id)
        
        # Wir prüfen Host und Gast 1
        for role in ["host", "guest1"]:
            guest = self.guest_manager.get_guest_by_role(role)
            if not guest: continue
            
            old_seat = guest.get("current_seat", -1)
            name = guest.get("name", role)
            
            new_seat = -1
            found_in_scene = False
            
            # Finde neuen Sitz anhand Track ID
            current_track_id = guest.get("current_track_id", -1)
            
            for seat in current_seats:
                if seat.status == 'OCCUPIED' and seat.person_id == current_track_id:
                    new_seat = seat.id
                    found_in_scene = True
                    break
            
            if found_in_scene:
                updates_to_save.append((role, new_seat))
                
                if old_seat != -1 and old_seat != new_seat:
                    changes_detected.append(f"{name} moved from Chair {old_seat} to Chair {new_seat}.")
                elif old_seat == new_seat:
                    changes_detected.append(f"{name} stayed on Chair {new_seat}.")
            else:
                changes_detected.append(f"{name} is currently not seen on any chair.")

        # 3. Kontext bauen
        if not changes_detected:
            context = "No known guests seen on chairs."
        else:
            context = "Situation update: " + " ".join(changes_detected)
            
        rospy.loginfo(f"📝 Kontext für LLM: {context}")
        
        # Publish Context
        ctx_pub = rospy.Publisher('/smart_brain/scene_context', String, queue_size=1, latch=True)
        ctx_pub.publish(String(context))
        
        # 4. Smart Brain Conversation starten (State 13)
        conversation = Conversation(
            role="guest2", # Irrelevant, aber wir sind im Gast 2 Loop
            state_number=13,
            guest_manager=self.guest_manager
        )
        conversation.start()
        
        while not conversation.is_done() and not rospy.is_shutdown():
            rospy.sleep(0.5)
        conversation.stop()
        
        # 5. Speichern der neuen Positionen
        rospy.loginfo("💾 Speichere neue Sitzpositionen...")
        for role, seat_id in updates_to_save:
            self.guest_manager.update_seat(role, seat_id)
            
        return 'done'
