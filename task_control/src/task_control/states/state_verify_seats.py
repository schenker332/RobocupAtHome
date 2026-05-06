#!/usr/bin/env python3
"""
State: Verify Seats
===================
State 9: Überprüft, wer sich wo hingesetzt hat und speichert dies im Gedächtnis.

ABLAUF:
1. Wartezeit (10s)
2. Snapshot der Sitzplätze
3. Identifiziere Personen (via Track-ID)
4. Speichere Sitzplätze im GuestManager
5. Bestätige via Smart Brain (State 4)
"""

import rospy
import smach
from messages.msg import SeatArray
from std_msgs.msg import String
from task_control.actions.conversation import Conversation

class StateVerifySeats(smach.State):
    def __init__(self, guest_manager, target_state=9):
        smach.State.__init__(self, outcomes=['done', 'failed'])
        self.guest_manager = guest_manager
        self.target_state = target_state

    def execute(self, userdata):
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo(f"🪑 STATE {self.target_state}: VERIFY SEATS")
        rospy.loginfo("="*60)
        
        # 1. WARTESCHLEIFE: Warte bis wir die Leute sehen
        timeout_duration = 30.0  # 30 Sekunden Wartezeit
        start_time = rospy.Time.now()
        
        # In State 15 erwarten wir 3 Leute, sonst 2
        needed_count = 3 if self.target_state == 15 else 2
        
        rospy.loginfo(f"⏳ WAITING: Need {needed_count} persons identified on chairs before generating context...")
        rospy.loginfo(f"⏳ Scanne Szene (max {timeout_duration}s)...")
        
        current_seats = []
        found_people_list = []
        
        while (rospy.Time.now() - start_time).to_sec() < timeout_duration:
            try:
                msg = rospy.wait_for_message('/seat_status_list', SeatArray, timeout=1.0)
                current_seats = msg.seats
                
                # Zähle bekannte Gesichter
                temp_found_list = []
                found_roles = []
                
                for seat in current_seats:
                    if seat.status == 'OCCUPIED':
                        track_id = seat.person_id
                        guest = self.guest_manager.get_guest_by_track_id(track_id)
                        if guest:
                            found_roles.append(guest['role'])
                            temp_found_list.append(f"{guest['name']} on Chair {seat.id}")
                            self.guest_manager.update_seat(guest['role'], seat.id)
                
                if len(found_roles) >= needed_count:
                    rospy.loginfo(f"✅ {len(found_roles)} Personen erkannt!")
                    found_people_list = temp_found_list
                    break
                     
                rospy.loginfo_throttle(2, f"   ... sehe {len(found_roles)}/{needed_count} bekannte Personen ...")
                
            except Exception:
                pass
                
            if rospy.is_shutdown():
                return 'failed'

        # 3. Kontext für Smart Brain bauen
        if found_people_list:
            context = "Final seating check: " + ", ".join(found_people_list) + "."
        else:
            context = "I cannot see anyone seated right now."
            
        rospy.loginfo(f"📝 Report: {context}")
        
        # Sende an Smart Brain
        ctx_pub = rospy.Publisher('/smart_brain/scene_context', String, queue_size=1, latch=True)
        ctx_pub.publish(String(context))
        
        # 5. Kurze Bestätigung via LLM
        conversation = Conversation(
            role="host",
            state_number=self.target_state,
            guest_manager=self.guest_manager
        )
        conversation.start()
        
        while not conversation.is_done() and not rospy.is_shutdown():
            rospy.sleep(0.5)
            
        conversation.stop()
        
        return 'done'
