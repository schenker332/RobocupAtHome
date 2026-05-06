#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from collections import deque
from ultralytics import YOLO
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# --- IMPORTE DER NEUEN MESSAGES ---
from messages.msg import PersonPose, Seat, SeatArray 

class TrackedChair:
    def __init__(self, box, update_interval=30):
        self.update_interval = update_interval
        self.counter = 0 
        
        self.detection_buffer = deque([1], maxlen=update_interval) 
        self.box_buffer = deque([box], maxlen=update_interval)      
        self.occupancy_buffer = deque([False], maxlen=update_interval) 
        
        self.last_seen_time = rospy.Time.now()
        
        self.visual_box = box 
        self.visual_occupancy = False
        self.visual_occupant_id = -1
        self.current_frame_occupant = -1

    def update(self, box):
        self.detection_buffer.append(1)
        self.box_buffer.append(box)
        self.last_seen_time = rospy.Time.now()

    def miss(self):
        self.detection_buffer.append(0)

    def set_occupancy_vote(self, is_occupied, person_id):
        self.occupancy_buffer.append(is_occupied)
        if is_occupied:
            self.current_frame_occupant = person_id

    def tick(self):
        self.counter += 1
        if self.counter >= self.update_interval:
            if len(self.box_buffer) > 0:
                self.visual_box = np.mean(self.box_buffer, axis=0).astype(int)
            
            votes_occupied = self.occupancy_buffer.count(True)
            self.visual_occupancy = (votes_occupied >= 25)
            
            if self.visual_occupancy:
                self.visual_occupant_id = self.current_frame_occupant
            else:
                self.visual_occupant_id = -1
            
            self.counter = 0

    @property
    def is_confirmed(self):
        return self.detection_buffer.count(1) >= 20

    @property
    def center_x(self):
        return int((self.visual_box[0] + self.visual_box[2]) / 2)


class SeatDetectorNode:
    def __init__(self):
        rospy.init_node('seat_detector_final', anonymous=True)

        rospy.loginfo("Lade Modelle...")
        self.model_chairs = YOLO('yolo11m.pt')
        self.bridge = CvBridge()
        
        self.pose_buffer = {}
        self.person_history = {}
        self.tracked_chairs = []
        
        self.pose_sub = rospy.Subscriber('/vision/person_poses', PersonPose, self.pose_callback, queue_size=1)
        self.image_sub = rospy.Subscriber('/xtion/rgb/image_raw', Image, self.image_callback, queue_size=1)
        
        self.debug_pub = rospy.Publisher('/yolo/seat_analysis', Image, queue_size=1)
        self.seat_pub = rospy.Publisher('/seat_status_list', SeatArray, queue_size=1)
        
        self.history_size = 30    
        self.threshold_count = 25 
        
        self.last_print_time = rospy.Time(0)
        
        rospy.loginfo(f"SYSTEM BEREIT. Priorisierung: Virtuelle Stühle > Reale Stühle.")

    def pose_callback(self, msg):
        self.pose_buffer[msg.track_id] = (msg, rospy.Time.now())

    def calculate_inclusion_ratio(self, inner_box, outer_box):
        """
        Berechnet, wie viel Prozent der inner_box (realer Stuhl) 
        innerhalb der outer_box (virtueller Stuhl/Person) liegen.
        """
        ix1, iy1, ix2, iy2 = inner_box
        ox1, oy1, ox2, oy2 = outer_box

        # Schnittmenge berechnen
        x_left = max(ix1, ox1)
        y_top = max(iy1, oy1)
        x_right = min(ix2, ox2)
        y_bottom = min(iy2, oy2)

        if x_right < x_left or y_bottom < y_top:
            return 0.0

        intersection_area = (x_right - x_left) * (y_bottom - y_top)
        inner_area = (ix2 - ix1) * (iy2 - iy1)

        if inner_area == 0: return 0.0
        
        return intersection_area / inner_area

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError: return

        now = rospy.Time.now()

        # 1. PERSONEN MANAGEMENT
        active_people_msgs = []
        TIMEOUT_SEC = 0.8  
        for pid in list(self.pose_buffer.keys()):
            pose_msg, timestamp = self.pose_buffer[pid]
            if (now - timestamp).to_sec() > TIMEOUT_SEC:
                del self.pose_buffer[pid]
                if pid in self.person_history: del self.person_history[pid]
            else:
                active_people_msgs.append(pose_msg)

        valid_people = []
        for person in active_people_msgs:
            if len(person.keypoints) >= 39:
                c_s1, c_s2 = person.keypoints[17], person.keypoints[20]
                c_h1, c_h2 = person.keypoints[35], person.keypoints[38]
                if (c_s1 + c_s2 + c_h1 + c_h2) / 4.0 < 0.5: continue
            
            valid_people.append(person)
            pid = person.track_id
            
            if pid not in self.person_history:
                self.person_history[pid] = {
                    'thigh_incs': deque(maxlen=15), 
                    'ratios': deque(maxlen=15),     
                    'state_buffer': deque(maxlen=self.history_size),
                    'state': 'STANDING' 
                }
            
            hist = self.person_history[pid]
            t_inc, _, _ = self.calculate_absolute_inclinations(person.keypoints)
            hist['thigh_incs'].append(t_inc)
            avg_t = np.mean(hist['thigh_incs'])

            frontal_ratio = self.calculate_frontal_ratio(person.keypoints)
            hist['ratios'].append(frontal_ratio)
            avg_ratio = np.mean(hist['ratios'])
            
            is_sitting_now = (avg_t > 50.0) or (0.1 < avg_ratio < 0.65)
            hist['state_buffer'].append('SITTING' if is_sitting_now else 'STANDING')
            
            votes_sit = hist['state_buffer'].count('SITTING')
            votes_stand = hist['state_buffer'].count('STANDING')
            
            if hist['state'] == 'STANDING' and votes_sit >= self.threshold_count:
                hist['state'] = 'SITTING'
            elif hist['state'] == 'SITTING' and votes_stand >= self.threshold_count:
                hist['state'] = 'STANDING'

        # 2. STUHL TRACKING (YOLO)
        chair_results = self.model_chairs(cv_image, verbose=False, conf=0.4, classes=[56])
        current_frame_boxes = []
        if chair_results[0].boxes is not None:
            for box in chair_results[0].boxes:
                current_frame_boxes.append(box.xyxy[0].cpu().numpy().astype(int))

        used_new_indices = set()
        for tracker in self.tracked_chairs:
            best_idx = -1
            min_dist = 10000 
            tx1, ty1, tx2, ty2 = tracker.visual_box
            t_cx, t_cy = (tx1+tx2)/2, (ty1+ty2)/2

            for i, new_box in enumerate(current_frame_boxes):
                if i in used_new_indices: continue
                nx1, ny1, nx2, ny2 = new_box
                n_cx, n_cy = (nx1+nx2)/2, (ny1+ny2)/2
                dist = np.sqrt((t_cx - n_cx)**2 + (t_cy - n_cy)**2)
                
                if dist < 120 and dist < min_dist:
                    min_dist = dist
                    best_idx = i
            
            if best_idx != -1:
                tracker.update(current_frame_boxes[best_idx])
                used_new_indices.add(best_idx)
            else:
                tracker.miss()

        for i, new_box in enumerate(current_frame_boxes):
            if i not in used_new_indices:
                self.tracked_chairs.append(TrackedChair(new_box, update_interval=self.history_size))

        # 3. INTERNES UPDATE DER TRACKER
        # Wir lassen die Tracker ticken, aber die Occupancy-Logik für reale Stühle
        # ist jetzt weniger wichtig, da wir sie eh löschen, wenn jemand drauf sitzt.
        # Wir lassen es trotzdem laufen für den Fall "Person steht noch davor".
        for tracker in self.tracked_chairs:
            tracker.tick()

        # ---------------------------------------------------------
        # 4. DATEN AUFBEREITUNG & FILTER LOGIK
        # ---------------------------------------------------------
        temp_seat_list = []
        
        # A) ERSTELLE IMMER VIRTUAL CHAIRS FÜR SITZENDE PERSONEN
        virtual_chair_boxes = [] # Speichern für den Vergleich später

        for person in valid_people:
            pid = person.track_id
            state = self.person_history[pid]['state']
            
            if state == 'SITTING':
                px1, py1, px2, py2 = self.get_rect(person.bbox)
                p_cx = int((px1 + px2) / 2)
                
                # Wir merken uns die Box für den Vergleich
                virtual_box = [px1, py1, px2, py2]
                virtual_chair_boxes.append(virtual_box)

                temp_seat_list.append({
                    'type': 'virtual',
                    'x_center': p_cx,
                    'status': 'OCCUPIED',
                    'person_id': pid,
                    'box': [px1, py1+50, px2, py2] # Visual Box etwas verschoben
                })

        # B) FILTERE REALE STÜHLE (Das 80% Kriterium)
        visible_chairs = [t for t in self.tracked_chairs if t.is_confirmed]
        
        for tracker in visible_chairs:
            real_box = tracker.visual_box
            should_remove = False

            # Prüfe gegen alle virtuellen Stühle
            for v_box in virtual_chair_boxes:
                ratio = self.calculate_inclusion_ratio(real_box, v_box)
                # Wenn der reale Stuhl zu >= 80% in der virtuellen Box (Person) liegt
                if ratio >= 0.80:
                    should_remove = True
                    break # Ein Treffer reicht zum Löschen
            
            if not should_remove:
                # Nur hinzufügen, wenn NICHT von einem virtuellen Stuhl überdeckt
                # Status basiert hier nur auf Tracker-Historie oder default EMPTY
                # Da wir virtuelle Stühle priorisieren, ist ein realer Stuhl hier meist EMPTY,
                # es sei denn, die Person steht gerade erst auf oder detection flackert.
                # Wir setzen ihn auf EMPTY oder nehmen den alten Tracker Status,
                # aber sicherheitshalber EMPTY, da SITTING ja jetzt virtual ist.
                
                temp_seat_list.append({
                    'type': 'real',
                    'x_center': tracker.center_x,
                    'status': 'EMPTY', # Da wir "Besetzt" jetzt über Virtual lösen -> Real ist Empty
                    'person_id': -1,
                    'box': tracker.visual_box
                })

        # C) Sortieren
        temp_seat_list.sort(key=lambda s: s['x_center'])

        # D) MSG BEFÜLLEN
        seat_array_msg = SeatArray()
        seat_array_msg.header.stamp = rospy.Time.now()
        seat_array_msg.header.frame_id = "camera_link"
        
        console_output = "\n=== AKTUELLER STUHL STATUS (Links -> Rechts) ===\n"
        
        for i, seat_data in enumerate(temp_seat_list):
            s_msg = Seat()
            s_msg.id = i + 1              
            s_msg.status = seat_data['status']
            s_msg.person_id = seat_data['person_id']
            s_msg.type = seat_data['type']
            
            seat_array_msg.seats.append(s_msg)
            
            p_str = f"Person {s_msg.person_id}" if s_msg.person_id != -1 else "---"
            status_symbol = "[X]" if s_msg.status == "OCCUPIED" else "[ ]"
            console_output += f"Stuhl {s_msg.id}: {status_symbol} {s_msg.status:<8} | {p_str:<10} | Typ: {s_msg.type}\n"
            
        # E) PUBLISH
        self.seat_pub.publish(seat_array_msg)
        
        # F) PRINT
        if (now - self.last_print_time).to_sec() > 2.0:
            print(console_output)
            self.last_print_time = now

        # ---------------------------------------------------------
        # 5. VISUALISIERUNG
        # ---------------------------------------------------------
        self.draw_dashboard(cv_image, len(temp_seat_list), len(valid_people))

        for i, seat in enumerate(temp_seat_list):
            cx1, cy1, cx2, cy2 = seat['box']
            if seat['status'] == 'OCCUPIED':
                color = (0, 0, 255)
                text = f"S{i+1}: OCC (ID:{seat['person_id']})"
            else:
                color = (0, 255, 0)
                text = f"S{i+1}: EMPTY"
            
            cv2.rectangle(cv_image, (cx1, cy1), (cx2, cy2), color, 3)
    
            cv2.rectangle(cv_image, (cx1, cy1-25), (cx1+220, cy1), color, -1)
            cv2.putText(cv_image, text, (cx1+5, cy1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        
        for person in valid_people:
            self.draw_skeleton(cv_image, person.keypoints)
            pid = person.track_id
            state = self.person_history[pid]['state']
            px1, py1, px2, py2 = self.get_rect(person.bbox)
            color = (255, 100, 0) if state == 'STANDING' else (0, 165, 255)
            # Nur Box malen, wenn stehend, sonst verwirrend mit Virtual Chair Box
            if state == 'STANDING':
                cv2.rectangle(cv_image, (px1, py1), (px2, py2), color, 1)
            
            cv2.putText(cv_image, f"ID:{pid} {state}", (px1, py1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    # --- HELPER (unverändert) ---
    def calculate_frontal_ratio(self, kps):
        def gp(i): 
            if i*3+2 >= len(kps) or kps[i*3+2] < 0.5: return None
            return kps[i*3+1] 
        s_y = gp(5); h_y = gp(11); k_y = gp(13)
        if s_y is None or h_y is None or k_y is None:
            s_y = gp(6); h_y = gp(12); k_y = gp(14)  
        if s_y is None or h_y is None or k_y is None: return 1.0
        torso_len = abs(h_y - s_y)
        thigh_len = abs(k_y - h_y)
        if torso_len == 0: return 1.0
        return thigh_len / torso_len

    def get_hip_center(self, kps):
        def gp(i): 
            base = i * 3
            if base+2 >= len(kps) or kps[base+2] < 0.5: return None
            return np.array([kps[base], kps[base+1]])
        h_l, h_r = gp(11), gp(12)
        if h_l is not None and h_r is not None:
            return int((h_l[0] + h_r[0]) / 2), int((h_l[1] + h_r[1]) / 2)
        elif h_l is not None: return int(h_l[0]), int(h_l[1])
        elif h_r is not None: return int(h_r[0]), int(h_r[1])
        return -1, -1

    def calculate_absolute_inclinations(self, kps):
        def gp(i): 
            base = i * 3
            if base+2 >= len(kps) or kps[base+2] < 0.5: return None
            return np.array([kps[base], kps[base+1]])
        def get_angle_vertical(p_top, p_bottom):
            if p_top is None or p_bottom is None: return 0.0
            dx = p_bottom[0] - p_top[0]; dy = p_bottom[1] - p_top[1] 
            return np.abs(np.degrees(np.arctan2(dx, dy)))
        t_l = get_angle_vertical(gp(11), gp(13))
        t_r = get_angle_vertical(gp(12), gp(14))
        return max(t_l, t_r), 0, 0

    def draw_dashboard(self, img, n_chairs, n_people):
        cv2.rectangle(img, (0, 0), (250, 80), (0, 0, 0), -1)
        cv2.putText(img, f"Detected Seats: {n_chairs}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.putText(img, f"People: {n_people}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

    def draw_skeleton(self, img, kps):
        def gp(idx): return (int(kps[idx*3]), int(kps[idx*3+1])), kps[idx*3+2]
        p5, c5 = gp(5); p6, c6 = gp(6); p11, c11 = gp(11); p12, c12 = gp(12)
        if c5>0.5 and c11>0.5: cv2.line(img, p5, p11, (255,0,255), 2)
        if c6>0.5 and c12>0.5: cv2.line(img, p6, p12, (255,0,255), 2)
        if c11>0.5 and c12>0.5: cv2.line(img, p11, p12, (255,0,255), 2)
        if c5>0.5 and c6>0.5: cv2.line(img, p5, p6, (255,0,255), 2)
        cyan = (255, 255, 0)
        p13, c13 = gp(13); p15, c15 = gp(15)
        p14, c14 = gp(14); p16, c16 = gp(16)
        if c11>0.5 and c13>0.5: cv2.line(img, p11, p13, cyan, 4)
        if c13>0.5 and c15>0.5: cv2.line(img, p13, p15, (255,0,255), 2)
        if c12>0.5 and c14>0.5: cv2.line(img, p12, p14, cyan, 4)
        if c14>0.5 and c16>0.5: cv2.line(img, p14, p16, (255,0,255), 2)

    def get_rect(self, bbox):
        cx, cy, w, h = bbox.center.x, bbox.center.y, bbox.size_x, bbox.size_y
        return int(cx-w/2), int(cy-h/2), int(cx+w/2), int(cy+h/2)

if __name__ == '__main__':
    SeatDetectorNode()
    rospy.spin()