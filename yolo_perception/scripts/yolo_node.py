#!/usr/bin/env python3
import os
import rospy
import cv2
import time
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from messages.msg import GuestInfo, GuestList, PersonPose
from yolo_perception.color_classifier import ColorClassifier
from deepface import DeepFace
import tensorflow as tf
import torch

class YoloDetector:
    def __init__(self):
        rospy.init_node('yolo_pose_detector', anonymous=True)

        # --- 1. TENSORFLOW GPU CONFIG ---
        gpus = tf.config.list_physical_devices('GPU')
        if gpus:
            try:
                # Prevent TF from pre-allocating all VRAM so PyTorch has space
                for gpu in gpus:
                    tf.config.experimental.set_memory_growth(gpu, True)
                rospy.loginfo(f"CONFIG: TensorFlow GPU found. Device count: {len(gpus)}")
            except RuntimeError as e:
                rospy.logerr(f"CONFIG ERROR: TF Memory Growth failed: {e}")
        else:
            rospy.logwarn("CONFIG WARNING: TensorFlow is NOT using GPU!")

        # --- 2. YOLO MODEL LOADING & GPU FORCE ---
        model_name = rospy.get_param('~model_name', 'yolo11m-pose.pt')
        self.conf_threshold = rospy.get_param('~confidence', 0.5)

        model_path = model_name
        if not os.path.isabs(model_name):
            script_dir = os.path.dirname(os.path.abspath(__file__))
            candidate = os.path.join(script_dir, model_name)
            if os.path.exists(candidate):
                model_path = candidate

        rospy.loginfo(f"Loading YOLO model: {model_path}...")
        self.model = YOLO(model_path)
        
        # Explicitly move YOLO to GPU (cuda:0)
        if torch.cuda.is_available():
            self.model.to('cuda')
            rospy.loginfo(f"CONFIG: YOLO explicitly moved to: {self.model.device}")
        else:
            rospy.logwarn("CONFIG WARNING: PyTorch CUDA not available. YOLO will run on CPU.")

        # --- 3. PERFORMANCE SETTINGS ---
        self.process_face_every_n_frames = 5  
        self.publish_debug_every_n_frames = 2
        self.frame_counter = 0

        self.bridge = CvBridge()
        self.classifier = ColorClassifier()

        # --- 4. DEEPFACE WARMUP ---
        rospy.loginfo("Warming up DeepFace (Backend: ssd)...")
        dummy_img = np.zeros((224, 224, 3), dtype=np.uint8)
        try:
            DeepFace.represent(img_path=dummy_img, model_name="ArcFace", detector_backend="ssd", enforce_detection=False)
            rospy.loginfo("CONFIG: DeepFace/TensorFlow Backend ready.")
        except Exception as e:
            rospy.logerr(f"CONFIG ERROR: DeepFace warmup failed: {e}")

        # --- 5. ROS INFRASTRUCTURE ---
        input_image_topic = rospy.get_param('~input_image_topic', '/xtion/rgb/image_raw')
        self.image_sub = rospy.Subscriber(input_image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        self.detection_pub = rospy.Publisher('/yolo/detections', Detection2DArray, queue_size=1)
        self.debug_image_pub = rospy.Publisher('/yolo/debug_image', Image, queue_size=1)
        self.face_crop_pub = rospy.Publisher('/yolo/face_crop', Image, queue_size=1)
        self.guest_pub = rospy.Publisher('/vision/guest_info', GuestList, queue_size=1)
        self.pose_pub = rospy.Publisher('/vision/person_poses', PersonPose, queue_size=20)

    def image_callback(self, msg):
        loop_start = time.time()
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError: return

        self.frame_counter += 1
        do_face = (self.frame_counter % self.process_face_every_n_frames == 0)
        do_debug = (self.frame_counter % self.publish_debug_every_n_frames == 0)

        # --- PHASE 1: YOLO INFERENCE ---
        t_yolo_start = time.time()
        # Ensure 'device=0' is passed to track to confirm GPU usage
        results = self.model.track(cv_image, verbose=False, conf=self.conf_threshold, persist=True, device=0)
        t_yolo = (time.time() - t_yolo_start) * 1000

        # --- PHASE 2: FACE REC & GUEST INFO ---
        t_face_start = time.time()
        self.publish_guest_info(results, msg.header, cv_image, do_face)
        t_face = (time.time() - t_face_start) * 1000

        # --- PHASE 3: MISC PUBLISHING ---
        self.publish_detections(results, msg.header)
        self.publish_raw_poses(results, msg.header)

        if do_debug:
            self.publish_debug_image(results)

        # --- PHASE 4: DIAGNOSTICS ---
        loop_duration = (time.time() - loop_start) * 1000
        
        # Log every 10 frames to monitor performance health
        if self.frame_counter % 10 == 0:
            status = "HEALTHY" if loop_duration < 100 else "LAGGING"
            rospy.loginfo(f"PERF: Loop: {loop_duration:.1f}ms | YOLO: {t_yolo:.1f}ms | DeepFace({do_face}): {t_face:.1f}ms | Target: 10FPS -> {status}")

    def check_frontal_debug(self, keypoints, box_width, track_id):
        """
        Validates if person is looking at camera using pose keypoints.
        """
        if keypoints is None or len(keypoints) == 0: return False, "No Kps"
        kps = keypoints.cpu().numpy()
        nose = kps[0]; l_eye = kps[1]; r_eye = kps[2]; l_ear = kps[3]; r_ear = kps[4]
        conf = 0.6
        vis_nose = nose[2] > conf
        vis_eyes = l_eye[2] > conf and r_eye[2] > conf
        vis_ears = l_ear[2] > conf and r_ear[2] > conf

        if vis_nose and vis_eyes and vis_ears: return True, f"FACE: [ID {track_id}] POSE: FRONTAL (PERFECT)"
        if vis_nose and vis_eyes:
            dist_left = abs(nose[0] - l_eye[0])
            dist_right = abs(nose[0] - r_eye[0])
            if dist_right == 0: dist_right = 0.001
            ratio = dist_left / dist_right
            is_sym = 0.6 < ratio < 1.6
            res = "FRONTAL" if is_sym else "SIDE"
            return is_sym, f"FACE: [ID {track_id}] POSE: {res} (Ratio: {ratio:.2f})"
        return False, f"FACE: [ID {track_id}] POSE: INVALID (Side/Hidden)"

    def get_smart_face_crop(self, full_image, keypoints, person_box):
        """
        Uses keypoints to generate a stable, nose-centered face crop.
        """
        img_h, img_w = full_image.shape[:2]
        x1, y1, x2, y2 = map(int, person_box)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(img_w, x2), min(img_h, y2)
        person_h = y2 - y1
        fallback_crop = full_image[y1:y1+int(person_h*0.4), x1:x2]

        if keypoints is None or len(keypoints) == 0: return fallback_crop
        kps = keypoints.cpu().numpy()
        nose = kps[0]
        if nose[2] < 0.5: return fallback_crop
        head_kps = kps[0:5]
        visible_kps = head_kps[head_kps[:, 2] > 0.5]
        if len(visible_kps) < 2: return fallback_crop

        min_x = np.min(visible_kps[:, 0]); max_x = np.max(visible_kps[:, 0])
        face_w = max_x - min_x
        if face_w < 20: return fallback_crop

        nose_x = int(nose[0]); nose_y = int(nose[1])
        pad_up = face_w * 0.75; pad_down = face_w * 0.75; pad_side = face_w * 0.2

        crop_x1 = int(max(0, nose_x - (face_w/2) - pad_side))
        crop_x2 = int(min(img_w, nose_x + (face_w/2) + pad_side))
        crop_y1 = int(max(0, nose_y - pad_up))
        crop_y2 = int(min(img_h, nose_y + pad_down))

        if (crop_x2 - crop_x1) < 30 or (crop_y2 - crop_y1) < 30: return fallback_crop
        return full_image[crop_y1:crop_y2, crop_x1:crop_x2]

    def publish_raw_poses(self, results, header):
        """
        Sends raw pose data for downstream tasks.
        """
        result = results[0]
        if result.boxes is None: return
        keypoints_data = None
        if result.keypoints is not None: keypoints_data = result.keypoints.data

        for i, box in enumerate(result.boxes):
            if int(box.cls[0].cpu().numpy()) != 0: continue
            track_id = int(box.id[0].cpu().numpy()) if box.id is not None else -1
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            
            pose_msg = PersonPose()
            pose_msg.header = header
            pose_msg.track_id = track_id
            pose_msg.bbox.center.x = x1 + (x2 - x1) / 2.0
            pose_msg.bbox.center.y = y1 + (y2 - y1) / 2.0
            pose_msg.bbox.size_x = x2 - x1
            pose_msg.bbox.size_y = y2 - y1

            if keypoints_data is not None and len(keypoints_data) > i:
                pose_msg.keypoints = keypoints_data[i].cpu().numpy().flatten().tolist()
            else:
                pose_msg.keypoints = []
            self.pose_pub.publish(pose_msg)

    def publish_guest_info(self, results, header, original_image, do_face_recognition):
        """
        Extracts face vectors for identified people.
        """
        guest_list = GuestList()
        guest_list.header = header
        result = results[0]
        if result.boxes is None:
            self.guest_pub.publish(guest_list)
            return

        keypoints_data = None
        if result.keypoints is not None: keypoints_data = result.keypoints.data

        for i, box in enumerate(result.boxes):
            if int(box.cls[0].cpu().numpy()) != 0: continue
            track_id = int(box.id[0].cpu().numpy()) if box.id is not None else -1
            box_coords = box.xyxy[0].cpu().numpy()
            box_w = box_coords[2] - box_coords[0]

            guest = GuestInfo()
            guest.current_track_id = track_id
            guest.shirt_color = "unknown"
            guest.pants_color = "unknown"

            is_frontal = False
            kps_single = None
            if keypoints_data is not None and len(keypoints_data) > i:
                kps_single = keypoints_data[i]
                is_frontal, debug_str = self.check_frontal_debug(kps_single, box_w, track_id)
                # Print debug info to terminal periodically
                if self.frame_counter % 15 == 0:
                    rospy.loginfo(debug_str)

            face_encoding = []
            if do_face_recognition and is_frontal:
                try:
                    face_crop = self.get_smart_face_crop(original_image, kps_single, box_coords)
                    h_crop, w_crop = face_crop.shape[:2]

                    # Store face crop dimensions in the GuestInfo message
                    guest.face_width = w_crop
                    guest.face_height = h_crop

                    # Relaxed minimum size check: only discard extremely small faces
                    if h_crop < 30 or w_crop < 30:
                        rospy.logwarn_throttle(5.0, f"Face way too small for ID {track_id}: {w_crop}x{h_crop}")
                        continue

                    try:
                        self.face_crop_pub.publish(self.bridge.cv2_to_imgmsg(face_crop, "bgr8"))
                    except:
                        pass

                    # Optimized resizing
                    img_rgb = cv2.cvtColor(face_crop, cv2.COLOR_BGR2RGB)
                    if h_crop > 160:
                        img_rgb = cv2.resize(img_rgb, (160, 160))

                    face_objs = DeepFace.represent(
                        img_path=img_rgb,
                        model_name="ArcFace",
                        detector_backend="ssd",
                        enforce_detection=True,
                        align=True,
                    )
                    if len(face_objs) > 0:
                        face_encoding = face_objs[0]["embedding"]
                except Exception:
                    pass

            if len(face_encoding) > 0:
                guest.face_encoding = face_encoding
            guest_list.guests.append(guest)
        self.guest_pub.publish(guest_list)

    def publish_detections(self, results, header):
        detection_msg = Detection2DArray()
        detection_msg.header = header
        result = results[0]
        if result.boxes is None:
            self.detection_pub.publish(detection_msg)
            return
        for box in result.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls_id = int(box.cls[0].cpu().numpy())
            detection = Detection2D()
            detection.header = header
            detection.bbox.center.x = x1 + (x2 - x1) / 2.0
            detection.bbox.center.y = y1 + (y2 - y1) / 2.0
            detection.bbox.size_x = x2 - x1
            detection.bbox.size_y = y2 - y1
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = cls_id; hypothesis.score = float(box.conf[0].cpu().numpy())
            detection.results.append(hypothesis); detection_msg.detections.append(detection)
        self.detection_pub.publish(detection_msg)

    def publish_debug_image(self, results):
        if self.debug_image_pub.get_num_connections() > 0:
            plotted_image = results[0].plot()
            try: self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(plotted_image, "bgr8"))
            except: pass

if __name__ == '__main__':
    try: YoloDetector(); rospy.spin()
    except rospy.ROSInterruptException: pass
