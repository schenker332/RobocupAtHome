#!/home/niclas/robocup/PR/venv39/bin/python3
import rospy
import cv2
import numpy as np
import base64
import threading
import openai
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from guest_stylist.srv import AnalyzeSnapshot, AnalyzeSnapshotResponse

# --- CONFIG ---
API_KEY = "" 
MODEL = "gpt-4o-mini"  # Vision-fähiges Chat-Modell (gpt-image-1 ist NUR für Bild-GENERIERUNG!)

class StylistNode:
    def __init__(self):
        rospy.init_node('guest_stylist_node')
        
        self.client = openai.OpenAI(api_key=API_KEY)
        self.bridge = CvBridge()
        self.last_image = None
        self.lock = threading.Lock()
        
        # Subscriber
        img_topic = rospy.get_param('~image_topic', '/xtion/rgb/image_raw')
        self.sub = rospy.Subscriber(img_topic, Image, self.img_cb, queue_size=1)
        
        # Service
        self.srv = rospy.Service('/guest_stylist/analyze', AnalyzeSnapshot, self.handle_req)
        
        rospy.loginfo(f"🤵 Guest Stylist ready. Watching {img_topic}")

    def img_cb(self, msg):
        try:
            # Convert to CV2
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.last_image = cv_img
        except Exception as e:
            pass # Keep silent to avoid spam

    def handle_req(self, req):
        rospy.loginfo("Stylist: Taking a look at the guest...")
        
        target = None
        
        # 1. Bild auswählen
        if req.image.width > 0:
            try:
                target = self.bridge.imgmsg_to_cv2(req.image, "bgr8")
            except: 
                return AnalyzeSnapshotResponse(description="Error: Invalid image in request.")
        else:
            with self.lock:
                if self.last_image is None:
                    return AnalyzeSnapshotResponse(description="Error: No camera image available.")
                target = self.last_image.copy()

        # 2. Encoding & Resize (spart Tokens & Bandbreite)
        h, w = target.shape[:2]
        if w > 512:
            scale = 512 / w
            target = cv2.resize(target, (int(w*scale), int(h*scale)))
        
        # DEBUG: Speichere Bild zum Anschauen
        import os
        debug_dir = "/home/niclas/robocup/PR/pics"
        os.makedirs(debug_dir, exist_ok=True)
        debug_path = os.path.join(debug_dir, "guest_stylist_debug.jpg")
        cv2.imwrite(debug_path, target)
        rospy.loginfo(f"Bild gespeichert unter: {debug_path}")
        rospy.loginfo(f"   → Öffne es mit: eog {debug_path}")
        
        # Versuche Bild zu öffnen (falls GUI verfügbar)
        try:
            import subprocess
            subprocess.Popen(['eog', debug_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            rospy.loginfo("   → Bild wird automatisch geöffnet...")
        except:
            pass  # Kein GUI verfügbar, nur Datei speichern
            
        _, buf = cv2.imencode('.jpg', target, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        b64_img = base64.b64encode(buf).decode('utf-8')
        
        # 3. OpenAI Analyse
        try:
            rospy.loginfo(f"📤 Sende Request an OpenAI (Model: {MODEL})...")
            rospy.loginfo(f"   Bildgröße: {target.shape[1]}x{target.shape[0]}, {len(b64_img)} Bytes (base64)")
            
            resp = self.client.chat.completions.create(
                model=MODEL,
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": "Describe the main person in this image in detail for giving a personal compliment. You MUST include ALL of these: 1) gender (man/woman), 2) estimated age in years (e.g., 'about 25 years old'), 3) hair color and style (e.g., 'short brown hair'), 4) clothing colors and items (e.g., 'wearing a red shirt and blue jeans'). Be specific with colors! Return ONLY the description text in one sentence. Example: 'A woman about 30 years old with long blonde hair, wearing a green blouse and black pants.' If no person, say 'no person'."},
                            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64_img}", "detail": "low"}}
                        ]
                    }
                ],
                max_completion_tokens=100
            )
            desc = resp.choices[0].message.content.strip()
            rospy.loginfo(f"✨ Stylist says: '{desc}'")
            rospy.loginfo("💤 Waiting for next request...")
            return AnalyzeSnapshotResponse(description=desc)
            
        except Exception as e:
            rospy.logerr(f"Stylist Error: {e}")
            rospy.logerr(f"   API Key (erste 20 Zeichen): {API_KEY[:20]}...")
            rospy.logerr(f"   Model: {MODEL}")
            rospy.logerr(f"   Tipp: Service Account Keys unterstützen oft keine Vision-Models!")
            rospy.logerr(f"   Tipp: Teste mit einem User API Key oder warte auf OpenAI Server-Fix")
            rospy.loginfo("💤 Waiting for next request...")
            return AnalyzeSnapshotResponse(description=f"Error: {str(e)}")

if __name__ == '__main__':
    StylistNode()
    rospy.spin()
