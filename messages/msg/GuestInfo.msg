# GUEST_INFO.msg
#
# This message stores persistent data about a known person (guest or host).

# -----------------
# 1. LOGIC/WORLD MODEL IDENTIFIERS (Persistent IDs)
# -----------------
# Internal ID used by the State Machine (e.g., "GUEST_1", "HOST")
string internal_id

# Name and drink announced during the introduction
string name
string favorite_drink

# -----------------
# 2. VISION DATA (For Tracking and Re-Identification)
# -----------------
# The temporary ID assigned by the YOLO tracker. -1 if track is lost.
int32 current_track_id

# The face encoding vector (e.g., 128 float values from face_recognition)
float32[] face_encoding

int32 face_width
int32 face_height

# Dominant shirt color (e.g., "RED", "BLUE"). Used as Re-ID backup.
string shirt_color

string pants_color

# -----------------
# 3. SPATIAL DATA (For Anchoring, updated by the Brain)
# -----------------
# The last known location in the map frame (e.g., where they were seated)
geometry_msgs/PoseStamped last_map_pose