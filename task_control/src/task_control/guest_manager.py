#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np

class GuestManager:
    def __init__(self):
        self.known_guests = []
        # Threshold set to 0.5 as requested
        self.match_threshold = 0.5


    def register_guest_profile(self, role, name, drink, clothes_msg=None):
        new_guest = {
            "role": role,
            "name": name,
            "favorite_drink": drink,  # WICHTIG: "favorite_drink" nicht "drink" für Konsistenz
            "face_encodings": [],
            "current_track_id": -1, 
            "current_seat": -1,
            "visual_description": None # NEW: Style description
        }
        self.known_guests.append(new_guest)
        #rospy.loginfo(f"Created profile for {role} ({name}). Waiting for face samples...")

    def update_seat(self, role, seat_id):
        """Speichert den aktuellen Sitzplatz für einen Gast."""
        guest = self.get_guest_by_role(role)
        if guest:
            guest["current_seat"] = seat_id
            # rospy.loginfo(f"🪑 Saved seat for {role}: Chair {seat_id}")

    def update_track_id(self, role, track_id):
        """Aktualisiert die aktuelle YOLO Track ID für einen Gast."""
        guest = self.get_guest_by_role(role)
        if guest:
            guest["current_track_id"] = track_id

    def get_guest_by_track_id(self, track_id):
        """Sucht einen Gast anhand seiner aktuellen Track ID."""
        if track_id == -1: return None
        for guest in self.known_guests:
            if guest.get("current_track_id") == track_id:
                return guest
        return None

    def update_guest(self, role, **kwargs):
        """
        Modular update: Aktualisiert beliebige Felder eines Guests.
        Beispiel: update_guest("host", name="Niclas", drink="Cola")
        
        HINWEIS: 'drink' wird automatisch auf 'favorite_drink' gemappt!
        """
        guest = self.get_guest_by_role(role)
        if not guest:
            rospy.logwarn(f"Cannot update - no guest with role '{role}' found!")
            return False
        
        for key, value in kwargs.items():
            # Mapping: Smart Brain sendet "drink", wir speichern als "favorite_drink"
            if key == "drink":
                key = "favorite_drink"
            
            if key in guest:
                old_value = guest[key]
                guest[key] = value
                rospy.loginfo(f"📝 Updated {role}.{key}: '{old_value}' → '{value}'")
            else:
                rospy.logwarn(f"Unknown field '{key}' for guest update!")
        return True

    def clear_samples_for_role(self, role):
        guest = self.get_guest_by_role(role)
        if guest:
            old_count = len(guest["face_encodings"])
            guest["face_encodings"] = [] 
            rospy.logwarn(f"CLEARED {old_count} corrupted samples for {role}.")

    def add_face_sample(self, role, encoding):
        if len(encoding) == 0: return
        guest = self.get_guest_by_role(role)
        if guest:
            guest["face_encodings"].append(np.array(encoding))
            sample_count = len(guest["face_encodings"])
            rospy.loginfo(f"Added face sample #{sample_count} for {role}.")

    def identify_guest_detailed(self, live_encoding, track_id="?"):
        """
        Gibt eine detaillierte Liste aller Vergleiche zurück.
        Rückgabe: Liste von Dictionaries [{'name': 'Niclas', 'role': 'host', 'dist': 0.12}, ...]
        """
        if live_encoding is None or len(live_encoding) == 0: return []
        if not self.known_guests: return []

        live_vec = np.array(live_encoding)
        results = []

        for guest in self.known_guests:
            stored_vectors = guest["face_encodings"]
            if not stored_vectors:
                continue

            # Beste Distanz für diesen Gast finden
            guest_min_dist = 100.0
            for stored_vec in stored_vectors:
                dot = np.dot(live_vec, stored_vec)
                norm_a = np.linalg.norm(live_vec)
                norm_b = np.linalg.norm(stored_vec)
                if norm_a == 0 or norm_b == 0: continue
                sim = dot / (norm_a * norm_b)
                dist = 1.0 - sim
                if dist < guest_min_dist: guest_min_dist = dist
            
            results.append({
                "name": guest["name"],
                "role": guest["role"],
                "dist": guest_min_dist,
                "is_match": guest_min_dist < self.match_threshold
            })
            
        return results

    def identify_guest(self, live_encoding, track_id="?"):
        """
        Compares live face against ALL guests and prints ALL distances.
        Returns: (guest_dict, distance)
        """
        if live_encoding is None or len(live_encoding) == 0: return None, 100.0
        if not self.known_guests: return None, 100.0

        # Nutze die neue detaillierte Funktion intern
        results = self.identify_guest_detailed(live_encoding, track_id)
        
        best_match_guest = None
        global_lowest_distance = 100.0

        for res in results:
            if res["dist"] < global_lowest_distance:
                global_lowest_distance = res["dist"]
                if res["is_match"]:
                    best_match_guest = self.get_guest_by_role(res["role"])

        return best_match_guest, global_lowest_distance

    def get_guest_by_role(self, role):
        for guest in self.known_guests:
            if guest["role"] == role: return guest
        return None

    def get_guest_info(self, role):
        """Gibt alle Infos eines Guests zurück (für Debugging/Display)."""
        guest = self.get_guest_by_role(role)
        if guest:
            return {
                "role": guest["role"],
                "name": guest["name"],
                "favorite_drink": guest["favorite_drink"],
                "face_samples": len(guest["face_encodings"])
            }
        return None
