#!/usr/bin/env python3
"""
Task Control Configuration
===========================
Zentrale Konfigurationsdatei für Wegpunkte und andere Einstellungen.
"""

# === WEGPUNKTE ===
# Koordinaten gemessen mit 'rostopic echo /amcl_pose'
WAYPOINTS = {
    "host":     {"x": 1.2803457668322669, "y": -0.07426162016580672, "z": -0.24766416072108388, "w": 0.9688459441491827},
    "seats":    {"x": 0.08603908090974946, "y": -0.01397897753090582, "z": -0.259, "w": 0.966},  # -30° (optimiert)
    "door":     {"x": -1.785381654612821, "y": -0.0498903083778172, "z": -0.831, "w": 0.556},  # 5° nach links gedreht
    "greeting": {"x": -0.8987414833147379, "y": -0.3006151626987276, "z": -0.9955928056691757, "w": 0.09378147631477626},
    "wait":     {"x": 0.08603908090974946, "y": -0.01397897753090582, "z": 0.8290375725550416, "w": 0.5591929034707469},  # Wie seats, aber 135° gedreht
}
