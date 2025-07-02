"""
Unified script to add test data to both devices.json and attendance_system.db.
- Adds test devices to devices.json (for dashboard display)
- Adds test students, devices, attendance, and logs to the SQLite database
"""

import json
import os
import sqlite3
from datetime import datetime, timedelta
import pytz

# --- Devices JSON Section ---
DEVICES_JSON = 'devices.json'
IRAN_TIMEZONE = pytz.timezone('Asia/Tehran')

def get_iran_time_str():
    return datetime.now(IRAN_TIMEZONE).strftime("%Y-%m-%d %H:%M:%S")

def add_devices_to_json():
    devices = [
        {
            "deviceName": "ESP32-1",
            "deviceType": "master",
            "ipAddress": "192.168.1.10",
            "firstSeen": get_iran_time_str(),
            "lastSeen": get_iran_time_str(),
            "status": "online",
            "firmwareVersion": "v1.0.0",
            "freeHeap": 20000,
            "peopleCount": 0
        },
        {
            "deviceName": "ESP32-2",
            "deviceType": "slave",
            "ipAddress": "192.168.1.11",
            "firstSeen": get_iran_time_str(),
            "lastSeen": get_iran_time_str(),
            "status": "online",
            "firmwareVersion": "v1.0.1",
            "freeHeap": 18000,
            "peopleCount": 3
        },
        {
            "deviceName": "ESP32-3",
            "deviceType": "slave",
            "ipAddress": "192.168.1.12",
            "firstSeen": get_iran_time_str(),
            "lastSeen": get_iran_time_str(),
            "status": "offline",
            "firmwareVersion": "v1.0.1",
            "freeHeap": 17000,
            "peopleCount": 0
        }
    ]

    # If file exists, load and append (avoid duplicates by deviceName)
    if os.path.exists(DEVICES_JSON):
        try:
            with open(DEVICES_JSON, 'r') as f:
                existing = json.load(f)
                existing_names = {d.get("deviceName") for d in existing if "deviceName" in d}
                for dev in devices:
                    if dev["deviceName"] not in existing_names:
                        existing.append(dev)
                devices = existing
        except Exception:
            pass

    with open(DEVICES_JSON, 'w') as f:
        json.dump(devices, f, indent=2)
    print("Test devices added to devices.json.")

# --- Database Section ---
DATABASE = 'attendance_system.db'

def add_students(conn):
    students = [
        ("A1B2C3D4", "Ali", "Rezaei", "99123456", "Dr. Ahmadi"),
        ("E5F6G7H8", "Sara", "Mohammadi", "99123457", "Dr. Ahmadi"),
        ("I9J0K1L2", "Reza", "Karimi", "99123458", "Dr. Hosseini"),
    ]
    for uid, first, last, number, prof in students:
        try:
            conn.execute(
                "INSERT OR IGNORE INTO students (uid, firstName, lastName, studentNumber, professor) VALUES (?, ?, ?, ?, ?)",
                (uid, first, last, number, prof)
            )
        except Exception as e:
            print(f"Error adding student {uid}: {e}")

def add_devices(conn):
    devices = [
        ("ESP32-1", "master", "192.168.1.10", "2025-07-02 08:00:00", "2025-07-02 09:00:00", "online", "v1.0.0", 20000, 0),
        ("ESP32-2", "slave", "192.168.1.11", "2025-07-02 08:10:00", "2025-07-02 09:05:00", "online", "v1.0.1", 18000, 3),
        ("ESP32-3", "slave", "192.168.1.12", "2025-07-02 08:20:00", "2025-07-02 08:50:00", "offline", "v1.0.1", 17000, 0),
    ]
    for name, typ, ip, first_seen, last_seen, status, fw, heap, people in devices:
        try:
            conn.execute(
                """INSERT OR IGNORE INTO devices
                (device_name, device_type, ip_address, first_seen, last_seen, status, firmware_version, free_heap, people_count)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                (name, typ, ip, first_seen, last_seen, status, fw, heap, people)
            )
        except Exception as e:
            print(f"Error adding device {name}: {e}")

def add_attendance(conn):
    # Get student IDs
    students = conn.execute("SELECT id FROM students").fetchall()
    devices = conn.execute("SELECT ip_address FROM devices").fetchall()
    now = datetime.now()
    for i, student in enumerate(students):
        # Assign attendance to each student, rotating device IPs
        device_ip = devices[i % len(devices)][0] if devices else "192.168.1.10"
        attendance_time = (now - timedelta(days=i)).strftime("%Y-%m-%d %H:%M:%S")
        try:
            conn.execute(
                "INSERT INTO attendance (student_id, attendance_time, device_ip) VALUES (?, ?, ?)",
                (student[0], attendance_time, device_ip)
            )
        except Exception as e:
            print(f"Error adding attendance for student {student[0]}: {e}")

def add_system_logs(conn):
    logs = [
        ("ESP32-1", "startup", "Device started successfully"),
        ("ESP32-2", "heartbeat", "Heartbeat received"),
        ("System", "student_add", "Test student added"),
    ]
    for device, log_type, msg in logs:
        try:
            conn.execute(
                "INSERT INTO system_logs (device_name, log_type, message) VALUES (?, ?, ?)",
                (device, log_type, msg)
            )
        except Exception as e:
            print(f"Error adding log: {e}")

def add_all_test_data_to_db():
    try:
        conn = sqlite3.connect(DATABASE)
        add_students(conn)
        add_devices(conn)
        add_attendance(conn)
        add_system_logs(conn)
        conn.commit()
        print("Test data added to attendance_system.db.")
        conn.close()
    except Exception as e:
        print(f"Could not add test data: {e}")

# --- Main ---
if __name__ == "__main__":
    add_devices_to_json()
    add_all_test_data_to_db()