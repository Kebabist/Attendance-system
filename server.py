"""
ESP32 Attendance System Server

A Flask web application that manages student attendance using ESP32 devices.
The system provides:
- Student registration and management
- Real-time attendance tracking with duplicate prevention
- Device heartbeat monitoring and status management  
- Firmware upload and OTA (Over-The-Air) updates
- Web-based dashboard with real-time updates
- System event logging and monitoring

Database: SQLite (attendance_system.db)
Data Storage: JSON files for device persistence
Timezone: Iran Standard Time (Asia/Tehran)
ESP32 Communication: REST API endpoints for heartbeat and attendance
"""

from flask import (
    Flask, request, jsonify, render_template_string, send_from_directory
)
import sqlite3
import json
import os
from datetime import datetime, timedelta
import pytz
import requests
from flask_cors import CORS
import threading
import time
from werkzeug.utils import secure_filename
import asyncio
import concurrent.futures

# Flask application initialization
app = Flask(__name__)
# Enable Cross-Origin Resource Sharing for API endpoints
CORS(app)

# Configuration constants
DATABASE = 'attendance_system.db'    # SQLite database file for persistent storage
STUDENTS_JSON = 'students.json'     # JSON file for initial student data loading
DEVICES_JSON = 'devices.json'       # JSON file for device persistence and recovery
FIRMWARE_FOLDER = 'firmware'        # Directory for storing firmware files
IRAN_TIMEZONE = pytz.timezone('Asia/Tehran')  # Iran timezone for local time handling

# Create firmware upload folder if it doesn't exist
os.makedirs(FIRMWARE_FOLDER, exist_ok=True)

# Thread lock for database operations to prevent concurrent access issues
db_lock = threading.Lock()

def _load_devices_data():
    """
    Load device data from JSON file.
    Creates empty list if file doesn't exist.
    
    Returns:
        list: Device data as a list of device objects
    """
    if not os.path.exists(DEVICES_JSON):
        with open(DEVICES_JSON, 'w') as f:
            json.dump([], f) # Create an empty list if file doesn't exist
        return []
    try:
        with open(DEVICES_JSON, 'r') as f:
            data = json.load(f)
            return data if isinstance(data, list) else [] # Ensure it's a list
    except (IOError, json.JSONDecodeError) as e:
        print(f"Error loading devices data: {e}") # Added print for debugging
        return []

def _save_devices_data(devices):
    """
    Save device data to JSON file for persistence.
    
    Args:
        devices (list): Device data to save
    """
    try:
        with open(DEVICES_JSON, 'w') as f:
            json.dump(devices, f, indent=4)
    except IOError as e:
        print(f"Error saving devices data: {e}") # Added print for debugging
        pass


class DatabaseConnection:
    """
    Context manager for SQLite database connections.
    Provides thread-safe database access with optimized settings for performance.
    """
    
    def __enter__(self):
        """
        Establish database connection with optimized SQLite settings.
        
        Returns:
            sqlite3.Connection: Database connection object with row factory
        """
        self.conn = sqlite3.connect(DATABASE, timeout=30, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row  # Enable column access by name
        # Enable WAL mode for better concurrency
        self.conn.execute('PRAGMA journal_mode=WAL;')
        self.conn.execute('PRAGMA synchronous=NORMAL;')
        self.conn.execute('PRAGMA cache_size=1000;')
        self.conn.execute('PRAGMA temp_store=memory;')
        return self.conn
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Close database connection and handle transactions properly.
        
        Args:
            exc_type: Exception type if an error occurred
            exc_val: Exception value if an error occurred  
            exc_tb: Exception traceback if an error occurred
        """
        if self.conn:
            try:
                if exc_type is None:
                    self.conn.commit()  # Commit if no errors
                else:
                    self.conn.rollback()  # Rollback on error
            finally:
                self.conn.close()
            

def get_db_connection():
    """
    Create a new database connection with optimized settings.
    
    Returns:
        sqlite3.Connection: Database connection with row factory and performance optimizations
    """
    conn = sqlite3.connect(DATABASE, timeout=30, check_same_thread=False)
    conn.row_factory = sqlite3.Row  # Enable column access by name
    # Enable WAL mode for better concurrency
    conn.execute('PRAGMA journal_mode=WAL;')
    conn.execute('PRAGMA synchronous=NORMAL;')
    conn.execute('PRAGMA cache_size=1000;')
    conn.execute('PRAGMA temp_store=memory;')
    return conn


def log_system_event(device_name, log_type, message):
    """
    Thread-safe logging of system events with retry logic.
    
    Args:
        device_name (str): Name/IP of the device generating the event
        log_type (str): Type of log event (INFO, ERROR, WARNING, etc.)
        message (str): Log message content
    """
    retry_count = 0
    max_retries = 3
    
    while retry_count < max_retries:
        try:
            with DatabaseConnection() as conn:
                c = conn.cursor()
                c.execute(
                    '''INSERT INTO system_logs (device_name, log_type, message)
                       VALUES (?, ?, ?)''',
                    (device_name, log_type, message)
                )
            return  # Success, exit the function
        except sqlite3.OperationalError as e:
            if "database is locked" in str(e).lower() and retry_count < max_retries - 1:
                retry_count += 1
                time.sleep(0.05 * retry_count)  # Small delay with backoff
                continue
            else:
                print(f"Database locked in log_system_event after {retry_count + 1} attempts: {str(e)}")
                break
        except Exception as e:
            print(f"Error logging event: {str(e)}")
            break


def update_device_status(device_name, device_type, ip_address, firmware_version=None, free_heap=None, people_count=None):
    """
    Update device status with people count support.
    Manages device registry in JSON file with heartbeat timestamps.
    
    Args:
        device_name (str): Name of the device
        device_type (str): Type of device (master/slave)
        ip_address (str): IP address of the device
        firmware_version (str, optional): Current firmware version
        free_heap (int, optional): Available memory in bytes
        people_count (int, optional): Number of people detected by device
    """
    # Load existing devices
    if os.path.exists(DEVICES_JSON):
        try:
            with open(DEVICES_JSON, 'r') as f:
                devices_data = json.load(f)
                if not isinstance(devices_data, list):
                    devices_data = []
        except (json.JSONDecodeError, IOError):
            devices_data = []
    else:
        devices_data = []
    
    current_time_str = get_iran_time().strftime("%Y-%m-%d %H:%M:%S")
    
    # Find existing device
    found_device = None
    for device in devices_data:
        if (device.get('deviceName') == device_name or 
            device.get('device_name') == device_name):
            found_device = device
            break
    
    if found_device:
        # Update existing device
        found_device['deviceName'] = device_name
        found_device['deviceType'] = device_type
        found_device['ipAddress'] = ip_address
        found_device['lastSeen'] = current_time_str
        found_device['status'] = 'online'
        
        if firmware_version is not None:
            found_device['firmwareVersion'] = firmware_version
        if free_heap is not None:
            found_device['freeHeap'] = free_heap
        if people_count is not None:
            found_device['peopleCount'] = people_count
    else:
        # Add new device
        new_device = {
            'deviceName': device_name,
            'deviceType': device_type,
            'ipAddress': ip_address,
            'firstSeen': current_time_str,
            'lastSeen': current_time_str,
            'status': 'online',
            'firmwareVersion': firmware_version,
            'freeHeap': free_heap,
            'peopleCount': people_count if people_count is not None else 0
        }
        devices_data.append(new_device)
    
    # Save updated devices data
    try:
        with open(DEVICES_JSON, 'w') as f:
            json.dump(devices_data, f, indent=2)
    except IOError as e:
        print(f"Error saving devices data: {e}")


def get_iran_time():
    """
    Get current time in Iran timezone.
    
    Returns:
        datetime: Current datetime object in Asia/Tehran timezone
    """
    return datetime.now(IRAN_TIMEZONE)


def init_db():
    """
    Initialize database and create all required tables.
    Creates students, attendance, devices, and system_logs tables with proper indexes.
    """
    conn = sqlite3.connect(DATABASE)
    c = conn.cursor()

    # Students table
    c.execute('''CREATE TABLE IF NOT EXISTS students
                 (id INTEGER PRIMARY KEY AUTOINCREMENT,
                  uid TEXT COLLATE NOCASE UNIQUE,
                  firstName TEXT,
                  lastName TEXT,
                  studentNumber TEXT UNIQUE,
                  professor TEXT,
                  registerDate TIMESTAMP DEFAULT CURRENT_TIMESTAMP)''')

    # Attendance table
    c.execute('''CREATE TABLE IF NOT EXISTS attendance
                 (id INTEGER PRIMARY KEY AUTOINCREMENT,
                  student_id INTEGER,
                  attendance_time TIMESTAMP,
                  device_ip TEXT,
                  FOREIGN KEY(student_id) REFERENCES students(id))''')
    
    try:
        c.execute('''CREATE UNIQUE INDEX IF NOT EXISTS idx_student_daily_attendance
                     ON attendance (student_id, DATE(attendance_time))''')
    except sqlite3.OperationalError as e:
        print(f"Could not create unique index on attendance, it might already exist: {e}")


    # Devices table for device management
    c.execute('''CREATE TABLE IF NOT EXISTS devices
                 (id INTEGER PRIMARY KEY AUTOINCREMENT,
                  device_name TEXT UNIQUE COLLATE NOCASE,
                  device_type TEXT,
                  ip_address TEXT,
                  first_seen TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                  last_seen TIMESTAMP,
                  status TEXT DEFAULT 'offline',
                  firmware_version TEXT,
                  free_heap INTEGER,
                  people_count INTEGER DEFAULT 0)''')


    # System logs table
    c.execute('''CREATE TABLE IF NOT EXISTS system_logs
                 (id INTEGER PRIMARY KEY AUTOINCREMENT,
                  timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                  device_name TEXT,
                  log_type TEXT,
                  message TEXT)''')


    conn.commit()
    conn.close()


@app.route("/api/devices/<path:device_name_to_delete>", methods=["DELETE"])
def delete_device_entry_route(device_name_to_delete):
    with db_lock:
        devices = _load_devices_data()
        
        device_to_remove_actual_name = None
        found_device_to_delete = False
        
        # Create a new list excluding the device to be deleted (case-insensitive search)
        updated_devices_list = []
        for dev in devices:
            # Check if this is the device to delete (case-insensitive)
            # and we haven't already marked one for deletion in this pass
            if not found_device_to_delete and dev["deviceName"].lower() == device_name_to_delete.lower():
                device_to_remove_actual_name = dev["deviceName"] # Store the actual name for logging
                found_device_to_delete = True
                # Skip adding this device to the new list, effectively deleting it
            else:
                updated_devices_list.append(dev)
        
        if found_device_to_delete:
            _save_devices_data(updated_devices_list)
            log_system_event(device_to_remove_actual_name, "device_management", f"Device '{device_to_remove_actual_name}' deleted successfully (request was for '{device_name_to_delete}').")
            return jsonify({"message": f"Device '{device_to_remove_actual_name}' deleted successfully."}), 200
        else:
            log_system_event(device_name_to_delete, "device_management", f"Attempted to delete device '{device_name_to_delete}', but it was not found (case-insensitive search).")
            return jsonify({"error": f"Device '{device_name_to_delete}' not found."}), 404


def load_students_from_json():
    """
    Load students from JSON file to database.
    Reads student data from students.json and inserts into database,
    avoiding duplicates using INSERT OR IGNORE.
    """
    if not os.path.exists(STUDENTS_JSON):
        return

    with open(STUDENTS_JSON, 'r', encoding='utf-8') as f:
        students_data = json.load(f)

    conn = sqlite3.connect(DATABASE)
    c = conn.cursor()

    for student in students_data:
        try:
            c.execute('''INSERT OR IGNORE INTO students
                         (uid, firstName, lastName, studentNumber, professor)
                         VALUES (?, ?, ?, ?, ?)''',
                      (student['uid'].upper(),
                       student['firstName'],
                       student['lastName'],
                       student['studentNumber'],
                       student['professor']))
        except sqlite3.IntegrityError as e:
            print(f"Error inserting student {student['uid']}: {str(e)}")
            continue

    conn.commit()
    conn.close()


# Flask Routes - Web Interface and API Endpoints

@app.route('/')
def dashboard():
    """
    Main integrated dashboard route.
    Serves the complete web interface with device monitoring,
    attendance tracking, and system management.
    
    Returns:
        str: Rendered HTML dashboard template
    """
    return render_template_string(DASHBOARD_TEMPLATE)


# Student Management API Endpoints

@app.route('/api/students', methods=['POST'])
def add_student():
    """
    Add a new student to the system.
    Validates required fields and prevents duplicate UID/student numbers.
    
    Expected JSON payload:
        uid (str): Student UID (will be converted to uppercase)
        firstName (str): Student's first name
        lastName (str): Student's last name  
        studentNumber (str): Unique student identification number
        professor (str): Associated professor name
        
    Returns:
        JSON: Success message with student ID or error details
    """
    try:
        data = request.get_json()
        required_fields = [
            'uid',
            'firstName',
            'lastName',
            'studentNumber',
            'professor'        ]
        
        if not all(field in data for field in required_fields):
            return jsonify(
                {'error': 'Required fields missing'}
            ), 400

        with DatabaseConnection() as conn:
            c = conn.cursor()

            # Check for duplicates
            existing = c.execute(
                'SELECT id FROM students WHERE uid = ? OR studentNumber = ?',
                (data['uid'].upper(), data['studentNumber'])
            ).fetchone()
            
            if existing:
                return jsonify({'error': 'Student already exists'}), 409

            # Insert new student
            c.execute(
                '''INSERT INTO students (
                        uid, firstName, lastName, studentNumber, professor
                    ) VALUES (?, ?, ?, ?, ?)''',
                (
                    data['uid'].upper(),
                    data['firstName'],
                    data['lastName'],
                    data['studentNumber'],
                    data['professor']
                )
            )
            # Commit is handled by DatabaseConnection context manager

        log_system_event(
            'System',
            'student_add',
            f"Student {data['firstName']} {data['lastName']} added"
        )
        
        return jsonify({'message': 'Student added successfully'}), 201

    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/students', methods=['GET'])
def get_students():
    """Get all students"""
    conn = sqlite3.connect(DATABASE)
    c = conn.cursor()
    
    students = c.execute('SELECT * FROM students ORDER BY lastName').fetchall()
    conn.close()

    students_list = []
    for student in students:
        students_list.append({
            'id': student[0],
            'uid': student[1],
            'firstName': student[2],
            'lastName': student[3],
            'studentNumber': student[4],
            'professor': student[5],
            'registerDate': student[6] if len(student) > 6 else None
        })

    return jsonify(students_list)


# Attendance routes
@app.route('/register_attendance', methods=['POST'])
def register_attendance():
    """
    Register attendance from ESP32 devices with duplicate prevention.
    
    Accepts student identification via either studentNumber or UID.
    Prevents duplicate attendance records for the same day.
    Logs all attendance attempts and errors for auditing.
    
    Expected JSON payload:
        studentNumber (str) OR uid (str): Student identification
        
    Returns:
        JSON: Success/error message with student details
    """    
    data = request.get_json()
    client_ip = request.remote_addr

    if not data or ('studentNumber' not in data and 'uid' not in data):
        log_system_event(f"ESP32-{client_ip}", 'attendance_error', 'Student ID not provided in request.')
        return jsonify({'error': 'Student ID (studentNumber or UID) not provided'}), 400

    student_identifier_key = 'studentNumber' if 'studentNumber' in data else 'uid'
    student_identifier_value = data[student_identifier_key]
    if student_identifier_key == 'uid':
        student_identifier_value = student_identifier_value.upper()


    with DatabaseConnection() as conn:
        c = conn.cursor()

        # Find student
        if student_identifier_key == 'studentNumber':
            student = c.execute(
                'SELECT id, uid, firstName, lastName, studentNumber FROM students WHERE studentNumber = ?',
                (student_identifier_value,)
            ).fetchone()
        else: # uid
            student = c.execute(
                'SELECT id, uid, firstName, lastName, studentNumber FROM students WHERE uid = ?',
                (student_identifier_value,)
            ).fetchone()

        if not student:
            log_system_event(f"ESP32-{client_ip}", 'attendance_error', f"Student not found with {student_identifier_key}: {student_identifier_value}")
            return jsonify(
                {'error': f'Student not found with {student_identifier_key}: {student_identifier_value}'}
            ), 404

        student_id = student['id']
        student_name_display = f"{student['firstName']} {student['lastName']} ({student['studentNumber']})"

        # Check for existing attendance for this student today
        current_time_obj = get_iran_time()
        current_date_str = current_time_obj.strftime('%Y-%m-%d')
        current_datetime_str = current_time_obj.strftime('%Y-%m-%d %H:%M:%S')

        c.execute("""
            SELECT id FROM attendance
            WHERE student_id = ? AND DATE(attendance_time) = ?
        """, (student_id, current_date_str))
        existing_attendance = c.fetchone()

        if existing_attendance:
            log_system_event(
                f"ESP32-{client_ip}", 'duplicate_attendance', 
                f"Duplicate attendance attempt for {student_name_display}"
            )
            return jsonify({
                'message': f'Attendance already registered for {student_name_display} today.',
                'status': 'duplicate',
                'student': {
                    'uid': student['uid'],
                    'firstName': student['firstName'],
                    'lastName': student['lastName'],
                    'studentNumber': student['studentNumber']
                },
                'attendance_time': current_datetime_str # Or the time of the original attendance
            }), 409 # HTTP 409 Conflict

        # Register new attendance
        c.execute(
            '''INSERT INTO attendance (student_id, attendance_time, device_ip)
               VALUES (?, ?, ?)''',
            (student_id, current_datetime_str, client_ip)
        )
        # Commit is handled by DatabaseConnection context manager

    # Update device status (assuming device_name can be derived or is static for attendance points)
    # If the ESP32 sends its name, use that. Otherwise, derive from IP.
    device_name_from_esp = data.get('device_name', f"ESP32-{client_ip}")
    device_type_from_esp = data.get('device_type', 'slave') # Assume slave if not specified
    update_device_status(device_name_from_esp, device_type_from_esp, client_ip)
    
    log_system_event(
        device_name_from_esp, 'attendance_registered', 
        f"Attendance registered for {student_name_display}"
    )

    return jsonify({
        'message': 'Attendance registered successfully',
        'status': 'success',
        'student': {
            'uid': student['uid'],
            'firstName': student['firstName'],
            'lastName': student['lastName'],
            'studentNumber': student['studentNumber']
        },
        'attendance_time': current_datetime_str
    }), 201 # HTTP 201 Created


# Attendance Management API Endpoints

@app.route('/api/attendance', methods=['GET'])
def get_attendance():
    """
    Get attendance records with pagination support.
    
    Query parameters:
        limit (int, optional): Maximum number of records to return (default: 50)
        
    Returns:
        JSON: List of attendance records with student details
    """
    limit = request.args.get('limit', 50, type=int)
    
    try:
        with DatabaseConnection() as conn:
            c = conn.cursor()
            
            attendance = c.execute('''SELECT a.id, s.uid, s.firstName, s.lastName,
                                    s.studentNumber, a.attendance_time, a.device_ip
                                    FROM attendance a
                                    JOIN students s ON a.student_id = s.id
                                    ORDER BY a.attendance_time DESC
                                    LIMIT ?''', (limit,)).fetchall()

        attendance_list = []
        for record in attendance:
            attendance_list.append({
                'id': record[0],
                'uid': record[1],
                'studentName': f"{record[2]} {record[3]}",
                'studentNumber': record[4],
                'attendanceTime': record[5],
                'deviceIP': record[6]
            })

        return jsonify(attendance_list)
    except Exception as e:
        print(f"Error in get_attendance: {str(e)}")
        return jsonify({'error': str(e)}), 500


# Device Management API Endpoints

@app.route('/api/devices', methods=['GET'])
def get_devices():
    """
    Get all registered devices with people count and status information.
    
    Retrieves device data from JSON file storage including:
    - Device name and type (master/slave)
    - IP address and connection status
    - People count from sensors
    - Last heartbeat timestamp
    - Firmware version and system health
    
    Returns:
        JSON: List of all registered devices with their current status
    """
    devices_list = []
    
    try:
        # Check if you're using JSON file approach
        if os.path.exists(DEVICES_JSON):
            try:
                with open(DEVICES_JSON, 'r') as f:
                    devices_data = json.load(f)
                    if not isinstance(devices_data, list):
                        devices_data = []
            except (json.JSONDecodeError, IOError):
                devices_data = []
        else:
            devices_data = []
        
        current_time = get_iran_time()
        
        for device in devices_data:
            try:
                # Handle different possible key names
                device_name = device.get('deviceName') or device.get('device_name', 'Unknown')
                device_type = device.get('deviceType') or device.get('device_type', 'unknown')
                ip_address = device.get('ipAddress') or device.get('ip_address', 'N/A')
                last_seen = device.get('lastSeen') or device.get('last_seen', '')
                firmware_version = device.get('firmwareVersion') or device.get('firmware_version')
                free_heap = device.get('freeHeap') or device.get('free_heap')
                people_count = device.get('peopleCount') or device.get('people_count') or device.get('currentPeople', 0)
                
                # Calculate status based on last seen
                status = 'offline'  # Default
                if last_seen:
                    try:
                        last_seen_dt = datetime.strptime(last_seen, '%Y-%m-%d %H:%M:%S')
                        last_seen_iran = IRAN_TIMEZONE.localize(last_seen_dt)
                        time_diff = (current_time - last_seen_iran).total_seconds()
                        status = 'online' if time_diff < 300 else 'offline'  # 5 minutes timeout
                    except ValueError:
                        status = 'offline'
                
                device_info = {
                    'deviceName': device_name,
                    'deviceType': device_type,
                    'ipAddress': ip_address,
                    'lastSeen': last_seen,
                    'status': status,
                    'firmwareVersion': firmware_version,
                    'freeHeap': free_heap,
                    'peopleCount': people_count  # Changed back to peopleCount for consistency
                }
                devices_list.append(device_info)
                
            except Exception as e:
                print(f"Error processing device: {e}")
                continue
        
        return jsonify(devices_list)
        
    except Exception as e:
        print(f"Error in get_devices: {str(e)}")
        return jsonify([]), 500

@app.route('/api/device_heartbeat', methods=['POST'])
def device_heartbeat():
    """
    Enhanced device heartbeat handler with people counting capability.
    
    Receives periodic status updates from ESP32 devices including:
    - Device identification and type
    - Firmware version and system health
    - People count from sensors
    - Memory usage statistics
    
    Expected JSON payload:
        device_name (str, optional): Device identifier
        device_type (str, optional): Device type (master/slave)
        firmware_version (str, optional): Current firmware version
        free_heap (int, optional): Available memory in bytes
        peopleCount/people_count (int, optional): Detected people count
        
    Returns:
        JSON: Status acknowledgment
    """
    try:
        data = request.get_json() or {}
        ip = request.remote_addr
        
        device_name = data.get('device_name', f"ESP32-{ip}")
        device_type = data.get('device_type', 'unknown')
        firmware_version = data.get('firmware_version')
        free_heap = data.get('free_heap')
        people_count = data.get('peopleCount', data.get('people_count'))
        
        # Update device status
        update_device_status(
            device_name=device_name,
            device_type=device_type,
            ip_address=ip,
            firmware_version=firmware_version,
            free_heap=free_heap,
            people_count=people_count
        )
        
        log_system_event(device_name, 'heartbeat', f'Heartbeat with people count: {people_count}')
        return jsonify({'status': 'ok'})
        
    except Exception as e:
        print(f"Error in device_heartbeat: {str(e)}")
        return jsonify({'error': str(e)}), 500


# ========== COMMENTED OUT: LoRa slave heartbeat endpoint ==========
# Since we're now using WiFi OTA for slaves, this LoRa heartbeat is not needed
# Keep code for potential future fallback use
"""
@app.route('/api/lora_slave_heartbeat', methods=['POST'])
def lora_slave_heartbeat():
    \"\"\"Handle LoRa slave heartbeat data forwarded by master device\"\"\"
    try:
        data = request.get_json()
        master_ip = request.remote_addr
        
        # Log the LoRa slave heartbeat
        log_system_event('ESP32-Master', 'lora_heartbeat', f'LoRa slave heartbeat from {master_ip}: {data}')
        
        # Extract slave information if available
        if data and 'slave_data' in data:
            slave_info = data['slave_data']
            slave_id = slave_info.get('slave_id', 'Unknown')
            slave_ip = slave_info.get('slave_ip', 'Unknown')
            
            # Try to update slave device status
            if slave_ip != 'Unknown':
                try:
                    with DatabaseConnection() as conn:
                        c = conn.cursor()
                        current_time = get_iran_time().strftime('%Y-%m-%d %H:%M:%S')
                        
                        # Update slave device status
                        c.execute('''INSERT OR REPLACE INTO devices 
                                    (device_name, device_type, ip_address, last_seen, status, firmware_version, free_heap)
                                    VALUES (?, 'slave', ?, ?, 'online', ?, ?)''',
                                  (f"ESP32-{slave_ip}", slave_ip, current_time, 
                                   slave_info.get('firmware_version'), slave_info.get('free_heap')))
                        
                        log_system_event(f"ESP32-{slave_ip}", 'lora_heartbeat', f'LoRa heartbeat via master')
                except Exception as db_error:
                    print(f"Database error in lora_slave_heartbeat: {str(db_error)}")
        
        return jsonify({'status': 'ok', 'message': 'LoRa slave heartbeat processed'})
    except Exception as e:
        print(f"Error in lora_slave_heartbeat: {str(e)}")
        return jsonify({'error': str(e)}), 500
"""
# ========== END COMMENTED LoRa slave heartbeat ==========
    

@app.route('/api/device/<device_ip>/reboot', methods=['POST'])
def reboot_device(device_ip):
    """Send reboot command to device"""
    try:
        response = requests.get(f"http://{device_ip}/reboot", timeout=5)
        log_system_event(f"ESP32-{device_ip}", 'command', 'Reboot command sent')
        return jsonify({'message': 'Reboot command sent successfully'})
    except requests.RequestException as e:
        return jsonify({
            'error': f'Failed to send reboot command: {str(e)}'
        }), 500


# ========== COMMENTED OUT: Trigger slave LoRa OTA endpoint ==========
# Since we're now using WiFi OTA for slaves, this endpoint is not needed
# Keep code for potential future fallback use
"""
@app.route('/api/device/<device_ip>/trigger_slave_ota', methods=['POST'])
def trigger_slave_ota(device_ip):
    \"\"\"Trigger slave OTA mode via master device for LoRa-based slaves\"\"\"
    try:
        # Check if this is a slave device
        with DatabaseConnection() as conn:
            c = conn.cursor()
            device_info = c.execute('SELECT device_type, device_name FROM devices WHERE ip_address = ?', (device_ip,)).fetchone()
            
            if not device_info:
                return jsonify({'error': 'Device not found in database'}), 404
                
            device_type = device_info[0]
            device_name = device_info[1]
            
            if device_type != 'slave':
                return jsonify({'error': 'This function is only for slave devices'}), 400
            
            # For LoRa-based slaves, we need to send the command through the master device
            # Find the master device (should be ESP32-Master)
            master_device = c.execute('SELECT ip_address FROM devices WHERE device_type = ? AND status = ?', 
                                     ('master', 'online')).fetchone()
            
            if not master_device:
                return jsonify({'error': 'No online master device found to relay the command'}), 503
                
            master_ip = master_device[0]
        
        # Send LoRa OTA trigger command to master, which will relay it to the slave
        response = requests.get(f"http://{master_ip}/trigger_slave_ota?target={device_ip}", timeout=10)
        
        if response.status_code == 200:
            log_system_event(device_name, 'command', f'Slave OTA triggered via master {master_ip}')
            return jsonify({'message': f'Slave OTA triggered successfully via master {master_ip}'})
        else:
            error_msg = f'Master device returned error: {response.text}'
            log_system_event(device_name, 'command_error', error_msg)
            return jsonify({'error': error_msg}), response.status_code
            
    except requests.RequestException as e:
        error_msg = f'Failed to communicate with master device: {str(e)}'
        log_system_event(f"ESP32-{device_ip}", 'command_error', error_msg)
        return jsonify({'error': error_msg}), 500
    except Exception as e:
        error_msg = f'Unexpected error: {str(e)}'
        log_system_event(f"ESP32-{device_ip}", 'command_error', error_msg)
        return jsonify({'error': error_msg}), 500
"""
# ========== END COMMENTED trigger slave LoRa OTA ==========


@app.route('/api/device/<device_ip>/status', methods=['GET'])
def get_device_status(device_ip):
    """Get device status"""
    try:
        response = requests.get(f"http://{device_ip}/status", timeout=5)
        if response.status_code == 200:
            status_data = response.json()
            # Update device info in database
            update_device_status(
                f"ESP32-{device_ip}", "master", device_ip, 
                None, status_data.get('freeHeap')
            )
            return jsonify(status_data)
        else:
            return jsonify({'error': 'Device not responding'}), 500
    except requests.RequestException as e:
        return jsonify({'error': f'Failed to get device status: {str(e)}'}), 500


# Firmware Management API Endpoints

@app.route('/api/firmware/upload', methods=['POST'])
def upload_firmware():
    """
    Upload firmware file for ESP32 devices.
    
    Accepts .bin firmware files and stores them securely in the firmware folder.
    Logs all upload events for audit trail.
    
    Expected form data:
        firmware (file): Binary firmware file (.bin extension required)
        
    Returns:
        JSON: Success message with filename or error details
    """
    if 'firmware' not in request.files:
        return jsonify({'error': 'No firmware file provided'}), 400
    
    file = request.files['firmware']
    if file.filename == '':
        return jsonify({'error': 'No file selected'}), 400
    
    if file and file.filename.endswith('.bin'):
        filename = secure_filename(file.filename)
        filepath = os.path.join(FIRMWARE_FOLDER, filename)
        file.save(filepath)
        
        log_system_event('System', 'firmware_upload', f"Firmware {filename} uploaded")
        return jsonify({'message': f'Firmware {filename} uploaded successfully'})
    
    return jsonify({'error': 'Invalid file format. Only .bin files allowed'}), 400


@app.errorhandler(500)
def handle_500(e):
    print(f"500 error: {str(e)}")
    return jsonify({'error': 'Internal server error', 'details': str(e)}), 500


@app.errorhandler(404)
def handle_404(e):
    return jsonify({'error': 'Not found', 'details': str(e)}), 404


# Make the firmware endpoint more robust
@app.route('/api/firmware', methods=['GET'])
def list_firmware():
    """List available firmware files"""
    try:
        # Make sure the firmware folder exists
        if not os.path.exists(FIRMWARE_FOLDER):
            os.makedirs(FIRMWARE_FOLDER)
            
        files = [f for f in os.listdir(FIRMWARE_FOLDER) if f.endswith('.bin')]
        firmware_list = []
        
        for file in files:
            filepath = os.path.join(FIRMWARE_FOLDER, file)
            stat = os.stat(filepath)
            firmware_list.append({
                'filename': file,
                'size': stat.st_size,
                'uploadDate': datetime.fromtimestamp(stat.st_mtime).strftime('%Y-%m-%d %H:%M:%S')
            })        
        return jsonify(firmware_list)
    except Exception as e:
        print(f"Error in list_firmware: {str(e)}")
        return jsonify({'error': str(e)}), 500


@app.route('/api/firmware/download/<filename>', methods=['GET'])
def download_firmware(filename):
    """Download firmware file for LoRa OTA - used by master device to fetch firmware"""
    try:
        # Security check - only allow .bin files and prevent path traversal
        if not filename.endswith('.bin') or '..' in filename or '/' in filename or '\\' in filename:
            return jsonify({'error': 'Invalid filename'}), 400
            
        filepath = os.path.join(FIRMWARE_FOLDER, filename)
        
        if not os.path.exists(filepath):
            return jsonify({'error': 'Firmware file not found'}), 404
            
        # Send the file as binary data
        return send_from_directory(FIRMWARE_FOLDER, filename, as_attachment=True, 
                                 mimetype='application/octet-stream')
                                 
    except Exception as e:
        print(f"Error in download_firmware: {str(e)}")
        return jsonify({'error': str(e)}), 500


@app.route('/api/firmware/push', methods=['POST'])
def push_firmware():
    """
    Push firmware to ESP32 devices with enhanced progress tracking.
    
    Supports both master and slave devices with automatic device type detection.
    Provides real-time progress monitoring and comprehensive error handling.
    Logs all OTA operations for audit and debugging purposes.
    
    Expected form data:
        device_ip (str): Target device IP address
        filename (str): Firmware filename to push
        
    Returns:
        JSON: Success/error message with operation details
    """
    try:
        device_ip = request.form.get('device_ip')
        filename = request.form.get('filename')
        if not device_ip or not filename:
            return jsonify({'error': 'Missing device_ip or filename'}), 400
            
        filepath = os.path.join(FIRMWARE_FOLDER, filename)
        if not os.path.exists(filepath):
            return jsonify({'error': 'Firmware file not found'}), 404
            
        # Get file size
        file_size = os.path.getsize(filepath)
        print(f"🔄 Starting firmware push: {filename} to {device_ip}")
        print(f"📊 File size: {file_size} bytes ({file_size/1024:.2f} KB)")
        log_system_event('System', 'ota_start', f"Starting OTA push of {filename} ({file_size} bytes) to {device_ip}")
        
        # Check device type from database
        with DatabaseConnection() as conn:
            c = conn.cursor()
            device_info = c.execute('SELECT device_type FROM devices WHERE ip_address = ?', (device_ip,)).fetchone()        
        device_type = device_info[0] if device_info else 'unknown'
        print(f"📱 Device type: {device_type}")
        
        # ========== COMMENTED OUT: LoRa OTA for slaves ==========
        # Now using WiFi OTA for all devices (both master and slave)
        # Keep LoRa OTA code for potential future fallback use
        """
        # For slave devices, use LoRa OTA instead of direct WiFi OTA
        if device_type == 'slave':
            print("🔄 Slave device detected, using LoRa OTA...")
            try:
                # Find master device
                with DatabaseConnection() as conn:
                    c = conn.cursor()
                    master_device = c.execute('SELECT ip_address FROM devices WHERE device_type = ? AND status = ?', 
                                             ('master', 'online')).fetchone()
                
                if not master_device:
                    return jsonify({'error': 'No online master device found for LoRa OTA'}), 503
                    
                master_ip = master_device[0]
                
                # Use a file-serving approach instead of sending large Base64 data
                print(f"📦 Setting up LoRa OTA for {file_size} bytes via file serving")
                
                # Calculate CRC32 for verification
                import zlib
                with open(filepath, 'rb') as f:
                    firmware_data = f.read()
                firmware_crc = zlib.crc32(firmware_data) & 0xffffffff
                
                # Convert IP to slave ID format
                slave_id = f"SLAVE-{device_ip.replace('.', '')}"
                
                # First, trigger slave to enter LoRa OTA mode
                print("📡 Step 1: Triggering slave LoRa OTA mode...")
                trigger_response = requests.get(f"http://{master_ip}/trigger_slave_ota", timeout=10)
                
                if trigger_response.status_code != 200:
                    return jsonify({'error': f'Failed to trigger slave OTA mode: {trigger_response.text}'}), 500
                  # Wait a moment for slave to enter OTA mode
                time.sleep(2)                # Now send the LoRa OTA initiation command with firmware details
                print("📡 Step 2: Initiating LoRa OTA on master...")
                
                # Use the IP address that the master is already connecting to
                # Since the master is sending heartbeats successfully, we know it can reach this IP
                server_host = request.headers.get('Host', 'Replace with server IP
                
                # use the Wi-Fi adapter IP that the master can actually reach
                if server_host in ['127.0.0.1', 'localhost', 'Replace with server IP
                    # Master is on 192.168.64.x network, so use the Wi-Fi IP
                    flask_server_url = "http://Replace with server IP
                    print(f"📡 Using Wi-Fi IP for master connectivity: {flask_server_url}")
                else:
                    flask_server_url = f"http://{server_host}:5001"
                    print(f"📡 Using request host: {flask_server_url}")
                
                print(f"📡 Master IP: {master_ip}, Server URL: {flask_server_url}")
                
                master_payload = {
                    'action': 'init_lora_ota',
                    'slave_id': slave_id,
                    'slave_ip': device_ip,
                    'firmware_size': file_size,
                    'firmware_crc32': firmware_crc,
                    'filename': filename,
                    'flask_server': flask_server_url,
                    'firmware_path': f'/api/firmware/download/{filename}'
                }
                
                # Use a simpler endpoint
                init_response = requests.post(f"http://{master_ip}/init_lora_ota", 
                                            json=master_payload, timeout=15)
                
                if init_response.status_code == 200:
                    log_system_event(f"ESP32-{device_ip}", 'lora_ota_initiated', 
                                   f"LoRa OTA initiated for {filename} via master")
                    return jsonify({
                        'message': f'LoRa OTA initiated for slave device via master {master_ip}',
                        'status': 'lora_ota_initiated',
                        'slave_id': slave_id,
                        'file_size': file_size,
                        'master_ip': master_ip,
                        'method': 'file_serving'
                    })
                else:
                    error_msg = f'Failed to initiate LoRa OTA: {init_response.text}'
                    log_system_event(f"ESP32-{device_ip}", 'ota_error', error_msg)
                    return jsonify({'error': error_msg}), init_response.status_code
                    
            except Exception as e:
                error_msg = f'LoRa OTA setup failed: {str(e)}'
                log_system_event(f"ESP32-{device_ip}", 'ota_error', error_msg)
                return jsonify({'error': error_msg}), 500
        """
        # ========== END COMMENTED LoRa OTA ==========
        
        # For all devices (master and slave), use direct WiFi OTA
        print(f"🔄 Using WiFi OTA for {device_type} device: {device_ip}")
        # Check if device is reachable first
        try:
            status_response = requests.get(f'http://{device_ip}/status', timeout=5)
            if status_response.status_code != 200:
                return jsonify({'error': 'Device not reachable'}), 500
        except requests.RequestException:
            return jsonify({'error': 'Device not responding'}), 500
        
        # Open file and prepare for upload
        with open(filepath, 'rb') as f:
            # *** CRITICAL FIX *** - Properly set Content-Length header
            files = {
                'firmware': (filename, f, 'application/octet-stream')
            }
            
            # *** ENHANCED *** - Set additional headers to help with size detection
            headers = {
                'Content-Length': str(file_size),
                'X-File-Size': str(file_size),
                'X-Firmware-Name': filename
            }
            
            url = f'http://{device_ip}/update'
            print(f"🌐 Uploading to: {url}")
            print(f"📤 Headers: Content-Length={file_size}")
            
            try:
                # Use requests session with proper chunking
                session = requests.Session()
                
                # Prepare the request with explicit content length
                print("📤 Starting firmware upload with size information...")
                
                # Create a custom request with file size information
                response = session.post(
                    url, 
                    files=files,
                    headers=headers,
                    timeout=300,  # 5 minute timeout
                    stream=False  # Don't stream to ensure proper headers
                )
                
                if response.status_code == 200:
                    print("✅ Upload successful!")
                    
                    # Wait a moment for processing
                    time.sleep(2)
                      # Check final status
                    final_status = check_ota_completion(device_ip, filename)
                    
                    log_system_event(f"ESP32-{device_ip}", 'wifi_ota_success', 
                                   f"WiFi OTA: Firmware {filename} ({file_size} bytes) uploaded successfully to {device_type}")
                    
                    return jsonify({
                        'message': f'Firmware uploaded successfully to {device_type} device via WiFi ({file_size} bytes)',
                        'status': final_status,
                        'file_size': file_size,
                        'method': 'wifi_direct'
                    })
                else:
                    error_msg = f"Upload failed with status {response.status_code}: {response.text}"
                    print(f"❌ {error_msg}")
                    log_system_event(f"ESP32-{device_ip}", 'ota_error', error_msg)
                    return jsonify({'error': error_msg}), 500
                    
            except requests.exceptions.Timeout:
                print("⏱️ Upload request timed out, checking device status...")
                # Check if OTA is still in progress
                ota_status = check_ota_progress(device_ip)
                if ota_status and ota_status.get('inProgress'):
                    return jsonify({
                        'message': f'OTA upload started ({file_size} bytes), monitoring progress...',
                        'status': 'in_progress',
                        'progress': ota_status,
                        'file_size': file_size
                    })
                else:
                    error_msg = "Upload timed out and no progress detected"
                    print(f"❌ {error_msg}")
                    log_system_event(f"ESP32-{device_ip}", 'ota_error', error_msg)
                    return jsonify({'error': error_msg}), 500
                    
            except requests.RequestException as e:
                error_msg = f'Upload failed: {str(e)}'
                print(f"❌ {error_msg}")
                log_system_event(f"ESP32-{device_ip}", 'ota_error', error_msg)
                return jsonify({'error': error_msg}), 500
                
    except Exception as e:
        error_msg = f'Error pushing firmware: {str(e)}'
        print(f"❌ {error_msg}")
        log_system_event('System', 'ota_error', error_msg)
        return jsonify({'error': error_msg}), 500


# OTA Progress Monitoring Functions

def check_ota_progress(device_ip):
    """
    Check OTA progress on target device.
    
    Args:
        device_ip (str): IP address of the device
        
    Returns:
        dict or None: Progress data from device or None if unreachable
    """
    try:
        response = requests.get(f'http://{device_ip}/ota_progress', timeout=5)
        if response.status_code == 200:
            return response.json()
    except requests.RequestException:
        pass
    return None


def check_ota_completion(device_ip, filename):
    """
    Enhanced OTA completion monitoring with progress tracking.
    
    Monitors device throughout the OTA process, waiting for completion
    or failure within a reasonable timeout period.
    
    Args:
        device_ip (str): IP address of the device
        filename (str): Firmware filename being pushed
        
    Returns:
        dict: Status information about the OTA operation
    """
    print("⏳ Monitoring OTA completion...")
    
    # Wait a bit for the upload to start processing
    time.sleep(3)
    
    # Check progress for up to 3 minutes
    for i in range(90):  # 90 attempts, 2 seconds each = 3 minutes
        try:
            progress = check_ota_progress(device_ip)
            if progress:
                percent = progress.get('percent', 0)
                status = progress.get('status', 'Unknown')
                in_progress = progress.get('inProgress', False)
                
                print(f"📊 OTA Progress: {percent}% - {status} (InProgress: {in_progress})")
                
                if not in_progress:
                    if percent >= 100:
                        print("✅ OTA completed successfully!")
                        return 'completed'
                    elif percent == 0:
                        print("❓ OTA status unclear, device may have rebooted")
                        return 'completed'  # Assume success if device rebooted
                    else:
                        print(f"❌ OTA failed at {percent}%!")
                        return 'failed'
                        
                # Log progress every 15 attempts (30 seconds)
                if i % 15 == 0:
                    log_system_event(f"ESP32-{device_ip}", 'ota_progress', 
                                   f"OTA Progress: {percent}% - {status}")
            else:
                # If we can't get progress, device might be rebooting
                if i > 30:  # After 1 minute of no response, assume success
                    print("📱 Device not responding, likely rebooted successfully")
                    return 'completed'
            
            time.sleep(2)
            
        except Exception as e:
            print(f"⚠️ Error checking OTA progress: {str(e)}")
            # If device stops responding after some progress, it likely rebooted successfully
            if i > 20:  # After 40 seconds
                print("📱 Device stopped responding, likely completed and rebooted")
                return 'completed'
            
    print("⏱️ OTA progress monitoring timed out")
    return 'timeout'


@app.route('/api/firmware/progress/<device_ip>', methods=['GET'])
def get_firmware_progress(device_ip):
    """Get real-time OTA progress from device"""
    try:
        progress = check_ota_progress(device_ip)
        if progress:
            return jsonify(progress)
        else:
            return jsonify({'error': 'Unable to get progress'}), 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/device/<device_ip>/ota_status', methods=['GET'])
def get_device_ota_status(device_ip):
    """Get detailed OTA status from device"""
    try:
        # Try both status endpoints
        status_response = requests.get(f"http://{device_ip}/status", timeout=5)
        progress_response = requests.get(f"http://{device_ip}/ota_progress", timeout=5)
        
        result = {}
        
        if status_response.status_code == 200:
            result['device_status'] = status_response.json()
            
        if progress_response.status_code == 200:
            result['ota_progress'] = progress_response.json()
            
        return jsonify(result)
        
    except requests.RequestException as e:
        return jsonify({'error': f'Failed to get OTA status: {str(e)}'}), 500


# System Logging and Monitoring API Endpoints

@app.route('/api/logs', methods=['GET'])
def get_system_logs():
    """
    Get system logs with pagination support.
    
    Retrieves system event logs including device heartbeats, attendance events,
    firmware updates, errors, and other system activities.
    
    Query parameters:
        limit (int, optional): Maximum number of log entries to return (default: 100)
        
    Returns:
        JSON: List of log entries ordered by timestamp (newest first)
    """
    limit = request.args.get('limit', 100, type=int)
    
    try:
        with DatabaseConnection() as conn:
            c = conn.cursor()
            
            logs = c.execute('''SELECT * FROM system_logs 
                               ORDER BY timestamp DESC LIMIT ?''', (limit,)).fetchall()
        
        logs_list = []
        for log in logs:
            logs_list.append({
                'id': log[0],
                'timestamp': log[1],
                'deviceName': log[2],
                'logType': log[3],
                'message': log[4]
            })
        
        return jsonify(logs_list)
    except Exception as e:
        print(f"Error in get_system_logs: {str(e)}")
        return jsonify({'error': str(e)}), 500


# Web Dashboard Template
# Complete HTML/CSS/JavaScript template for the integrated attendance system dashboard
DASHBOARD_TEMPLATE = '''

<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Integrated Attendance System</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/css/bootstrap.min.css" rel="stylesheet">
    <link href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css" rel="stylesheet">
<style>
    .status-online { color: #28a745; }
    .status-offline { color: #dc3545; }
    .card-header { background-color: #f8f9fa; }
    .log-entry { font-family: monospace; font-size: 0.85em; }
    .device-card { transition: all 0.3s ease; }
    .device-card:hover { 
        transform: translateY(-2px); 
        box-shadow: 0 4px 8px rgba(0,0,0,0.1); 
    }
    .people-count-highlight {
        background: linear-gradient(45deg, #28a745, #20c997);
        color: white;
        padding: 5px 10px;
        border-radius: 15px;
        font-size: 0.9em;
        font-weight: bold;
    }
    .people-count-zero {
        color: #6c757d;
        font-style: italic;
    }
</style>
</head>
<body>
    <nav class="navbar navbar-expand-lg navbar-dark bg-primary">
        <div class="container">
            <a class="navbar-brand" href="#">
                <i class="fas fa-graduation-cap me-2"></i>
                Attendance System
            </a>
            <span class="navbar-text">
                <i class="fas fa-clock me-1"></i>
                <span id="current-time"></span>
            </span>
        </div>
    </nav>

    <div class="container mt-4">
        <!-- Statistics Cards -->
        <div class="row mb-4">
            <div class="col-md-3">
                <div class="card text-white bg-primary">
                    <div class="card-body">
                        <div class="d-flex justify-content-between">
                            <div>
                                <h6 class="card-title">Total Students</h6>
                                <h4 id="total-students">0</h4>
                            </div>
                            <div class="align-self-center">
                                <i class="fas fa-users fa-2x"></i>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
            <div class="col-md-3">
                <div class="card text-white bg-success">
                    <div class="card-body">
                        <div class="d-flex justify-content-between">
                            <div>
                                <h6 class="card-title">Today's Attendance</h6>
                                <h4 id="today-attendance">0</h4>
                            </div>
                            <div class="align-self-center">
                                <i class="fas fa-check-circle fa-2x"></i>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
            <div class="col-md-3">
                <div class="card text-white bg-info">
                    <div class="card-body">
                        <div class="d-flex justify-content-between">
                            <div>
                                <h6 class="card-title">People Present</h6>
                                <h4 id="people-count">0</h4>
                            </div>
                            <div class="align-self-center">
                                <i class="fas fa-user-friends fa-2x"></i>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
            <div class="col-md-3">
                <div class="card text-white bg-warning">
                    <div class="card-body">
                        <div class="d-flex justify-content-between">
                            <div>
                                <h6 class="card-title">Active Devices</h6>
                                <h4 id="active-devices">0</h4>
                            </div>
                            <div class="align-self-center">
                                <i class="fas fa-microchip fa-2x"></i>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>

        <!-- Tabs -->
        <ul class="nav nav-tabs" id="mainTabs" role="tablist">
            <li class="nav-item" role="presentation">
                <button 
                    class="nav-link active"
                    id="attendance-tab"
                    data-bs-toggle="tab"
                    data-bs-target="#attendance"
                        type="button"
                >
                    <i class="fas fa-clipboard-list me-1"></i>
                    Attendance
                </button>
            </li>
            <li class="nav-item" role="presentation">
                <button 
                    class="nav-link" 
                    id="students-tab" 
                    data-bs-toggle="tab" 
                    data-bs-target="#students" 
                    type="button">
                    <i class="fas fa-users me-1"></i>Students
                </button>
            </li>
            <li class="nav-item" role="presentation">
                <button
                    class="nav-link"
                    id="devices-tab"
                    data-bs-toggle="tab"
                    data-bs-target="#devices"
                    type="button"
                >
                    <i class="fas fa-microchip me-1"></i>Devices
                </button>
            </li>
            <li class="nav-item" role="presentation">
                <button 
                    class="nav-link" 
                    id="firmware-tab" 
                    data-bs-toggle="tab" 
                    data-bs-target="#firmware" 
                    type="button">
                    <i class="fas fa-download me-1"></i>Firmware
                </button>
            </li>
            <li class="nav-item" role="presentation">
                <button class="nav-link"
                        id="logs-tab"
                        data-bs-toggle="tab"
                        data-bs-target="#logs"
                        type="button">
                    <i class="fas fa-file-alt me-1"></i>Logs
                </button>
            </li>
        </ul>

        <div class="tab-content mt-3" id="mainTabContent">
            <!-- Attendance Tab -->
            <div class="tab-pane fade show active"
                 id="attendance"
                 role="tabpanel">
                <div class="card">
                    <div class="card-header">
                        <h5 class="mb-0">
                            <i class="fas fa-clipboard-list me-2"></i>
                            Recent Attendance
                        </h5>
                    </div>
                    <div class="card-body">
                        <div class="table-responsive">
                            <table class="table table-striped" id="attendance-table">
                                <thead>
                                    <tr>
                                        <th>Student Name</th>
                                        <th>Student Number</th>
                                        <th>Time</th>
                                        <th>Device IP</th>
                                    </tr>
                                </thead>
                                <tbody></tbody>
                            </table>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Students Tab -->
            <div class="tab-pane fade" id="students" role="tabpanel">
                <div class="card">
                    <div class="card-header d-flex justify-content-between align-items-center">
                        <h5 class="mb-0"><i class="fas fa-users me-2"></i>Students Management</h5>
                        <button class="btn btn-primary btn-sm" data-bs-toggle="modal" data-bs-target="#addStudentModal">
                            <i class="fas fa-plus me-1"></i>Add Student
                        </button>
                    </div>
                    <div class="card-body">
                        <div class="table-responsive">
                            <table class="table table-striped" id="students-table">
                                <thead>
                                    <tr>
                                        <th>UID</th>
                                        <th>Name</th>
                                        <th>Student Number</th>
                                        <th>Professor</th>
                                        <th>Register Date</th>
                                    </tr>
                                </thead>
                                <tbody></tbody>
                            </table>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Devices Tab -->
            <div class="tab-pane fade" id="devices" role="tabpanel">
                <div class="card">
                    <div class="card-header">
                        <h5 class="mb-0"><i class="fas fa-microchip me-2"></i>Device Management</h5>
                    </div>
                    <div class="card-body">
                        <div class="row" id="devices-container"></div>
                    </div>
                </div>
            </div>

            <!-- Firmware Tab -->
            <div class="tab-pane fade" id="firmware" role="tabpanel">
                <div class="card">
                    <div class="card-header">
                        <h5 class="mb-0"><i class="fas fa-download me-2"></i>Firmware Management</h5>
                    </div>
                    <div class="card-body">
                        <div class="mb-3">
                            <label for="firmwareFile" class="form-label">Upload Firmware (.bin file)</label>
                            <input class="form-control" type="file" id="firmwareFile" accept=".bin">
                            <button class="btn btn-primary mt-2" onclick="uploadFirmware()">
                                <i class="fas fa-upload me-1"></i>Upload
                            </button>
                        </div>
                        <div class="table-responsive">
                            <table class="table table-striped" id="firmware-table">
                                <thead>
                                    <tr>
                                        <th>Filename</th>
                                        <th>Size</th>
                                        <th>Upload Date</th>
                                    </tr>
                                </thead>
                                <tbody></tbody>
                            </table>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Logs Tab -->
            <div class="tab-pane fade" id="logs" role="tabpanel">
                <div class="card">
                    <div class="card-header">
                        <h5 class="mb-0"><i class="fas fa-file-alt me-2"></i>System Logs</h5>
                    </div>
                    <div class="card-body">
                        <div id="logs-container" style="max-height: 400px; overflow-y: auto;"></div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <!-- Add Student Modal -->
    <div class="modal fade" id="addStudentModal" tabindex="-1">
        <div class="modal-dialog">
            <div class="modal-content">
                <div class="modal-header">
                    <h5 class="modal-title">Add New Student</h5>
                    <button type="button" class="btn-close" data-bs-dismiss="modal"></button>
                </div>
                <div class="modal-body">
                    <form id="addStudentForm">
                        <div class="mb-3">
                            <label for="uid" class="form-label">UID</label>
                            <input type="text" class="form-control" id="uid" required>
                        </div>
                        <div class="mb-3">
                            <label for="firstName" class="form-label">First Name</label>
                            <input type="text" class="form-control" id="firstName" required>
                        </div>
                        <div class="mb-3">
                            <label for="lastName" class="form-label">Last Name</label>
                            <input type="text" class="form-control" id="lastName" required>
                        </div>
                        <div class="mb-3">
                            <label for="studentNumber" class="form-label">Student Number</label>
                            <input type="text" class="form-control" id="studentNumber" required>
                        </div>
                        <div class="mb-3">
                            <label for="professor" class="form-label">Professor</label>
                            <input type="text" class="form-control" id="professor" required>
                        </div>
                    </form>
                </div>
                <div class="modal-footer">
                    <button type="button" class="btn btn-secondary" data-bs-dismiss="modal">Cancel</button>
                    <button type="button" class="btn btn-primary" onclick="addStudent()">Add Student</button>
                </div>
            </div>
        </div>
    </div>

    <script src="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/js/bootstrap.bundle.min.js"></script>
    <script>
        /*
         * ESP32 Attendance System Dashboard JavaScript
         * Handles real-time data loading, device management, and user interactions
         */
        
        // Global Variables
        let devices = []; // Cached device list for firmware management
        
        /**
         * Load device data for firmware management operations
         * Updates the global devices array with current device status
         */
        async function loadDevicesForFirmware() {
            const response = await fetch('/api/devices');
            devices = await response.json();
        }
        
        /**
         * Update the current time display in the navigation bar
         * Shows local system time in user's timezone
         */
        function updateTime() {
            const now = new Date();
            document.getElementById('current-time').textContent = now.toLocaleString();
        }
        
        // Initialize time display and update every second
        updateTime();
        setInterval(updateTime, 1000);

        /**
         * Calculate today's date string in Iran timezone
         * Converts local time to Iran Standard Time (+3:30 UTC)
         * Used for filtering today's attendance records
         * 
         * @returns {string} Date string in YYYY-MM-DD format (Iran timezone)
         */
        function getIranTodayDateString() {
            // Iran timezone offset is +3:30 or +4:30 (with DST)
            // We'll use +3:30 for simplicity, or you can use a library like moment.js with timezone support
            const now = new Date();
            // Calculate Iran offset in minutes (+3:30 = 210)
            const iranOffset = 3.5 * 60;
            // Get UTC time in ms, add Iran offset in ms
            const iranTime = new Date(now.getTime() + (iranOffset - now.getTimezoneOffset()) * 60000);
            return iranTime.toISOString().split('T')[0];
        }
        
        /*
         * Data Loading Functions
         * Handle fetching and displaying data from the backend API
         */
        
        /**
         * Load and display attendance records
         * Fetches recent attendance data and updates the attendance table
         * Also calculates and updates today's attendance count in the statistics card
         */
        async function loadAttendance() {
            try {
                const response = await fetch('/api/attendance');
                const data = await response.json();
                const tbody = document.querySelector('#attendance-table tbody');
                tbody.innerHTML = '';
                
                // Populate attendance table with records
                data.forEach(record => {
                    const row = tbody.insertRow();
                    row.innerHTML = `
                        <td>${record.studentName}</td>
                        <td>${record.studentNumber}</td>
                        <td>${record.attendanceTime}</td>
                        <td>${record.deviceIP}</td>
                    `;
                });
                
                // Calculate today's attendance count for statistics
                const today = getIranTodayDateString();
                const todayCount = data.filter(r => {
                    if (!r.attendanceTime) return false;
                    const recordDatePart = r.attendanceTime.split(' ')[0];
                    return recordDatePart === today;
                }).length;
                
                // Update the today's attendance counter in statistics card
                document.getElementById('today-attendance').textContent = todayCount;
            } catch (error) {
                console.error('Error loading attendance:', error);
            }
        }
        
        /**
         * Load and display student records
         * Fetches all registered students and updates the students table
         * Also updates the total students count in the statistics card
         */
        async function loadStudents() {
            try {
                const response = await fetch('/api/students');
                const data = await response.json();
                const tbody = document.querySelector('#students-table tbody');
                tbody.innerHTML = '';
                
                // Populate students table with student records
                data.forEach(student => {
                    const row = tbody.insertRow();
                    row.innerHTML = `
                        <td>${student.uid}</td>
                        <td>${student.firstName} ${student.lastName}</td>
                        <td>${student.studentNumber}</td>
                        <td>${student.professor}</td>
                        <td>${student.registerDate || 'N/A'}</td>
                    `;
                });
                
                // Update total students count in statistics card
                document.getElementById('total-students').textContent = data.length;
            } catch (error) {
                console.error('Error loading students:', error);
            }
        }

        /*
         * Device Management Functions
         * Handle device operations like deletion, status checks, and rebooting
         */
        
        /**
         * Delete a device entry from the system
         * Shows confirmation dialog and removes device from JSON storage
         * 
         * @param {string} deviceName - Name of the device to delete
         */
        async function deleteDevice(deviceName) {
            if (confirm(`Are you sure you want to delete the device entry '${deviceName}'? This action cannot be undone.`)) {
                try {
                    // Encode the deviceName in case it contains special characters like '/'
                    const response = await fetch(`/api/devices/${encodeURIComponent(deviceName)}`, {
                        method: 'DELETE'
                    });
                    const result = await response.json(); // Try to parse JSON regardless of status
                    if (response.ok) {
                        alert(result.message || 'Device deleted successfully.');
                        loadDevices(); // Refresh the device list
                    } else {
                        alert('Error: ' + (result.error || `Failed to delete device (status: ${response.status}).`));
                    }
                } catch (error) {
                    console.error('Error deleting device:', error);
                    alert('Error deleting device: ' + error.message);
                }
            }
        }

        /**
         * Load and display device information with real-time status
         * Fetches all devices and displays them as cards with status indicators
         * Calculates statistics: active devices count and total people count from slave devices
         * Shows people count only for slave devices (not master devices)
         */
        async function loadDevices() {
            try {
                const response = await fetch('/api/devices');
                const data = await response.json();
                const container = document.getElementById('devices-container');
                container.innerHTML = '';
                
                // Initialize counters for statistics
                let activeCount = 0;
                let totalPeopleCount = 0;
                
                // Process each device and create display cards
                data.forEach(device => {
                    // Count online devices for statistics
                    if (device.status === 'online') activeCount++;
                    
                    // Only sum up peopleCount from slave devices (people counting sensors)
                    if (device.deviceType === 'slave') {
                        if (typeof device.peopleCount === 'number') {
                            totalPeopleCount += device.peopleCount;
                        } else if (device.peopleCount && !isNaN(Number(device.peopleCount))) {
                            totalPeopleCount += Number(device.peopleCount);
                        }
                    }
                    
                    // Create device card element with status and controls
                    const deviceCard = document.createElement('div');
                    deviceCard.className = 'col-md-6 col-lg-4 mb-3';
                    deviceCard.innerHTML = `
                        <div class="card device-card">
                            <div class="card-body">
                                <h6 class="card-title">
                                    <i class="fas fa-microchip me-2"></i>${device.deviceName}
                                    <span class="badge ${device.status === 'online' ? 'bg-success' : 'bg-danger'} ms-2">
                                        ${device.status}
                                    </span>
                                    ${device.deviceType ? 
                                        `<span class="badge bg-secondary ms-1">${device.deviceType}</span>` : ''}
                                </h6>
                                <p class="card-text">
                                    <small class="text-muted">
                                        IP: ${device.ipAddress}<br>
                                        Last Seen: ${device.lastSeen}<br>
                                        ${device.freeHeap ? 'Free Heap: ' + device.freeHeap + ' bytes<br>' : ''}
                                        ${device.deviceType === 'slave' && device.peopleCount !== undefined ? 
                                            '<strong class="people-count-highlight">People Count: ' + device.peopleCount + '</strong><br>' : ''}
                                    </small>
                                </p>
                                <div class="btn-group btn-group-sm" role="group">
                                    <button class="btn btn-outline-primary" onclick="getDeviceStatus('${device.ipAddress}')">
                                        <i class="fas fa-info-circle"></i> Status
                                    </button>
                                    <button class="btn btn-outline-warning" onclick="rebootDevice('${device.ipAddress}')">
                                        <i class="fas fa-power-off"></i> Reboot
                                    </button>
                                    <button class="btn btn-outline-danger" onclick="deleteDevice('${device.deviceName}')">
                                        <i class="fas fa-trash"></i> Delete
                                    </button>
                                </div>
                            </div>
                        </div>
                    `;
                    container.appendChild(deviceCard);
                });
                
                // Update statistics cards with calculated values
                document.getElementById('active-devices').textContent = activeCount;
                // Update the people-count card with total from slave devices only
                document.getElementById('people-count').textContent = totalPeopleCount;
            } catch (error) {
                console.error('Error loading devices:', error);
            }
        }

        /**
         * Update the total people count display
         * Updates the people count statistics card with aggregated data
         * 
         * @param {number} totalPeople - Total count of people from all devices
         */
        function updateTotalPeopleCount(totalPeople) {
            // Check if we have a people count card in the statistics section
            let peopleCountCard = document.getElementById('people-count');
            if (!peopleCountCard) {
                // If no people count card exists, we can add one or update an existing card
                // For now, let's just log it
                console.log(`Total people across all devices: ${totalPeople}`);
            } else {
                peopleCountCard.textContent = totalPeople;
            }
        }
        
        /*
         * Firmware Management Functions
         * Handle firmware loading, uploading, and OTA (Over-The-Air) updates
         */
        
        /**
         * Load and display available firmware files with device selection
         * Fetches firmware list and creates interactive table with push capabilities
         * Includes robust error handling for JSON parsing and network issues
         */
        async function loadFirmware() {
            try {
                // Load devices first for firmware push functionality
                await loadDevicesForFirmware();

                const response = await fetch('/api/firmware');
                
                // Validate response before parsing JSON
                if (!response.ok) {
                    const errorText = await response.text();
                    throw new Error(`Server error (${response.status}): ${errorText}`);
                }
                
                // Ensure response is actually JSON
                const contentType = response.headers.get("content-type");
                if (!contentType || !contentType.includes("application/json")) {
                    const text = await response.text();
                    console.error("Non-JSON response:", text);
                    throw new Error("Server did not return JSON");
                }
                
                const data = await response.json();
                const tbody = document.querySelector('#firmware-table tbody');
                tbody.innerHTML = '';

                // Handle empty firmware list
                if (!data || data.length === 0) {
                    tbody.innerHTML = '<tr><td colspan="4" class="text-center">No firmware files available</td></tr>';
                    return;
                }

                // Create table rows for each firmware file
                data.forEach(firmware => {
                    const row = tbody.insertRow();
                    
                    // Build dropdown of online devices for firmware push
                    let deviceOptions = devices
                        .filter(d => d.status === 'online')
                        .map(d => `<option value="${d.ipAddress}">${d.deviceName} (${d.ipAddress})</option>`)
                        .join('');
                    
                    // Handle case when no devices are online
                    if (!deviceOptions) {
                        deviceOptions = '<option value="">No online devices</option>';
                    }
                    
                    // Create firmware table row with push functionality
                    row.innerHTML = `
                        <td>${firmware.filename}</td>
                        <td>${(firmware.size / 1024).toFixed(2)} KB</td>
                        <td>${firmware.uploadDate}</td>
                        <td>
                            <select class="form-select form-select-sm" id="device-select-${firmware.filename}">
                                ${deviceOptions}
                            </select>
                            <button class="btn btn-sm btn-success mt-1" onclick="pushFirmwareToDevice('${firmware.filename}', document.getElementById('device-select-${firmware.filename}').value)">
                                Push to Device
                            </button>
                        </td>
                    `;
                });
            } catch (error) {
                console.error('Error loading firmware:', error);
                document.querySelector('#firmware-table tbody').innerHTML = 
                    `<tr><td colspan="4" class="text-center text-danger">Error loading firmware: ${error.message}</td></tr>`;
            }
        }

        /**
         * Push firmware to selected ESP32 device via OTA
         * Initiates Over-The-Air firmware update with real-time progress monitoring
         * Handles button state changes and progress feedback
         * 
         * @param {string} filename - Name of the firmware file to push
         * @param {string} deviceIP - IP address of the target device
         */
        async function pushFirmwareToDevice(filename, deviceIP) {
            if (!deviceIP) {
                alert("Please select a device first");
                return;
            }
            
            // Update button to show loading state
            const button = event.target;
            const originalText = button.innerHTML;
            button.disabled = true;
            button.innerHTML = '<i class="fas fa-spinner fa-spin"></i> Pushing...';
            
            try {
                // Prepare form data for firmware push request
                const formData = new FormData();
                formData.append('device_ip', deviceIP);
                formData.append('filename', filename);
                
                console.log(`🔄 Starting firmware push: ${filename} to ${deviceIP}`);
                
                // Initiate firmware push via API
                const response = await fetch('/api/firmware/push', {
                    method: 'POST',
                    body: formData
                });
                
                // Handle server errors
                if (!response.ok) {
                    const errorData = await response.json();
                    throw new Error(`Server error (${response.status}): ${errorData.error}`);
                }
                
                const result = await response.json();
                console.log('✅ Firmware push result:', result);
                
                // Handle different response states
                if (result.status === 'in_progress') {
                    // OTA started successfully, begin monitoring
                    button.innerHTML = '<i class="fas fa-spinner fa-spin"></i> Monitoring...';
                    await monitorOTAProgress(deviceIP, filename, button);
                } else {
                    // OTA completed immediately or failed
                    alert(result.message || "Firmware pushed successfully");
                    button.innerHTML = originalText;
                    button.disabled = false;
                }
                
            } catch (error) {
                console.error('❌ Error pushing firmware:', error);
                alert('Error: ' + error.message);
                button.innerHTML = originalText;
                button.disabled = false;
            }
        }

        /**
         * Monitor OTA (Over-The-Air) update progress in real-time
         * Polls device for progress updates and provides visual feedback
         * Handles completion, failure, and timeout scenarios
         * 
         * @param {string} deviceIP - IP address of the device being updated
         * @param {string} filename - Name of the firmware file being pushed
         * @param {HTMLElement} button - Button element to update with progress
         */
        async function monitorOTAProgress(deviceIP, filename, button) {
            const maxAttempts = 60; // Maximum monitoring time: 2 minutes
            let attempts = 0;
            
            /**
             * Recursive function to check OTA progress
             * Polls device every 2 seconds for progress updates
             */
            const checkProgress = async () => {
                try {
                    const response = await fetch(`/api/firmware/progress/${deviceIP}`);
                    
                    if (response.ok) {
                        const progress = await response.json();
                        
                        const percent = progress.percent || 0;
                        const status = progress.status || 'Unknown';
                        
                        // Update button with current progress percentage
                        button.innerHTML = `<i class="fas fa-download"></i> ${percent}% - ${status}`;
                        
                        console.log(`📊 OTA Progress: ${percent}% - ${status}`);
                        
                        // Check if OTA operation completed
                        if (!progress.inProgress) {
                            if (percent >= 100) {
                                // OTA completed successfully
                                button.innerHTML = '<i class="fas fa-check"></i> Completed!';
                                button.className = 'btn btn-sm btn-success mt-1';
                                alert(`✅ Firmware ${filename} uploaded successfully!`);
                                
                                // Reset button to original state after 3 seconds
                                setTimeout(() => {
                                    button.innerHTML = 'Push to Device';
                                    button.className = 'btn btn-sm btn-success mt-1';
                                    button.disabled = false;
                                }, 3000);
                                
                                return; // Stop monitoring
                            } else {
                                // OTA failed
                                button.innerHTML = '<i class="fas fa-times"></i> Failed!';
                                button.className = 'btn btn-sm btn-danger mt-1';
                                alert(`❌ Firmware upload failed!`);
                                
                                // Reset button to original state after 3 seconds
                                setTimeout(() => {
                                    button.innerHTML = 'Push to Device';
                                    button.className = 'btn btn-sm btn-success mt-1';
                                    button.disabled = false;
                                }, 3000);
                                
                                return; // Stop monitoring
                            }
                        }
                        
                    } else {
                        console.log('⚠️ Progress check failed, device may be rebooting...');
                    }
                    
                    attempts++;
                    if (attempts < maxAttempts) {
                        setTimeout(checkProgress, 2000); // Check every 2 seconds
                    } else {
                        button.innerHTML = '<i class="fas fa-question"></i> Timeout';
                        button.className = 'btn btn-sm btn-warning mt-1';
                        alert('⏱️ OTA progress monitoring timed out. Check device manually.');
                        
                        // Reset button after 3 seconds
                        setTimeout(() => {
                            button.innerHTML = 'Push to Device';
                            button.className = 'btn btn-sm btn-success mt-1';
                            button.disabled = false;
                        }, 3000);
                    }
                    
                } catch (error) {
                    console.error('Error monitoring progress:', error);
                    attempts++;
                    if (attempts < maxAttempts) {
                        setTimeout(checkProgress, 2000);
                    }
                }
            };
            
            // Start initial progress check after 1 second delay
            setTimeout(checkProgress, 1000);
        }
        
        /**
         * Load and display system logs
         * Fetches recent system events and displays them in chronological order
         * Each log entry shows timestamp, device name, event type, and message
         */
        async function loadLogs() {
            try {
                const response = await fetch('/api/logs');
                const data = await response.json();
                const container = document.getElementById('logs-container');
                container.innerHTML = '';
                
                // Create log entry elements for each log record
                data.forEach(log => {
                    const logEntry = document.createElement('div');
                    logEntry.className = 'log-entry mb-2 p-2 border-bottom';
                    logEntry.innerHTML = `
                        <span class="text-muted">[${log.timestamp}]</span>
                        <span class="badge bg-secondary">${log.deviceName}</span>
                        <span class="badge bg-info">${log.logType}</span>
                        ${log.message}
                    `;
                    container.appendChild(logEntry);
                });
                
                // Update total logs count if element exists
                const totalLogsElement = document.getElementById('total-logs');
                if (totalLogsElement) {
                    totalLogsElement.textContent = data.length;
                }
            } catch (error) {
                console.error('Error loading logs:', error);
            }
        }

        /*
         * Device Control Functions
         * Handle direct device operations like rebooting and status checking
         */
        
        /**
         * Reboot an ESP32 device remotely
         * Shows confirmation dialog before sending reboot command
         * 
         * @param {string} deviceIP - IP address of the device to reboot
         */
        async function rebootDevice(deviceIP) {
            if (confirm(`Are you sure you want to reboot device ${deviceIP}?`)) {
                try {
                    const response = await fetch(`/api/device/${deviceIP}/reboot`, { method: 'POST' });
                    const result = await response.json();
                    alert(result.message || result.error);
                    loadDevices(); // Refresh device list after reboot
                } catch (error) {
                    alert('Error: ' + error.message);
                }
            }
        }

        /**
         * Trigger LoRa OTA mode for slave devices (Legacy function)
         * This function is kept for compatibility but may not be actively used
         * 
         * @param {string} deviceIP - IP address of the slave device
         */
        async function triggerSlaveOTA(deviceIP) {
            try {
                const response = await fetch(`/api/device/${deviceIP}/trigger_slave_ota`, { method: 'POST' });
                const result = await response.json();
                alert(result.message || result.error);
            } catch (error) {
                alert('Error: ' + error.message);
            }
        }

        /**
         * Get detailed status information from an ESP32 device
         * Fetches device health data including uptime, memory, and OTA status
         * Displays comprehensive device information in an alert dialog
         * 
         * @param {string} deviceIP - IP address of the device to query
         */
        async function getDeviceStatus(deviceIP) {
            try {
                const response = await fetch(`/api/device/${deviceIP}/ota_status`);
                const result = await response.json();
                
                if (response.ok) {
                    // Format device status information for display
                    let statusMsg = `Device Status:\n`;
                    
                    // Add device health information if available
                    if (result.device_status) {
                        const ds = result.device_status;
                        statusMsg += `Uptime: ${ds.uptime}ms\n`;
                        statusMsg += `Free Heap: ${ds.freeHeap} bytes\n`;
                        statusMsg += `Last UID: ${ds.lastUID || 'None'}\n`;
                        statusMsg += `OTA Status: ${ds.otaStatus || 'Ready'}\n`;
                    }
                    
                    // Add OTA progress information if available
                    if (result.ota_progress) {
                        const op = result.ota_progress;
                        statusMsg += `\nOTA Progress:\n`;
                        statusMsg += `In Progress: ${op.inProgress ? 'Yes' : 'No'}\n`;
                        if (op.inProgress) {
                            statusMsg += `Progress: ${op.percent}%\n`;
                            statusMsg += `Bytes: ${op.progress}/${op.total}\n`;
                        }
                    }
                    
                    alert(statusMsg);
                } else {
                    alert('Error: ' + result.error);
                }
                
                loadDevices(); // Refresh device list after status check
            } catch (error) {
                alert('Error: ' + error.message);
            }
        }

        /*
         * Student Management Functions
         * Handle student registration and data validation
         */
        
        /**
         * Add a new student to the system
         * Validates form data and submits student information to the server
         * Handles success/error responses and refreshes the student list
         */
        async function addStudent() {
            // Collect form data from the student registration modal
            const formData = {
                uid: document.getElementById('uid').value,
                firstName: document.getElementById('firstName').value,
                lastName: document.getElementById('lastName').value,
                studentNumber: document.getElementById('studentNumber').value,
                professor: document.getElementById('professor').value
            };

            try {
                // Submit student data to the server
                const response = await fetch('/api/students', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(formData)
                });
                
                const result = await response.json();
                if (response.ok) {
                    alert('Student added successfully!');
                    // Clear form and close modal on success
                    document.getElementById('addStudentForm').reset();
                    bootstrap.Modal.getInstance(document.getElementById('addStudentModal')).hide();
                    loadStudents(); // Refresh student list
                } else {
                    alert('Error: ' + result.error);
                }
            } catch (error) {
                alert('Error: ' + error.message);
            }
        }

        /*
         * File Upload Functions
         * Handle firmware file uploads with validation
         */
        
        /**
         * Upload firmware file to the server
         * Validates file selection, creates form data, and submits to upload endpoint
         * Handles success/error responses and refreshes firmware list
         * Only accepts .bin firmware files for ESP32 devices
         * 
         * @returns {Promise<void>} Resolves when upload completes or fails
         */
        async function uploadFirmware() {
            // Get the file input element and selected file
            const fileInput = document.getElementById('firmwareFile');
            const file = fileInput.files[0];
            
            // Validate that a file has been selected
            if (!file) {
                alert('Please select a firmware file');
                return;
            }

            // Create FormData object for multipart file upload
            const formData = new FormData();
            formData.append('firmware', file);

            try {
                // Submit the firmware file to the upload endpoint
                const response = await fetch('/api/firmware/upload', {
                    method: 'POST',
                    body: formData // FormData handles Content-Type automatically
                });
                
                // Parse the JSON response from the server
                const result = await response.json();
                if (response.ok) {
                    // Upload successful - notify user and refresh UI
                    alert('Firmware uploaded successfully!');
                    fileInput.value = ''; // Clear the file input
                    loadFirmware(); // Refresh the firmware list table
                } else {
                    // Upload failed - show server error message
                    alert('Error: ' + result.error);
                }
            } catch (error) {
                // Network or parsing error - show generic error message
                alert('Error: ' + error.message);
            }
        }

        /*
         * Dashboard Initialization and Control Functions
         * Handle dashboard startup, auto-refresh intervals, and tab navigation
         */

        /**
         * Initialize the dashboard with all data sections
         * Loads all required data when the page first loads
         * Called once on page load to populate all tabs with initial data
         * 
         * This function ensures all dashboard sections are populated:
         * - Attendance records for today's activity
         * - Student list for management
         * - Device status and controls
         * - Available firmware files
         * - Recent system logs
         */
        function initDashboard() {
            loadAttendance(); // Load recent attendance records
            loadStudents();   // Load registered students
            loadDevices();    // Load device status and info
            loadFirmware();   // Load available firmware files
            loadLogs();       // Load system event logs
        }

        /*
         * Dashboard Auto-Refresh System
         * Implements different refresh intervals for different data types
         * to balance real-time updates with server performance
         */

        // Initial dashboard load when page is ready
        initDashboard();
        
        /**
         * High-frequency refresh interval (10 seconds)
         * Updates frequently changing data that users need to see in real-time:
         * - New attendance entries as students scan cards
         * - Device status changes (online/offline)
         * - System logs and events
         */
        setInterval(() => {
            loadAttendance(); // Check for new attendance entries
            loadDevices();    // Update device status and people counts
            loadLogs();       // Load recent system events
        }, 10000); // 10 seconds - frequent updates for real-time data
        
        /**
         * Low-frequency refresh interval (30 seconds)
         * Updates less frequently changing data to reduce server load:
         * - Student list (rarely changes during operation)
         * - Firmware files (only updated when new files are uploaded)
         */
        setInterval(() => {
            loadStudents(); // Refresh student database
            loadFirmware(); // Check for new firmware files
        }, 30000); // 30 seconds - less frequent for static data

        /*
         * Tab Navigation Event Listeners
         * Refresh data when users switch between dashboard tabs
         * Ensures users see current data when they navigate to different sections
         */

        // Students tab - refresh student data when tab becomes visible
        document.getElementById('students-tab').addEventListener('shown.bs.tab', loadStudents);
        
        // Devices tab - refresh device status when tab becomes visible
        document.getElementById('devices-tab').addEventListener('shown.bs.tab', loadDevices);
        
        // Firmware tab - refresh firmware list when tab becomes visible
        document.getElementById('firmware-tab').addEventListener('shown.bs.tab', loadFirmware);
        
        // Logs tab - refresh system logs when tab becomes visible
        document.getElementById('logs-tab').addEventListener('shown.bs.tab', loadLogs);
    </script>
</body>
</html>
'''


"""
Main application entry point.

Initializes the database, loads initial student data from JSON,
and starts the Flask development server with CORS enabled.
The server is accessible from all network interfaces on port 5001.
"""
if __name__ == '__main__':

    # Initialize database tables and schema
    init_db()
    
    # Load students from JSON file if it exists
    try:
        load_students_from_json()
        print("✅ Students loaded successfully from JSON")
    except Exception as e:
        print(f"⚠️ Error loading students: {str(e)}")
    
    # Start the Flask development server
    print("🚀 Starting Integrated Attendance System...")
    print("🌐 Access dashboard at: http://localhost:5001")
    print("📱 ESP32 devices can connect to: http://<server-ip>:5001")
    
    # Run server on all interfaces with debug mode enabled
    app.run(host='0.0.0.0', port=5001, debug=True)