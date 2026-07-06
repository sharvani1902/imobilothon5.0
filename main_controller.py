"""
main_controller.py

Host-side main controller for the Integrated Driver Wellness System.
Runs on a host machine (e.g., Raspberry Pi, PC) and performs two main tasks concurrently:
1.  **Vitals GUI (Main Thread):** Renders a Tkinter dashboard to simulate and display
    driver vitals (HR, SpO2). Sends emergency signals to the ESP32 if vitals
    cross critical thresholds.
2.  **Vision Processing (Worker Thread):** Uses OpenCV and MediaPipe to monitor
    the driver's face for signs of fatigue (microsleep, high PERCLOS, yawning)
    and distraction. Sends fatigue alerts to the ESP32.

Communicates with the ESP32 vehicle controller via Serial.
"""

import tkinter as tk
from tkinter import ttk
import time
import math
import sys
import requests
import serial
from PIL import Image, ImageDraw, ImageFilter, ImageTk
import cv2
import mediapipe as mp
from collections import deque
from picamera2 import Picamera2
import subprocess
import threading

# ==================================================================
# === Vitals & GUI Configuration
# ==================================================================

# Serial port for communicating with the ESP32.
# e.g., '/dev/ttyUSB0' (Linux/Pi) or 'COM3' (Windows)
SERIAL_PORT = "/dev/ttyUSB0"
ser = None  # Serial connection object
serial_buffer = b""  # Buffer for incoming serial data
# Lock to prevent concurrent serial access from GUI and Vision threads
serial_lock = threading.Lock()

# --- Thresholds Based on Driving Safety Guidelines ---
HR_SAFE_MIN = 60
HR_SAFE_MAX = 100
HR_MILD_LOW_MIN = 50
HR_MILD_HIGH_MAX = 120
HR_STOP_LOW_MIN = 40
HR_STOP_HIGH_MAX = 140

SPO2_SAFE_MIN = 95
SPO2_MILD_MIN = 92
SPO2_STOP_MIN = 88

# --- GUI State ---
last_alert = None
last_spo2_alert = None
bg_photo = None
CANVAS_SIZE = 600
CENTER = CANVAS_SIZE // 2

# ==================================================================
# === Vision Configuration
# ==================================================================
SOUND_BREAK = "/home/volkswagon/Downloads/audio/break.mp3"
SOUND_FOCUS = "/home/volkswagon/Downloads/audio/focus.mp3"

# --- MEDIAPIPE SETUP ---
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(
    max_num_faces=1,
    refine_landmarks=True,
    min_detection_confidence=0.6,
    min_tracking_confidence=0.6
)

# --- PICAMERA2 SETUP ---
print("🎥 Initializing Camera...")
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": "BGR888", "size": (640, 480)})
picam2.configure(config)
picam2.start()
print(" Camera Ready")

# --- VISION LANDMARKS AND THRESHOLDS ---
LEFT_EYE  = [33, 160, 158, 133, 153, 144]
RIGHT_EYE = [362, 385, 387, 263, 373, 380]
MOUTH = [13, 14, 78, 38]  # Corrected landmark from 308 to 38
NOSE_TIP = 1

CALIB_SECONDS = 3.0       # Duration for EAR calibration
EYE_CLOSED_TIME = 3.0     # Microsleep threshold (seconds)
MAR_HIGH = 0.72           # Mouth Aspect Ratio threshold for yawn start
MAR_LOW  = 0.55           # Mouth Aspect Ratio threshold for yawn end
MIN_YAWN_DURATION = 0.6   # Minimum time mouth is open to be a valid yawn
YAWN_WINDOW = 60          # Time window (seconds) to count yawns
YAWN_TRIGGER = 3          # Number of yawns in window to trigger alert
HEAD_TURN_THRESHOLD = 0.14 # Normalized X-coordinate change for head turn
HEAD_TURN_TIME = 1.2      # Head turn duration (seconds) to trigger alert
PERCLOS_START_DELAY = 20.0 # Grace period before PERCLOS calculation starts
PERCLOS_THRESHOLD = 0.30   # PERCLOS value (30%) to trigger alert
PERCLOS_ALERT_COOLDOWN = 15.0 # Cooldown (seconds) between PERCLOS alerts

# --- Vision State Variables ---
ear_samples = []
calib_start = time.time()
ear_open_mean = None
ear_closed_th = None
ear_reopen_th = None
eye_closed_start = None
eyes_closed_latch = False
yawn_active = False
yawn_open_start = None
yawn_times = deque()
head_turn_start = None
perclos_total_samples = 0
perclos_closed_samples = 0
perclos_start_time = None
last_perclos_alert_time = 0
eye_status = "---"

# Event to signal the vision thread to terminate gracefully
stop_event = threading.Event()

# ==================================================================
# === Sound & Vision Helper Functions
# ==================================================================

def play_sound(sound_file):
    """Plays a sound file non-blockingly using mpg123."""
    try:
        subprocess.Popen(["mpg123", "-q", sound_file])
        print(f"🔊 Playing: {sound_file.split('/')[-1]}")
    except FileNotFoundError:
        print("⚠ ERROR: 'mpg123' not found. Please install it: sudo apt install mpg123")
    except Exception as e:
        print(f"⚠ Error playing sound: {e}")

def alert_fatigue():
    """Triggers fatigue alert, plays sound, and sends 'F' command to ESP32."""
    print(" FATIGUE ALERT → Playing break.mp3")
    play_sound(SOUND_BREAK)

    # Send 'F' (Fatigue) command to ESP32, using the thread-safe lock
    with serial_lock:
        if ser:
            try:
                ser.write(b'F\n')
                print("Sent FATIGUE signal (F) to ESP32")
            except Exception as e:
                print(f"Serial write error from fatigue alert: {e}")

def alert_distraction():
    """Alert for distraction events (look away)."""
    print(" DISTRACTION ALERT → Playing focus.mp3")
    play_sound(SOUND_FOCUS)

def dist(p1, p2):
    """Calculates Euclidean distance between two MediaPipe landmarks."""
    return math.hypot(p1.x - p2.x, p1.y - p2.y)

def ear(lm, e):
    """Calculates Eye Aspect Ratio (EAR) for a single eye."""
    return (dist(lm[e[1]], lm[e[5]]) + dist(lm[e[2]], lm[e[4]])) / (2 * dist(lm[e[0]], lm[e[3]]))

def mar(lm):
    """Calculates Mouth Aspect Ratio (MAR)."""
    return dist(lm[MOUTH[0]], lm[MOUTH[1]]) / dist(lm[MOUTH[2]], lm[MOUTH[3]])

# ==================================================================
# === Vitals & GUI Helper Functions
# ==================================================================

def create_watch_background(size=CANVAS_SIZE):
    """Generates the main 'watch' background image for the Tkinter GUI."""
    img = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    padding = 20
    x1, y1 = padding, padding
    x2, y2 = size - padding, size - padding
    shadow = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    sd = ImageDraw.Draw(shadow)
    sd.rectangle((x1 + 8, y1 + 8, x2 + 8, y2 + 8), fill=(0, 0, 0, 140))
    shadow = shadow.filter(ImageFilter.GaussianBlur(20))
    img.alpha_composite(shadow)
    for i in range(15, 0, -1):
        brightness = int(80 - (i * 3))
        color = (brightness, brightness, brightness, 255)
        draw.rectangle((x1 - i, y1 - i, x2 + i, y2 + i), outline=color, width=1)
    draw.rectangle((x1, y1, x2, y2), outline=(200, 200, 200, 255), width=4)
    glass = Image.new("RGBA", (size, size), (18, 22, 28, 255))
    img.alpha_composite(glass)
    return ImageTk.PhotoImage(img)

def get_hr_status(hr):
    """Returns status level, color, and message based on heart rate."""
    if HR_SAFE_MIN <= hr <= HR_SAFE_MAX:
        return "safe", "#7CFC00", "Safe to drive"
    elif (HR_MILD_LOW_MIN <= hr < HR_SAFE_MIN) or (HR_SAFE_MAX < hr <= HR_MILD_HIGH_MAX):
        return "mild", "#FFD700", "Mild caution"
    elif (HR_STOP_LOW_MIN <= hr < HR_MILD_LOW_MIN) or (HR_MILD_HIGH_MAX < hr <= HR_STOP_HIGH_MAX):
        return "stop", "#FF8C00", "Stop driving"
    else:
        return "emergency", "#FF0000", "EMERGENCY"

def get_spo2_status(spo2):
    """Returns status level, color, and message based on SpO2."""
    if spo2 >= SPO2_SAFE_MIN:
        return "safe", "#7CFC00", "Normal"
    elif SPO2_MILD_MIN <= spo2 < SPO2_SAFE_MIN:
        return "mild", "#FFD700", "Caution"
    elif SPO2_STOP_MIN <= spo2 < SPO2_MILD_MIN:
        return "stop", "#FF8C00", "Alert"
    else:
        return "emergency", "#FF0000", "EMERGENCY"

def connect_to_esp32():
    """Tries to connect to the serial port after the GUI has started."""
    global ser
    if ser is None:
        try:
            ser = serial.Serial(
                SERIAL_PORT,
                115200,
                timeout=0.1,      # Read timeout
                write_timeout=1.0 # Write timeout
            )
            print(f"Successfully connected to serial port {SERIAL_PORT}")
        except Exception as e:
            print(f"ERROR: Could not open serial port. {e}")
            print("Running in simulation mode without serial connection.")
            ser = None

def process_serial_data():
    """
    Non-blocking read of serial port.
    Reads all available data, adds to buffer, and processes complete lines.
    """
    global serial_buffer, ser

    if ser and ser.in_waiting > 0:
        try:
            new_data = ser.read(ser.in_waiting)
            serial_buffer += new_data
        except Exception as e:
            print(f"Serial read error: {e}")
            ser.close()
            ser = None
            print("Serial connection lost.")
            return

    # Process all complete lines (ending in '\n') in the buffer
    while b'\n' in serial_buffer:
        try:
            line, serial_buffer = serial_buffer.split(b'\n', 1)
            response = line.decode('utf-8').strip()
            if response:
                print(f"ESP32 says: --> {response}")
        except Exception as e:
            print(f"Error processing serial line: {e}")
            serial_buffer = b"" # Clear buffer on error

def update_status(event=None):
    """
    Callback for slider changes.
    Updates GUI colors/text and sends 'E' (Emergency) or 'N' (Normal)
    command to the ESP32 based on vitals.
    """
    global last_alert, ser

    hr = hr_var.get()
    spo2 = spo2_var.get()

    hr_level, hr_color, hr_msg = get_hr_status(hr)
    spo2_level, spo2_color, spo2_msg = get_spo2_status(spo2)

    hud_text = f"HR: {hr} bpm\nSpO2: {spo2}%"
    canvas.itemconfig(digital_text_id, text=hud_text)

    # Prioritize the most severe status for display
    status_priority = {"emergency": 4, "stop": 3, "mild": 2, "safe": 1}

    if status_priority[hr_level] >= status_priority[spo2_level]:
        overall_level, overall_color, status_msg = hr_level, hr_color, hr_msg
    else:
        overall_level, overall_color, status_msg = spo2_level, spo2_color, spo2_msg

    # Update GUI elements
    canvas.itemconfig(digital_text_id, fill=overall_color)
    canvas.itemconfig(status_msg_id, text=status_msg, fill=overall_color)
    canvas.itemconfig(rect_bg_id, outline=overall_color)
    canvas.itemconfig(hr_label, text=f"HR: {hr} bpm")
    canvas.itemconfig(spo2_label, text=f"SpO2: {spo2}%")

    emergency_condition = (spo2 < SPO2_STOP_MIN or hr < HR_STOP_LOW_MIN or hr > HR_STOP_HIGH_MAX)

    # Send 'E' (Emergency) or 'N' (Normal) command based on vitals.
    with serial_lock:
        if ser:
            try:
                if emergency_condition:
                    serial_message = f"E,{hr},{spo2}\n"
                    ser.write(serial_message.encode('utf-8'))
                    if last_alert != "emergency":
                        print(f"Sending VITALS EMERGENCY signal to ESP32: {serial_message.strip()}")
                        last_alert = "emergency"
                else:
                    ser.write(b'N\n')
                    if last_alert != "normal":
                         print("Sending NORMAL signal to ESP32: N")
                    last_alert = "normal"
            except serial.SerialTimeoutException:
                print("Serial write timed out. ESP32 might not be ready.")
            except Exception as e:
                print(f"Serial write error: {e}")
                ser.close()
                ser = None
                print("Serial connection lost.")

    # Poll for serial data
    process_serial_data()

def update_time():
    """Updates the GUI clock and polls for incoming serial data."""
    ts = time.strftime("%I:%M:%S %p")
    canvas.itemconfig(time_text_id, text=ts)

    # Main polling point for incoming serial data
    process_serial_data()

    root.after(1000, update_time)

# ==================================================================
# === Vision Thread Function
# ==================================================================

def vision_loop():
    """
    Main loop for the vision processing thread.
    Handles all OpenCV and MediaPipe logic.
    """
    global ear_samples, calib_start, ear_open_mean, ear_closed_th, ear_reopen_th
    global eye_closed_start, eyes_closed_latch, yawn_active, yawn_open_start, yawn_times
    global head_turn_start, perclos_total_samples, perclos_closed_samples, perclos_start_time
    global last_perclos_alert_time, eye_status, video_label

    print("Vision Thread Started...")

    while not stop_event.is_set():
        try:
            frame = picam2.capture_array()
            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = face_mesh.process(rgb)

            display_frame = frame.copy() # Frame to draw on

            if result.multi_face_landmarks:
                lm = result.multi_face_landmarks[0].landmark

                ear_val = (ear(lm, LEFT_EYE) + ear(lm, RIGHT_EYE)) / 2
                mar_val = mar(lm)
                nose_x = lm[NOSE_TIP].x

                if ear_open_mean is None:
                    # --- CALIBRATION PHASE ---
                    if time.time() - calib_start <= CALIB_SECONDS:
                        if ear_val > 0:
                            ear_samples.append(ear_val)
                        cv2.putText(display_frame, "Calibrating... Keep eyes OPEN", (10,30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                    else:
                        if not ear_samples:
                            print("Calibration failed, no eye data. Retrying...")
                            calib_start = time.time()
                        else:
                            ear_open_mean = sum(ear_samples)/len(ear_samples)
                            ear_closed_th = ear_open_mean * 0.65
                            ear_reopen_th = ear_open_mean * 0.82
                            print(f"🔧 EAR calibrated: open={ear_open_mean:.3f}, closed={ear_closed_th:.3f}, reopen={ear_reopen_th:.3f}")
                            perclos_start_time = time.time()
                            print(f" PERCLOS calculation will display after {PERCLOS_START_DELAY} seconds.")

                else:
                    # --- MONITORING PHASE ---

                    # Eye status
                    if ear_val < ear_closed_th:
                        eye_status = "CLOSED"
                    else:
                        eye_status = "OPEN"

                    # Microsleep (Latching)
                    if not eyes_closed_latch:
                        if eye_status == "CLOSED":
                            if eye_closed_start is None:
                                eye_closed_start = time.time()
                            elif time.time() - eye_closed_start > EYE_CLOSED_TIME:
                                eyes_closed_latch = True
                                eye_closed_start = None
                                # Microsleep detected: Trigger fatigue alert.
                                alert_fatigue()
                        else:
                            eye_closed_start = None
                    else:
                        # Latch resets when eyes are clearly open
                        if ear_val > ear_reopen_th:
                            eyes_closed_latch = False

                    # PERCLOS Calculation
                    current_time = time.time()
                    is_closed_frame = 1 if eye_status == "CLOSED" else 0
                    perclos_total_samples += 1
                    perclos_closed_samples += is_closed_frame

                    perclos_val = 0.0
                    perclos_display = "Warming up..."

                    if current_time - perclos_start_time > PERCLOS_START_DELAY:
                        if perclos_total_samples > 0:
                            perclos_val = perclos_closed_samples / perclos_total_samples
                            perclos_display = f"{perclos_val*100:.0f}%"

                        if perclos_val > PERCLOS_THRESHOLD and current_time - last_perclos_alert_time > PERCLOS_ALERT_COOLDOWN:
                            # High PERCLOS detected: Trigger fatigue alert.
                            alert_fatigue()
                            last_perclos_alert_time = current_time

                    cv2.putText(display_frame, f"S:{eye_status} P:{perclos_display} E:{ear_val:.2f}", (10,30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

                # YAWN DETECTION (Always active post-calibration)
                if not yawn_active and mar_val > MAR_HIGH:
                    yawn_active = True
                    yawn_open_start = time.time()

                if yawn_active and mar_val < MAR_LOW:
                    yawn_active = False
                    if time.time() - yawn_open_start >= MIN_YAWN_DURATION:
                        yawn_times.append(time.time())
                        print(f" Yawn detected. (Count: {len(yawn_times)})")

                # Prune old yawns from the deque
                while yawn_times and time.time() - yawn_times[0] > YAWN_WINDOW:
                    yawn_times.popleft()

                # Check for yawn trigger
                if len(yawn_times) >= YAWN_TRIGGER:
                    # Frequent yawning detected: Trigger fatigue alert.
                    alert_fatigue()
                    yawn_times.clear() # Reset count after alert

                # HEAD TURN (Always active post-calibration)
                if abs(nose_x - 0.5) > HEAD_TURN_THRESHOLD:
                    if head_turn_start is None:
                        head_turn_start = time.time()
                    elif time.time() - head_turn_start > HEAD_TURN_TIME:
                        # Distraction detected: Play warning sound.
                        alert_distraction()
                        head_turn_start = None # Reset timer
                else:
                    head_turn_start = None

            else:
                # No face detected
                eye_status = "---"
                cv2.putText(display_frame, "No Face Detected", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # --- Update GUI Video Feed ---
            # Resize for GUI display
            gui_frame = cv2.resize(display_frame, (320, 240))
            # Convert for Tkinter
            rgb_frame = cv2.cvtColor(gui_frame, cv2.COLOR_BGR_RGB)
            img = Image.fromarray(rgb_frame)
            imgtk = ImageTk.PhotoImage(image=img)

            # Update the label in the main thread
            video_label.imgtk = imgtk
            video_label.configure(image=imgtk)

        except Exception as e:
            print(f"Error in vision loop: {e}")
            time.sleep(1)

        # Yield control to other threads to prevent 100% CPU
        time.sleep(0.01)

    print("Vision Thread Stopped.")
    picam2.stop()
    face_mesh.close()

# ==================================================================
# === MAIN SCRIPT: Build GUI, Start Threads
# ==================================================================

root = tk.Tk()
root.title("Integrated Driver Wellness System")
root.configure(bg="#000000")

# --- Left Frame: Vitals Dashboard ---
vitals_frame = tk.Frame(root, bg="#000000")
vitals_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

canvas = tk.Canvas(vitals_frame, width=CANVAS_SIZE, height=CANVAS_SIZE, bg="#000000", highlightthickness=0)
canvas.pack(fill=tk.BOTH, expand=True)

bg_photo = create_watch_background(CANVAS_SIZE)
canvas.create_image(0, 0, anchor="nw", image=bg_photo)

# GUI Elements
time_text_id = canvas.create_text(CENTER - 110, 50, text="", fill="#e6eefb", font=("Segoe UI", 16, "bold"))
battery_text_id = canvas.create_text(CENTER + 110, 50, text="100%", fill="#9eff7a", font=("Segoe UI", 14, "bold"))
canvas.create_text(CENTER, 85, text="HEALTH TRACKER", fill="#bfc7cf", font=("Helvetica", 14, "bold"))

digital_text_id = canvas.create_text(CENTER, CENTER - 40, text="HR: 75 bpm\nSpO2: 98%", fill="#7CFC00", font=("Consolas", 32, "bold"), justify="center")
status_msg_id = canvas.create_text(CENTER, CENTER + 50, text="Safe to drive", fill="#7CFC00", font=("Segoe UI", 15, "bold"), justify="center")

hr_var = tk.IntVar(value=75)
spo2_var = tk.IntVar(value=98)

# Simulation Sliders
rect_x1 = CENTER - 155
rect_y1 = CENTER + 95
rect_x2 = CENTER + 155
rect_y2 = CENTER + 210
rect_bg_id = canvas.create_rectangle(rect_x1, rect_y1, rect_x2, rect_y2, fill="#0d1117", outline="#00FF00", width=3)

hr_label = canvas.create_text(rect_x1 + 12, rect_y1 + 12, text="HR: 75 bpm", fill="#00FF00", font=("Segoe UI", 10, "bold"), anchor="w")
hr_slider = tk.Scale(canvas, from_=30, to=150, orient="horizontal", variable=hr_var, bg="#1a2332", fg="#00FF00", troughcolor="#0d1117", activebackground="#2a4a6a", length=250, highlightthickness=0, bd=0, relief="flat", command=update_status)
canvas.create_window(CENTER, rect_y1 + 35, window=hr_slider, width=280)
canvas.create_line(rect_x1 + 8, rect_y1 + 56, rect_x2 - 8, rect_y1 + 56, fill="#00FF00", width=1)

spo2_label = canvas.create_text(rect_x1 + 12, rect_y1 + 68, text="SpO2: 98%", fill="#00FF00", font=("Segoe UI", 10, "bold"), anchor="w")
spo2_slider = tk.Scale(canvas, from_=80, to=100, orient="horizontal", variable=spo2_var, bg="#1a2332", fg="#00FF00", troughcolor="#0d1117", activebackground="#2a4a6a", length=250, highlightthickness=0, bd=0, relief="flat", command=update_status)
canvas.create_window(CENTER, rect_y1 + 91, window=spo2_slider, width=280)

# --- Right Frame: Vision Feed ---
vision_frame = tk.Frame(root, bg="#111111", width=320, height=CANVAS_SIZE)
vision_frame.pack(side=tk.RIGHT, fill=tk.Y)
vision_frame.pack_propagate(False) # Prevent frame from resizing

tk.Label(vision_frame, text="DRIVER VISION", font=("Helvetica", 14, "bold"), bg="#111111", fg="#bfc7cf").pack(pady=10)

# Label to hold the video feed
video_label = tk.Label(vision_frame, bg="#000000")
video_label.pack(pady=10, padx=10)

# --- Start Application ---
update_time()           # Start the GUI clock
root.after(100, update_status) # Initialize slider status
root.after(500, connect_to_esp32) # Attempt initial serial connection

# Start the vision processing in a separate thread
vision_thread = threading.Thread(target=vision_loop, daemon=True)
vision_thread.start()

# Start the main GUI loop
root.mainloop()

# --- Cleanup on GUI close ---
print("GUI closed, stopping threads...")
stop_event.set() # Signal the vision thread to stop
vision_thread.join() # Wait for thread to finish

if ser:
    ser.close()
    print("Serial port closed.")

print("System Stopped")