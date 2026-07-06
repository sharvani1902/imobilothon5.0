AI-Enhanced Driver Wellness Monitoring

This project is an AI-enhanced system for monitoring driver wellness. It integrates this monitoring with a comprehensive, two-part vehicle control system designed to autonomously control the vehicle in response to emergencies.

The system is split into two main components:

main_controller.py (Host Controller): A Python application that runs on a host machine (e.g., Raspberry Pi). It uses a camera to monitor the driver for fatigue and distraction while simultaneously providing a GUI to simulate vital sign emergencies.

main_controller.ino (Vehicle Controller): An Arduino sketch for an ESP32. It controls the vehicle's motor, monitors steering wheel activity, and listens for emergency commands from the Host Controller. If a halt is triggered, it sends an emergency SMS with the vehicle's GPS location.

System Architecture
The two components communicate over a Serial (USB) connection.

1. Host Controller (main_controller.py)
This Python script runs two concurrent tasks:

Vision Processing Thread (Worker):

Uses OpenCV and MediaPipe with a Picamera2 feed.

Monitors the driver's face in real-time.

Detects signs of fatigue (microsleep, high PERCLOS, frequent yawning).

Detects signs of distraction (prolonged head turn away from the road).

Plays local audio alerts (break.mp3, focus.mp3) for the driver.

Sends a F (Fatigue) command to the ESP32 upon detecting fatigue.

Vitals GUI (Main Thread):

Renders a Tkinter dashboard.

This GUI is a simulation tool. It provides sliders for Heart Rate (HR) and SpO2 (blood oxygen) to simulate a critical vitals event.

If the simulated vitals cross emergency thresholds (e.g., SpO2 < 88), it sends an E (Emergency) command with the vitals data (e.g., E,75,85) to the ESP32.

If vitals return to a safe range, it sends a N (Normal) command.

2. Vehicle Controller (main_controller.ino)
This ESP32 sketch is the vehicle's main brain and safety system. It manages a state machine (STATE_NORMAL, STATE_EMERGENCY_RAMP, STATE_HALTED).

Serial Command Handling:

Listens for commands from the Host Controller.

F (Fatigue): Triggers an emergency ramp-down.

E (Vitals): Triggers an emergency ramp-down and stores the vitals data for the alert.

N (Normal): Cancels an ongoing ramp-down if the halt was triggered by vitals (simulating recovery).

Local Monitoring (Steering):

Independently monitors a potentiometer (simulating a steering wheel).

If no steering movement is detected for 10 seconds, it sounds a local buzzer.

If no movement is detected for 15 seconds, it self-triggers an emergency ramp-down (Cause: "Steering").

Vehicle Control & Alerting:

Motor Control: Manages a DC motor via an L298N driver. In STATE_NORMAL, it runs at MAX_DUTY.

Emergency Ramp-Down: When an emergency is triggered (by F, E, or Steering), it enters STATE_EMERGENCY_RAMP. It gradually reduces motor speed to zero over 10 seconds.

Halted State: Once the motor is at zero, it applies an active brake and enters STATE_HALTED.

GPS & SMS Alert: Upon halting, it uses a connected GPS module to get the current location. It then connects to WiFi and sends an emergency SMS via the Twilio API. The message includes the cause of the halt (Vitals, Fatigue, or Steering) and a Google Maps link to the vehicle's location.

Features
Host Controller
Real-time Fatigue Detection: Uses Eye Aspect Ratio (EAR) for microsleeps and PERCLOS.

Yawn Detection: Counts yawns (based on Mouth Aspect Ratio or MAR) within a time window to identify drowsiness.

Distraction Monitoring: Detects when the driver's head is turned away, using the nose landmark.

Vitals Simulation Dashboard: A clean Tkinter GUI to test the emergency response of the ESP32.

Thread-Safe Serial: Uses a threading.Lock to ensure vision and GUI threads don't write to the serial port simultaneously.

Vehicle Controller
Multi-Cause Emergency Trigger: The system can be halted by three distinct causes: poor vitals (from host), fatigue (from host), or driver inactivity (local).

Safe Vehicle Halt: Implements a 10-second ramp-down instead of an immediate, dangerous stop.

Steering Inactivity Failsafe: Acts as a "dead-man switch," ensuring the system reacts even if the host controller fails, as long as the driver is incapacitated.

GPS Time Synchronization: Critically, it uses the GPS module's time data to set the ESP32's internal clock, which is required to validate the SSL certificate for the Twilio API.

Remote Alerting: Automatically sends a Twilio SMS to a predefined number, providing critical location and context for a first responder.

Required Configuration
To use this system, you must update the configuration variables in both files.

In main_controller.py
SERIAL_PORT: The serial port your ESP32 is connected to (e.g., /dev/ttyUSB0 or COM3).

SOUND_BREAK: The file path to your "fatigue" alert audio file.

SOUND_FOCUS: The file path to your "distraction" alert audio file.

In main_controller.ino
WiFi Credentials:

ssid: Your WiFi network name.

password: Your WiFi password.

Twilio Credentials:

accountSID: Your Twilio Account SID.

authToken: Your Twilio Auth Token.

fromNumber: Your Twilio-provided phone number.

toNumber: The verified phone number to send the alert to.

Usage (Operational Flow)
Hardware Setup:

Connect the ESP32 to the motor driver, motor, potentiometer, GPS module, buzzer, and LED as per the pins defined in main_controller.ino.

Connect the Host (Raspberry Pi) to its camera and connect it to the ESP32 via USB.

Software Setup:

Install all Python dependencies from main_controller.py (e.g., opencv-python, mediapipe, pyserial, picamera2).

Install the required Arduino libraries for the ESP32 (e.g., TinyGPSPlus, base64).

Install the mpg123 audio player on the host (sudo apt install mpg123).

Configuration:

Edit both .py and .ino files to update all credentials and ports as described in the section above.

Execution:

Upload main_controller.ino to the ESP32. It will start, connect to WiFi, and (if wired) begin running the motor.

Run main_controller.py on the host machine.

Operation:

The host GUI and vision feed will appear. The system is now live.

You can test the system by:

Vitals: Moving the sliders in the GUI to an emergency level.

Fatigue: Covering your eyes or yawning repeatedly at the camera.

Steering: Stopping any movement of the potentiometer.

In any of these cases, the ESP32 will trigger its STATE_EMERGENCY_RAMP, slow the motor to a stop, and send the Twilio SMS alert.
