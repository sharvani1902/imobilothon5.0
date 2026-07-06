// =================================================================
// === INTEGRATED DRIVER WELLNESS - VEHICLE CONTROLLER (ESP32) ===
// =================================================================
//
// This sketch runs on the ESP32. It:
// 1. Monitors steering wheel (potentiometer) for inactivity.
// 2. Listens for Serial commands ('E' Vitals, 'F' Fatigue, 'N' Normal) from the host.
// 3. Manages a state machine (NORMAL, EMERGENCY_RAMP, HALTED).
// 4. Controls the motor (L298N) to simulate vehicle halt.
// 5. Reads GPS data for location.
// 6. Sends a Twilio SMS alert with cause and location upon halting.
//
// =================================================================

// --- WiFi & HTTP Libraries ---
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <base64.h>
#include <time.h>
#include <sys/time.h>

// --- GPS Libraries ---
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// --- WiFi Credentials ---
const char* ssid = "Kibo_AirFiber";
const char* password = "kibo@123";

// --- Twilio API Credentials ---
const char* accountSID = "ACXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"; // TODO: Populate with Twilio credentials
const char* authToken  = "YYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY"; // TODO: Populate with Twilio credentials
const char* fromNumber = "XXXXXXXXXX"; // Twilio Number
const char* toNumber   = "YYYYYYYYYY"; // Verified 'To' Number
const char* host = "api.twilio.com";
const int httpsPort = 443;

// --- Hardware Pins ---
const int ENA = 25;       // L298N ENA (PWM)
const int IN1 = 26;       // L298N IN1
const int IN2 = 27;       // L298N IN2
#define LED_PIN 4         // External LED
const int potPin = 34;    // Steering Potentiometer
const int buzzerPin = 2;  // Steering Warning Buzzer

// --- GPS Settings ---
static const int RXPin = 16; // ESP32 RX2
static const int TXPin = 17; // ESP32 TX2
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial SerialGPS(2); // Use UART2

// --- Motor Settings ---
const uint8_t MAX_DUTY = 160;               // Max motor speed (0-255)
const unsigned long RAMP_DOWN_TIME = 10000; // 10 seconds to ramp to 0
const unsigned long RAMP_DOWN_INTERVAL = RAMP_DOWN_TIME / MAX_DUTY;

// --- Steering Monitor Settings ---
const float EMA_ALPHA = 0.2f;   // Exponential Moving Average filter
const int   DEAD_BAND = 20;     // Potentiometer deadband to ignore jitter
const unsigned long BUZZER_ALERT_MS = 10000;    // 10s of inactivity for buzzer
const unsigned long EMERGENCY_TRIGGER_MS = 15000; // 15s of inactivity for halt

// --- State Machine ---
enum AppState {
  STATE_NORMAL,
  STATE_EMERGENCY_RAMP,
  STATE_HALTED
};
AppState currentState = STATE_NORMAL;

// --- Global Timers & States ---
int currentSpeed = 0;
unsigned long rampTimer = 0;
bool blinkingActive = false;
bool ledState = LOW;
unsigned long previousBlinkMillis = 0;
const long blinkInterval = 1000; // 1Hz blink
bool smsSent = false;

// --- Steering Globals ---
int   last_for_movement = 0; // Last "moved" potentiometer value
float ema = 0.0f;            // Smoothed potentiometer value
unsigned long lastMoveMs = 0;  // Timestamp of last movement
bool isBuzzerOn = false;

// --- Global Emergency Data ---
String emergencyCause = "Unknown";
String emergencyHR = "N/A";
String emergencySpO2 = "N/A";

// ----------------------------------------------------------------
// HELPER: WiFi Connection Function
// ----------------------------------------------------------------
void setupWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (attempts++ > 40) { // 20-second timeout
      Serial.println("\nFailed to connect to WiFi. Restarting...");
      ESP.restart();
    }
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// ----------------------------------------------------------------
// HELPER: GPS Feed
// ----------------------------------------------------------------
/**
 * @brief Reads available data from GPS module into the TinyGPS++ object.
 */
void feedGPS() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
}

// ----------------------------------------------------------------
// HELPER: Sync Clock to GPS
// ----------------------------------------------------------------
/**
 * @brief Syncs the ESP32's internal clock with UTC time from the GPS.
 * @note This is CRITICAL for Twilio's SSL/TLS certificate validation.
 */
void syncClockToGPS() {
  unsigned long start = millis();
  bool gpsTimeSet = false;
  Serial.print("Waiting for GPS fix to sync time");

  while (!gpsTimeSet && millis() - start < 30000) { // 30s timeout
    feedGPS(); // Keep feeding GPS data
    if (gps.date.isValid() && gps.time.isValid()) {
      struct tm tm;
      tm.tm_year = gps.date.year() - 1900;
      tm.tm_mon = gps.date.month() - 1;
      tm.tm_mday = gps.date.day();
      tm.tm_hour = gps.time.hour();
      tm.tm_min = gps.time.minute();
      tm.tm_sec = gps.time.second();

      time_t epochTime = mktime(&tm);
      struct timeval tv;
      tv.tv_sec = epochTime;
      tv.tv_usec = 0;
      settimeofday(&tv, NULL); // Set the ESP32 system time

      Serial.println("\nSuccessfully synced system clock to GPS (UTC).");
      gpsTimeSet = true;
    } else {
      Serial.print(".");
      delay(500);
    }
  }
  if (!gpsTimeSet) {
    Serial.println("\nWarning: Failed to get GPS time sync. SSL connection may fail.");
  }
}

// ----------------------------------------------------------------
// SETUP: Initialize all hardware
// ----------------------------------------------------------------
void setup() {
  Serial.begin(115200); // Serial to Host (Python)
  delay(1000);

  SerialGPS.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  Serial.println("GPS module initializing...");

  // --- Motor, LED & Buzzer Setup ---
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  // --- Steering Pot Setup ---
  analogReadResolution(12);
  analogSetPinAttenuation(potPin, ADC_11db);
  int init = analogRead(potPin); // Initial pot reading
  ema = init;
  last_for_movement = init;
  lastMoveMs = millis();

  // Start motor at full speed
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW); // Set forward direction
  analogWrite(ENA, MAX_DUTY);
  currentSpeed = MAX_DUTY;
  currentState = STATE_NORMAL;
  Serial.println("Motor starting. STATE: NORMAL");

  setupWiFi();
  syncClockToGPS(); // Critical for Twilio HTTPS

  Serial.println("\nSystem Ready. Monitoring Serial, Steering, and GPS.");
}

// ----------------------------------------------------------------
// LOOP: Main non-blocking loop
// ----------------------------------------------------------------
void loop() {
  unsigned long currentMillis = millis();

  feedGPS(); // Constantly read GPS data
  handleBlinking(currentMillis); // Handle LED state

  // Only run logic if system is not permanently halted
  if (currentState != STATE_HALTED) {
    checkSerial(); // Check for commands from Host
    checkSteering(currentMillis); // Check for steering movement
    handleMotor(currentMillis);   // Run motor state machine
  }
}

// ----------------------------------------------------------------
// HELPER FUNCTIONS
// ----------------------------------------------------------------

/**
 * @brief Reads 5 samples from an analog pin and returns the median.
 * @param pin The analog pin to read.
 * @return The median of 5 samples.
 */
int readMedian5(int pin) {
  int s[5];
  for (int i = 0; i < 5; i++) { s[i] = analogRead(pin); delayMicroseconds(300); }
  for (int i = 0; i < 5; i++)
    for (int j = i + 1; j < 5; j++)
      if (s[j] < s[i]) { int t = s[i]; s[i] = s[j]; s[j] = t; }
  return s[2]; // median
}

/**
 * @brief Manages the non-blocking blinking of the emergency LED.
 */
void handleBlinking(unsigned long currentMillis) {
  if (blinkingActive) {
    if (currentMillis - previousBlinkMillis >= blinkInterval) {
      previousBlinkMillis = currentMillis;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  }
}

/**
 * @brief Checks for incoming serial commands from the host (Python).
 * Handles 'E' (Vitals), 'F' (Fatigue), and 'N' (Normal).
 */
void checkSerial() {
  if (Serial.available()) {
    String serialData = Serial.readStringUntil('\n');
    serialData.trim();

    if (serialData.startsWith("E")) { // VITALS Emergency
      if (currentState == STATE_NORMAL) {
        Serial.print("EMERGENCY (Vitals) received! Data: ");
        Serial.println(serialData);

        // Parse the HR and SpO2 values
        int firstComma = serialData.indexOf(',');
        int secondComma = serialData.indexOf(',', firstComma + 1);

        if (firstComma > 0 && secondComma > 0) {
          emergencyHR = serialData.substring(firstComma + 1, secondComma);
          emergencySpO2 = serialData.substring(secondComma + 1);
        }
        emergencyCause = "Vitals";
        currentState = STATE_EMERGENCY_RAMP;
        blinkingActive = true;
        rampTimer = millis();
      }
    }
    else if (serialData.startsWith("F")) { // FATIGUE Emergency
      if (currentState == STATE_NORMAL) {
        Serial.println("EMERGENCY (Fatigue) received!");
        emergencyHR = "N/A";
        emergencySpO2 = "N/A";
        emergencyCause = "Fatigue";
        currentState = STATE_EMERGENCY_RAMP;
        blinkingActive = true;
        rampTimer = millis();
      }
    }
    else if (serialData.startsWith("N")) { // NORMAL Signal
      if (currentState == STATE_EMERGENCY_RAMP) {
        // Received 'N' (Normal) from host, cancel the ramp-down
        Serial.println("NORMAL received. Cancelling emergency ramp-down.");
        currentState = STATE_NORMAL;
        blinkingActive = false;
        digitalWrite(LED_PIN, LOW);

        // Speed motor back up
        currentSpeed = MAX_DUTY;
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, currentSpeed);
      }
    }
  }
}

/**
 * @brief Monitors the steering potentiometer for driver inactivity.
 * Triggers warnings (buzzer) and emergency state.
 */
void checkSteering(unsigned long currentMillis) {
  int   raw = readMedian5(potPin);
  ema = EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * ema;
  int   smoothed = (int)(ema + 0.5f);

  // 1) Movement detection
  if (abs(smoothed - last_for_movement) > DEAD_BAND) {
    last_for_movement = smoothed;
    lastMoveMs = currentMillis;

    // Silence buzzer if it was on
    if (isBuzzerOn) {
      digitalWrite(buzzerPin, LOW);
      isBuzzerOn = false;
      Serial.println("Movement detected. Buzzer OFF.");
    }

    // Movement detected, cancel a *steering-induced* emergency.
    if (currentState == STATE_EMERGENCY_RAMP && emergencyCause == "Steering") {
      Serial.println("Movement detected! Cancelling steering emergency.");
      currentState = STATE_NORMAL;
      blinkingActive = false;
      digitalWrite(LED_PIN, LOW);

      currentSpeed = MAX_DUTY;
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, currentSpeed);
    }
    return; // Exit function since movement was detected
  }

  // 2) --- NO MOVEMENT DETECTED ---
  // This code only runs if the 'if' block above is false.
  if (currentState == STATE_NORMAL) {
    unsigned long noMoveDuration = currentMillis - lastMoveMs;

    if (noMoveDuration > EMERGENCY_TRIGGER_MS) {
      // --- TRIGGER FULL EMERGENCY (STEERING) ---
      Serial.println("EMERGENCY (Steering): No movement for 15s.");
      emergencyCause = "Steering";
      emergencyHR = "N/A";
      emergencySpO2 = "N/A";

      currentState = STATE_EMERGENCY_RAMP;
      blinkingActive = true;
      rampTimer = currentMillis; // Start the motor ramp-down

      if (!isBuzzerOn) {
        digitalWrite(buzzerPin, HIGH);
        isBuzzerOn = true;
      }

    } else if (noMoveDuration > BUZZER_ALERT_MS) {
      // --- TRIGGER BUZZER WARNING ONLY ---
      if (!isBuzzerOn) {
        Serial.println("Warning: No steering for 10s. Sounding buzzer.");
        digitalWrite(buzzerPin, HIGH);
        isBuzzerOn = true;
      }
    }
  }
}


/**
 * @brief Manages the motor speed during the EMERGENCY_RAMP state.
 * Ramps speed to 0 and then applies an active brake.
 */
void handleMotor(unsigned long currentMillis) {
  if (currentState == STATE_EMERGENCY_RAMP) {
    if (currentSpeed > 0) {
      // Still ramping down
      if (currentMillis - rampTimer >= RAMP_DOWN_INTERVAL) {
        rampTimer = currentMillis;
        currentSpeed--;
        analogWrite(ENA, currentSpeed);
      }
    }
    else {
      // Motor speed has reached 0.
      // APPLY ACTIVE BRAKE (IN1=LOW, IN2=LOW)
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);

      Serial.println("Motor ramp-down complete. Active brake applied. System HALTED.");
      printFinalLocation();

      triggerEmergencySMS();

      // Enter HALTED state to prevent