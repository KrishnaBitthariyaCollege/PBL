#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MPU6050 sensor
Adafruit_MPU6050 mpu;

// HW-827 Pulse Sensor settings
#define PULSE_SENSOR_PIN 4

// Variables for heart rate calculation
const int RATE_SIZE = 10;
int rates[RATE_SIZE];
int rateSpot = 0;
unsigned long lastBeatTime = 0;
int BPM = 0;
int beatsPerMinute = 0;
int beatAvg = 0;

// Adaptive threshold variables
long signalValue = 0;
long signalMax = 2100;
long signalMin = 1800;
int threshold = 2000;
bool pulseDetected = false;
unsigned long lastPulseTime = 0;

// Variables for motion detection
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float totalAccel = 0;
float totalGyro = 0;

// Seizure detection variables
#define SEIZURE_WINDOW_SIZE 20      // Number of samples to analyze
#define SEIZURE_THRESHOLD 3.0       // Minimum acceleration for seizure detection (m/s²)
#define GYRO_THRESHOLD 2.0          // Minimum gyro for seizure detection (rad/s)
#define SEIZURE_COUNT_THRESHOLD 12  // Number of high-motion samples needed

// BPM thresholds for seizure validation
#define BPM_HIGH_THRESHOLD 120      // High BPM indicating possible seizure
#define BPM_LOW_THRESHOLD 50        // Low BPM indicating possible seizure
#define BPM_VALIDATION_TIME 2000    // Wait 2 seconds to validate BPM reading

float accelHistory[SEIZURE_WINDOW_SIZE];
float gyroHistory[SEIZURE_WINDOW_SIZE];
int historyIndex = 0;
bool historyFilled = false;

bool seizureDetected = false;
bool potentialSeizureMotion = false;
unsigned long seizureStartTime = 0;
unsigned long potentialSeizureStartTime = 0;
unsigned long lastSeizureAlert = 0;
int highMotionCount = 0;
bool bpmAbnormal = false;
bool fingerDetected = false;

// Buzzer pin (optional - uncomment if you add a buzzer)
// #define BUZZER_PIN 5

void scanI2C() {
  byte error, address;
  int nDevices = 0;
  
  Serial.println("Scanning I2C bus...");
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  
  if (nDevices == 0)
    Serial.println("No I2C devices found!");
  else
    Serial.println("Scan complete");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting Seizure Detection System...");
  
  // Initialize I2C
  Wire.begin(21, 22, 100000);
  delay(100);
  
  scanI2C();
  
  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
      Serial.println(F("OLED not found on 0x3C or 0x3D"));
    }
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();
  delay(500);
  
  // Initialize MPU6050
  Serial.println("Attempting MPU6050 at 0x68...");
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("Failed at 0x68, trying 0x69...");
    if (!mpu.begin(0x69, &Wire)) {
      Serial.println("Failed to find MPU6050!");
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("MPU6050 Error!");
      display.println("Check wiring:");
      display.println("VCC->3.3V");
      display.println("GND->GND");
      display.println("SDA->GPIO21");
      display.println("SCL->GPIO22");
      display.display();
      while (1) {
        delay(1000);
      }
    } else {
      Serial.println("MPU6050 found at 0x69!");
    }
  } else {
    Serial.println("MPU6050 found at 0x68!");
  }
  
  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Initialize pulse sensor
  pinMode(PULSE_SENSOR_PIN, INPUT);
  for (int i = 0; i < RATE_SIZE; i++) {
    rates[i] = 0;
  }
  
  // Initialize seizure detection arrays
  for (int i = 0; i < SEIZURE_WINDOW_SIZE; i++) {
    accelHistory[i] = 0;
    gyroHistory[i] = 0;
  }
  
  // Uncomment if using buzzer
  // pinMode(BUZZER_PIN, OUTPUT);
  // digitalWrite(BUZZER_PIN, LOW);
  
  Serial.println("Setup complete!");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("SEIZURE MONITOR");
  display.println("Ready!");
  display.println("Monitoring...");
  display.display();
  delay(2000);
}

void readPulseSensor() {
  signalValue = analogRead(PULSE_SENSOR_PIN);
  
  // Adaptive threshold
  if (signalValue > signalMax) {
    signalMax = signalValue;
  }
  if (signalValue < signalMin) {
    signalMin = signalValue;
  }
  
  signalMax = signalMax * 0.99 + signalValue * 0.01;
  signalMin = signalMin * 0.99 + signalValue * 0.01;
  threshold = (signalMax + signalMin) / 2;
  
  // Check if finger is detected based on signal range
  long signalRange = signalMax - signalMin;
  fingerDetected = (signalRange > 100);
  
  // Detect rising edge of pulse
  if (signalValue > threshold && !pulseDetected) {
    pulseDetected = true;
    unsigned long currentTime = millis();
    unsigned long IBI = currentTime - lastBeatTime;
    lastBeatTime = currentTime;
    
    // Only count beats with realistic intervals (300ms to 2000ms = 30-200 BPM)
    if (IBI > 300 && IBI < 2000) {
      beatsPerMinute = 60000 / IBI;
      
      rates[rateSpot++] = beatsPerMinute;
      if (rateSpot >= RATE_SIZE) {
        rateSpot = 0;
      }
      
      long total = 0;
      for (int i = 0; i < RATE_SIZE; i++) {
        total += rates[i];
      }
      beatAvg = total / RATE_SIZE;
      BPM = beatAvg;
    }
  }
  
  if (signalValue < threshold) {
    pulseDetected = false;
  }
  
  // Reset BPM if no beat detected for 3 seconds
  if (millis() - lastBeatTime > 3000) {
    BPM = 0;
  }
}

void readMPU6050() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;
  
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
  
  // Calculate total acceleration magnitude (remove gravity)
  float totalAccelRaw = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
  totalAccel = abs(totalAccelRaw - 9.8);
  
  // Calculate total gyroscope magnitude
  totalGyro = sqrt(gyroX*gyroX + gyroY*gyroY + gyroZ*gyroZ);
}

void detectSeizure() {
  // Store current motion data in circular buffer
  accelHistory[historyIndex] = totalAccel;
  gyroHistory[historyIndex] = totalGyro;
  
  historyIndex++;
  if (historyIndex >= SEIZURE_WINDOW_SIZE) {
    historyIndex = 0;
    historyFilled = true;
  }
  
  // Only analyze if we have enough data
  if (!historyFilled) {
    return;
  }
  
  // Count samples that exceed thresholds
  highMotionCount = 0;
  
  for (int i = 0; i < SEIZURE_WINDOW_SIZE; i++) {
    if (accelHistory[i] > SEIZURE_THRESHOLD || gyroHistory[i] > GYRO_THRESHOLD) {
      highMotionCount++;
    }
  }
  
  // Check if BPM is abnormal (high or low) AND finger is detected
  bpmAbnormal = fingerDetected && BPM > 30 && 
                ((BPM > BPM_HIGH_THRESHOLD) || (BPM < BPM_LOW_THRESHOLD));
  
  // Detect potential seizure motion (high motion count > 12)
  bool highMotionDetected = (highMotionCount > SEIZURE_COUNT_THRESHOLD);
  
  // SEIZURE LOGIC: High motion + Abnormal BPM = Seizure
  if (highMotionDetected && bpmAbnormal) {
    if (!seizureDetected) {
      // New seizure detected - both conditions met
      seizureStartTime = millis();
      seizureDetected = true;
      lastSeizureAlert = millis();
      
      // Uncomment to activate buzzer
      // digitalWrite(BUZZER_PIN, HIGH);
      
      Serial.println("⚠️⚠️⚠️ SEIZURE DETECTED! ⚠️⚠️⚠️");
      Serial.print("High motion count: ");
      Serial.print(highMotionCount);
      Serial.print(" | BPM: ");
      Serial.print(BPM);
      Serial.println(BPM > BPM_HIGH_THRESHOLD ? " (HIGH)" : " (LOW)");
    }
  } 
  else if (highMotionDetected && !bpmAbnormal) {
    // High motion but normal/no BPM - potential seizure, waiting for validation
    if (!potentialSeizureMotion) {
      potentialSeizureMotion = true;
      potentialSeizureStartTime = millis();
      Serial.println("⚠️ High motion detected - Waiting for BPM confirmation...");
      Serial.print("High motion: ");
      Serial.print(highMotionCount);
      Serial.print(" | BPM: ");
      Serial.print(BPM);
      Serial.print(" | Finger: ");
      Serial.println(fingerDetected ? "YES" : "NO");
    }
    
    // Clear seizure if it was previously detected but BPM normalized
    if (seizureDetected) {
      seizureDetected = false;
      seizureStartTime = 0;
      // digitalWrite(BUZZER_PIN, LOW);
      Serial.println("Seizure cleared - BPM normalized");
    }
  }
  else {
    // No high motion or seizure ended
    if (seizureDetected) {
      seizureDetected = false;
      seizureStartTime = 0;
      // digitalWrite(BUZZER_PIN, LOW);
      Serial.println("Seizure ended - Motion normalized");
    }
    if (potentialSeizureMotion) {
      potentialSeizureMotion = false;
      Serial.println("Potential seizure cleared - Motion normalized");
    }
  }
}

void updateDisplay() {
  display.clearDisplay();
  
  // Check if seizure is detected (high motion + abnormal BPM)
  if (seizureDetected) {
    // SEIZURE ALERT SCREEN
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("SEIZURE");
    display.println("ALERT!");
    
    display.setTextSize(1);
    display.setCursor(0, 35);
    display.print("BPM: ");
    display.print(BPM);
    display.println(BPM > BPM_HIGH_THRESHOLD ? " HIGH" : " LOW");
    
    display.setCursor(0, 45);
    display.print("Motion: ");
    display.print(highMotionCount);
    display.print("/20 HIGH");
    
    display.setCursor(0, 55);
    display.print("Accel: ");
    display.print(totalAccel, 1);
    display.print(" m/s2");
    
    // Flash display for alert
    if ((millis() / 500) % 2 == 0) {
      display.invertDisplay(true);
    } else {
      display.invertDisplay(false);
    }
  } 
  else if (potentialSeizureMotion) {
    // POTENTIAL SEIZURE - WAITING FOR BPM CONFIRMATION
    display.invertDisplay(false);
    
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("HIGH MOTION");
    display.println("Checking BPM...");
    display.drawLine(0, 18, 128, 18, SSD1306_WHITE);
    
    display.setCursor(0, 22);
    display.print("Motion: ");
    display.print(highMotionCount);
    display.print("/20");
    
    display.setCursor(0, 32);
    display.print("BPM: ");
    if (fingerDetected && BPM > 30) {
      display.print(BPM);
      display.print(" (Normal)");
    } else if (!fingerDetected) {
      display.print("No finger");
    } else {
      display.print("Reading...");
    }
    
    display.setCursor(0, 45);
    display.println("Place finger on");
    display.setCursor(0, 55);
    display.println("sensor for check");
  }
  else {
    // NORMAL MONITORING SCREEN
    display.invertDisplay(false);
    
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("SEIZURE MONITOR");
    display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
    
    // Heart Rate
    display.setCursor(0, 15);
    display.print("Heart Rate:");
    display.setTextSize(2);
    display.setCursor(0, 25);
    
    if (BPM > 30 && BPM < 200 && fingerDetected) {
      display.print(BPM);
      display.print(" BPM");
    } else if (!fingerDetected) {
      display.setTextSize(1);
      display.setCursor(0, 25);
      display.println("No finger");
      display.println("detected");
    } else {
      display.print("-- BPM");
    }
    
    // Motion Data
    display.setTextSize(1);
    display.setCursor(0, 45);
    display.print("Motion: ");
    display.print(totalAccel, 2);
    display.print(" m/s2");
    
    display.setCursor(0, 55);
    display.print("High: ");
    display.print(highMotionCount);
    display.print("/20 G:");
    display.print(totalGyro, 1);
  }
  
  display.display();
}

void loop() {
  // Read sensors
  readPulseSensor();
  readMPU6050();
  
  // Detect seizure patterns
  detectSeizure();
  
  // Update display every 250ms
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 250) {
    updateDisplay();
    lastDisplayUpdate = millis();
    
    // Serial output for debugging
    if (!seizureDetected) {
      Serial.print("BPM: ");
      Serial.print(BPM);
      Serial.print(" | Finger: ");
      Serial.print(fingerDetected ? "YES" : "NO");
      Serial.print(" | Accel: ");
      Serial.print(totalAccel, 2);
      Serial.print(" | Gyro: ");
      Serial.print(totalGyro, 2);
      Serial.print(" | High Motion: ");
      Serial.print(highMotionCount);
      Serial.print("/");
      Serial.print(SEIZURE_WINDOW_SIZE);
      Serial.print(" | Status: ");
      if (potentialSeizureMotion) {
        Serial.println("MONITORING");
      } else {
        Serial.println("NORMAL");
      }
    }
  }
  
  delay(20); // 50Hz sampling rate
}