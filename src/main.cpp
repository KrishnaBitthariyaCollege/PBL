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
#define PULSE_SENSOR_PIN 4   // Changed to GPIO 4 as per your connection

// Variables for heart rate calculation with improved algorithm
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

// I2C Scanner function
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
  Serial.println("Starting...");
  
  // Initialize I2C with custom pins and frequency
  Wire.begin(21, 22, 100000); // SDA=21, SCL=22, 100kHz
  delay(100);
  
  // Scan I2C bus first
  scanI2C();
  
  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    // Try alternate address
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
  
  // Initialize MPU6050 with address attempt
  Serial.println("Attempting MPU6050 at 0x68...");
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("Failed at 0x68, trying 0x69...");
    if (!mpu.begin(0x69, &Wire)) {
      Serial.println("Failed to find MPU6050 at both addresses!");
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
  
  // Initialize pulse sensor variables
  pinMode(PULSE_SENSOR_PIN, INPUT);
  for (int i = 0; i < RATE_SIZE; i++) {
    rates[i] = 0;
  }
  
  Serial.println("Setup complete!");
  Serial.println("Place finger on pulse sensor...");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Ready!");
  display.println("Place finger");
  display.println("on sensor...");
  display.display();
  delay(2000);
}

void readPulseSensor() {
  signalValue = analogRead(PULSE_SENSOR_PIN);
  
  // Adaptive threshold - automatically adjusts to signal range
  if (signalValue > signalMax) {
    signalMax = signalValue;
  }
  if (signalValue < signalMin) {
    signalMin = signalValue;
  }
  
  // Gradually decay max and min to adapt to changing conditions
  signalMax = signalMax * 0.99 + signalValue * 0.01;
  signalMin = signalMin * 0.99 + signalValue * 0.01;
  
  // Set threshold as midpoint
  threshold = (signalMax + signalMin) / 2;
  
  // Debug: Print raw sensor value occasionally
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 200) {
    Serial.print("Signal: ");
    Serial.print(signalValue);
    Serial.print(" | Threshold: ");
    Serial.print(threshold);
    Serial.print(" | Min: ");
    Serial.print(signalMin);
    Serial.print(" | Max: ");
    Serial.println(signalMax);
    lastDebug = millis();
  }
  
  // Detect rising edge of pulse
  if (signalValue > threshold && !pulseDetected) {
    pulseDetected = true;
    unsigned long currentTime = millis();
    unsigned long IBI = currentTime - lastBeatTime; // Inter-Beat Interval
    lastBeatTime = currentTime;
    
    // Only count beats with realistic intervals (between 300ms and 2000ms)
    // That's 30-200 BPM range
    if (IBI > 300 && IBI < 2000) {
      beatsPerMinute = 60000 / IBI;
      
      // Store in circular buffer for averaging
      rates[rateSpot++] = beatsPerMinute;
      if (rateSpot >= RATE_SIZE) {
        rateSpot = 0;
      }
      
      // Calculate average BPM from last 10 beats
      long total = 0;
      for (int i = 0; i < RATE_SIZE; i++) {
        total += rates[i];
      }
      beatAvg = total / RATE_SIZE;
      BPM = beatAvg;
      
      Serial.print("♥ Beat! BPM: ");
      Serial.print(beatsPerMinute);
      Serial.print(" | Avg: ");
      Serial.println(beatAvg);
    }
  }
  
  // Detect falling edge
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
  
  // Calculate total acceleration magnitude
  float totalAccelRaw = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
  
  // Remove gravity (9.8 m/s²) to show only movement
  totalAccel = abs(totalAccelRaw - 9.8);
}

void updateDisplay() {
  display.clearDisplay();
  
  // Display Heart Rate
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("SEIZURE MONITOR");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  // Heart Rate
  display.setTextSize(1);
  display.setCursor(0, 15);
  display.print("Heart Rate:");
  display.setTextSize(2);
  display.setCursor(0, 25);
  
  // Check if signal quality is good (range between min and max should be reasonable)
  long signalRange = signalMax - signalMin;
  
  if (BPM > 30 && BPM < 200 && signalRange > 100) {
    display.print(BPM);
    display.print(" BPM");
  } else if (signalRange < 100) {
    display.setTextSize(1);
    display.setCursor(0, 25);
    display.println("No finger");
    display.println("detected!");
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
  display.print("Rotation: ");
  display.print(abs(gyroX) + abs(gyroY) + abs(gyroZ), 1);
  
  display.display();
}

void loop() {
  // Read sensors
  readPulseSensor();
  readMPU6050();
  
  // Update display every 500ms
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 500) {
    updateDisplay();
    lastDisplayUpdate = millis();
    
    // Serial output for debugging
    Serial.print("BPM: ");
    Serial.print(BPM);
    Serial.print(" | Accel: ");
    Serial.print(totalAccel);
    Serial.print(" | Gyro: ");
    Serial.println(abs(gyroX) + abs(gyroY) + abs(gyroZ));
  }
  
  delay(20); // Small delay for stability
}