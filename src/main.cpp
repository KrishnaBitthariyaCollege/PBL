#include <Arduino.h>

const int PULSE_PIN = 4; // 'S' pin connected to GPIO4

// You MUST tune this value. See instructions below.
int THRESHOLD = 2500; 

// Variables to track the heartbeat
unsigned long lastBeatTime = 0;
bool beatInProgress = false;
int BPM = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Custom Pulse Detector Started.");
  Serial.println("Open Serial Plotter to find your THRESHOLD value.");
}

void loop() {
  int sensorValue = analogRead(PULSE_PIN);

  // --- This is for finding your THRESHOLD ---
  // 1. Open the Serial Plotter.
  // 2. Put your finger on the sensor.
  // 3. Look at the value of your pulse "peaks".
  // 4. Set the THRESHOLD variable above to be in the middle of your pulse.
  Serial.print(sensorValue);
  Serial.print(",");
  Serial.println(THRESHOLD); // This draws a flat line to help you
  // ------------------------------------------

  // Check if the signal just went ABOVE the threshold
  if (sensorValue > THRESHOLD && !beatInProgress) {
    // A new beat has started!
    beatInProgress = true;
    
    unsigned long timeNow = millis();
    unsigned long timeSinceLastBeat = timeNow - lastBeatTime;
    lastBeatTime = timeNow;
    
    // Convert time (milliseconds) to BPM (beats per minute)
    BPM = 60000 / timeSinceLastBeat;

    // Only print if the BPM is in a reasonable range (e.g., 30-200)
    if (BPM > 30 && BPM < 200) {
      Serial.print(" - ♥ Beat! BPM: ");
      Serial.println(BPM);
    }
  
  } else if (sensorValue < THRESHOLD) {
    // The signal has gone back down, ready for the next beat
    beatInProgress = false;
  }
  
  delay(100); // Check 100 times per second
}