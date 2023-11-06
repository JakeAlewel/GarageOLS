#include <EEPROM.h>
#include "types.h"
#include "lidar.h"
#include <TFMPlus.h>
#include "printf.h"   // Modified to support Intel based Arduino
#include <SoftwareSerial.h>

const SoftwareSerial sensorSerial1 (2, 3, false);
const SoftwareSerial sensorSerial2 (4, 5, false);
// const SoftwareSerial sensorSerial3 (14, 13, false);
// const SoftwareSerial sensorSerial4 (16, 15, false);

const int CLOCK_RATE_HZ = 100;
const int TICK = 1000 / CLOCK_RATE_HZ;

ParkingSpotPins createPins(int cutLights, int notCutLights, int good, int warn, int bad) {
  ParkingSpotPins p;
  p.cutLights = cutLights;
  p.notCutLights = notCutLights;
  p.good = good;
  p.warn = warn;
  p.bad = bad;
  return p;
}

ParkingSpot spot1 (createPins(7, 6, 9, 10, 11), sensorSerial1, sensorSerial2);
ParkingSpot spot2 (createPins(7, 6, 9, 10, 11), sensorSerial1, sensorSerial2);

// TODO: Update to two spots when I have the sensors
const int NUMBER_OF_SPOTS = 1;
ParkingSpot spots[NUMBER_OF_SPOTS] = {spot1};

const int CONFIG_BUTTON_PIN = 12;
bool configurationButtonPressed = false;

bool enableDebugLED = false;

void setup() {
  // Setup Serial
  Serial.begin(115200);
  delay(200);

  printf_begin(); 

  // Load Configuration From Disk
  int address = 0;
  EEPROM.get(address, spots[0].config);
  address += sizeof(spots[0].config);
  EEPROM.get(address, spots[1].config);

  // Initialize Pins
  initializeSensorPins(spots);
  pinMode(CONFIG_BUTTON_PIN, INPUT);

  for (int i = 0; i < NUMBER_OF_SPOTS; i++) {
    pinMode(spots[i].pins.cutLights, OUTPUT);
    pinMode(spots[i].pins.notCutLights, OUTPUT);
    pinMode(spots[i].pins.good, OUTPUT);
    pinMode(spots[i].pins.warn, OUTPUT);
    pinMode(spots[i].pins.bad, OUTPUT);
  }
  
  // DEBUG LED PIN
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  for (int i = 0; i < NUMBER_OF_SPOTS; i++) {
    // Read Sensor Data and Update State
    updateDoorIntersectDistance(spots[i]);
    updateBackInDistance(spots[i]);
    enableAllLEDsIfNeeded(spots[i]);
    enableCutLightsIfNeeded(spots[i]);
    
    // Render LEDs
    if(!spots[i].enableAllLEDs) {
      digitalWrite(spots[i].pins.cutLights, LOW);
      digitalWrite(spots[i].pins.notCutLights, LOW);
      digitalWrite(spots[i].pins.good, LOW);
      digitalWrite(spots[i].pins.warn, LOW);
      digitalWrite(spots[i].pins.bad, LOW);
      // TODO: Write LOW to other LEDs
    } else {
      digitalWrite(spots[i].pins.cutLights, spots[i].enableCutLights ? HIGH : LOW);
      digitalWrite(spots[i].pins.notCutLights, spots[i].enableCutLights ? LOW : HIGH);


      int configDistance = spots[i].config.backInDistance;
      int step = (configDistance / 6);
      int goodBreakpoint = configDistance - step;
      int warnBreakpoint = configDistance - (step * 2);
      int currentDistance = spots[i].backInDistance;

      digitalWrite(spots[i].pins.good, configDistance > currentDistance && currentDistance >= goodBreakpoint ? HIGH : LOW);
      digitalWrite(spots[i].pins.warn, goodBreakpoint > currentDistance && currentDistance > warnBreakpoint ? HIGH : LOW);
      digitalWrite(spots[i].pins.bad, warnBreakpoint >= currentDistance ? HIGH : LOW);
    }
  }

  // Update Config on Button Press
  updateConfigStateIfNeeded();

  // Blink Debug LED
  enableDebugLED = !enableDebugLED;
  digitalWrite(LED_BUILTIN, enableDebugLED ? HIGH : LOW);

  // Delay
  delay(TICK);
}

void updateConfigStateIfNeeded() {
  bool previousConfigButtonState = configurationButtonPressed;
  configurationButtonPressed = digitalRead(CONFIG_BUTTON_PIN) == HIGH;

  if(configurationButtonPressed && previousConfigButtonState != configurationButtonPressed) {
    for (int i = 0; i < NUMBER_OF_SPOTS; i++) {
      Serial.println("Writing Config Spot " + String(i + 1) + ":");
      Serial.println("Door Intersect - " + String(spots[i].doorIntersectDistance));
      Serial.println("Back In        - " + String(spots[i].backInDistance));

      spots[i].config.backInDistance = spots[i].backInDistance;
      spots[i].config.intersectDistance = spots[i].doorIntersectDistance;

      EEPROM.put(i * sizeof(spots[i]), spots[i].config);
    }
  }
}

// TODO: Extract refactor the timestamp updates
const int UPDATE_TIMESTAMP_THRESHOLD = 6;
void updateDoorIntersectDistance(ParkingSpot& spot) {
  int previousDistance = spot.doorIntersectDistance;
  spot.doorIntersectDistance = getSensorValue(spot, false);
  if(abs(previousDistance - spot.doorIntersectDistance) > UPDATE_TIMESTAMP_THRESHOLD) {
    spot.lastChangeTimestamp = millis();
  }
}

void updateBackInDistance(ParkingSpot& spot) {
  int previousDistance = spot.backInDistance;
  spot.backInDistance = getSensorValue(spot, true);
  if(abs(previousDistance - spot.backInDistance) > UPDATE_TIMESTAMP_THRESHOLD) {
    spot.lastChangeTimestamp = millis();
  }
}

const unsigned long ALL_LED_TIMEOUT = 1000 * 10; // 10 seconds
void enableAllLEDsIfNeeded(ParkingSpot& spot) {
  unsigned long currentTime = millis();
  // Handle the rollover of millis after ~41 days
  if(currentTime < spot.lastChangeTimestamp) {
    spot.lastChangeTimestamp = currentTime;
  }
  spot.enableAllLEDs = currentTime - spot.lastChangeTimestamp < ALL_LED_TIMEOUT;
}

const int CUT_LIGHT_THRESHOLD = 50;
void enableCutLightsIfNeeded(ParkingSpot& spot) {
  spot.enableCutLights = abs(spot.config.intersectDistance - spot.doorIntersectDistance) < CUT_LIGHT_THRESHOLD;
}