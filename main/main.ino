#include <EEPROM.h>
#include "types.h"
#include "lidar.h"

const int CLOCK_RATE_HZ = 10;
const int TICK = 1000 / CLOCK_RATE_HZ;

ParkingSpotPins createPins(int cutLights) {
  ParkingSpotPins p;
  p.cutLights = cutLights;
  return p;
}

ParkingSpot createSpot(ParkingSpotPins pins) {
  // TODO: Store Pin locations and pass in as parameters to this constructor
  ParkingSpot s;
  s.pins = pins;
  s.backInDistance = 0;
  s.doorIntersectDistance = 0;
  s.enableAllLEDs = false;
  s.enableCutLights = false;
  s.lastChangeTimestamp = millis();
  return s;
}

const int NUMBER_OF_SPOTS = 2;
ParkingSpot spot1 = createSpot(createPins(13));
ParkingSpot spot2 = createSpot(createPins(9));
ParkingSpot spots[NUMBER_OF_SPOTS] = {spot1, spot2};

void setup() {
  // Setup Serial
  Serial.begin(9600);

  // Load Configuration From Disk
  int address = 0;
  EEPROM.get(address, spot1.config);
  address += sizeof(spot1Config);
  EEPROM.get(address, spot2.config);

  // Initialize Pins
  initializeSensorPins();

  pinMode(spot1.pins.cutLights, OUTPUT);
  pinMode(spot2.pins.cutLights, OUTPUT);
  
  // DEBUG LED PIN
  // TODO: Remove
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
      // digitalWrite(spots[i].pins.cutLights, LOW);
      // TODO: Write LOW to other LEDs
    } else {
      // digitalWrite(spots[i].pins.cutLights, spots[i].enableCutLights ? HIGH : LOW);
      // TODO: Write GlideSlopeLeds
    }
  }

  // Delay
  delay(TICK);

  // DEBUG LED
  // TODO: Remove
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(100);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);                      // wait for a second
}

void updateDoorIntersectDistance(ParkingSpot& spot) {
  int previousDistance = spot.doorIntersectDistance;
  spot.doorIntersectDistance = getSensorValue(spot);
  if(previousDistance != spot.doorIntersectDistance) {
    spot.lastChangeTimestamp = millis();
  }
}

void updateBackInDistance(ParkingSpot& spot) {
  int previousDistance = spot.backInDistance;
  spot.backInDistance = getSensorValue(spot);
  if(previousDistance != spot.backInDistance) {
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

const int CUT_LIGHT_THRESHOLD = 10;
void enableCutLightsIfNeeded(ParkingSpot& spot) {
  spot.enableCutLights = abs(spot.config.intersectDistance - spot.doorIntersectDistance) < CUT_LIGHT_THRESHOLD;
}