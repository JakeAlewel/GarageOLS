#include <EEPROM.h>
#include "types.h"
#include "lidar.h"
#include <TFMPlus.h>
#include "printf.h"   // Modified to support Intel based Arduino
#include <SoftwareSerial.h>
#include <FastLED.h>

const SoftwareSerial sensorSerial1 (2, 3, false);
const SoftwareSerial sensorSerial2 (4, 5, false);
// const SoftwareSerial sensorSerial3 (14, 13, false);
// const SoftwareSerial sensorSerial4 (16, 15, false);

const int CLOCK_RATE_HZ = 10;
const int TICK = 1000 / CLOCK_RATE_HZ;

ParkingSpotPins createPins(int ols) {
  ParkingSpotPins p;
  p.ols = ols;
  return p;
}

const int SPOT_ONE_OLS_PIN = 11;
const int SPOT_TWO_OLS_PIN = 10;

ParkingSpot spot1 (createPins(SPOT_ONE_OLS_PIN), sensorSerial1, sensorSerial2);
ParkingSpot spot2 (createPins(SPOT_TWO_OLS_PIN), sensorSerial1, sensorSerial2);

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

  spots[0].lastChangeTimestamp = millis();
  spots[1].lastChangeTimestamp = millis();

  printf_begin(); 

  // Load Configuration From Disk
  int address = 0;
  EEPROM.get(address, spots[0].config);
  address += sizeof(spots[0].config);
  EEPROM.get(address, spots[1].config);

  // Initialize Pins
  initializeSensorPins(spots);
  pinMode(CONFIG_BUTTON_PIN, INPUT);

  FastLED.addLeds<WS2811, SPOT_ONE_OLS_PIN, RGB>(spots[0].leds, spots[0].NUM_OLS_LEDS);  
  // FastLED.addLeds<WS2811, SPOT_TWO_OLS_PIN, RGB>(spots[1].leds, spots[1].NUM_OLS_LEDS);  
  

  FastLED.setBrightness(20);
  
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
      writeLEDRange(spots[i].leds, 0, spots[i].NUM_OLS_LEDS, CRGB::Black);
      // for(int j = 0; j < spots[i].NUM_OLS_LEDS; j++) {
      //   spots[i].leds[j] = CRGB::Black;
      // }
    } else {
      writeLEDRange(spots[i].leds, 0, spots[i].NUM_OLS_LEDS, CRGB::Black);
      // for(int j = 0; j < spots[i].NUM_OLS_LEDS; j++) {
      //   spots[i].leds[j] = CRGB::Black;
      // }

      CRGB::HTMLColorCode cutLightColor = spots[i].enableCutLights ? CRGB::Green : CRGB::Black;
      writeLEDRange(spots[i].leds, 30, spots[i].NUM_OLS_LEDS, cutLightColor);
      writeLEDRange(spots[i].leds, 7, 8, cutLightColor);
      writeLEDRange(spots[i].leds, 11, 12, cutLightColor);


      // digitalWrite(spots[i].pins.cutLights, spots[i].enableCutLights ? HIGH : LOW);
      // digitalWrite(spots[i].pins.notCutLights, spots[i].enableCutLights ? LOW : HIGH);


      // int configDistance = spots[i].config.backInDistance;
      // int step = (configDistance / 6);
      // int goodBreakpoint = configDistance - step;
      // int warnBreakpoint = configDistance - (step * 2);
      // int currentDistance = spots[i].backInDistance;

      // digitalWrite(spots[i].pins.good, configDistance > currentDistance && currentDistance >= goodBreakpoint ? HIGH : LOW);
      // digitalWrite(spots[i].pins.warn, goodBreakpoint > currentDistance && currentDistance > warnBreakpoint ? HIGH : LOW);
      // digitalWrite(spots[i].pins.bad, warnBreakpoint >= currentDistance ? HIGH : LOW);
    }

    // FastLED[i].showLeds();
  }
  FastLED.show();

  // Update Config on Button Press
  updateConfigStateIfNeeded();

  // Blink Debug LED
  enableDebugLED = !enableDebugLED;
  digitalWrite(LED_BUILTIN, enableDebugLED ? HIGH : LOW);

  // Delay
  // delay(TICK);
}

void writeLEDRange(CRGB leds[], int start, int end, CRGB::HTMLColorCode value) {
  for (int i = start; i<= end; i++) {
    leds[i] = value;
  }
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
    Serial.println("Updating Timestamp Cut Light");
    spot.lastChangeTimestamp = millis();
  }
}

void updateBackInDistance(ParkingSpot& spot) {
  int previousDistance = spot.backInDistance;
  spot.backInDistance = getSensorValue(spot, true);
  if(abs(previousDistance - spot.backInDistance) > UPDATE_TIMESTAMP_THRESHOLD) {
    Serial.println("Updating Timestamp Back In");
    spot.lastChangeTimestamp = millis();
  }
}

const unsigned long ALL_LED_TIMEOUT = 1000 * 10; // 10 seconds
void enableAllLEDsIfNeeded(ParkingSpot& spot) {
  unsigned long currentTime = millis();
  // Handle the rollover of millis after ~41 days

  // if(spot.lastChangeTimestamp != 32768 && currentTime < spot.lastChangeTimestamp) {
  //   Serial.println(spot.lastChangeTimestamp);
  //   spot.lastChangeTimestamp = currentTime;
  // }

  Serial.println(String(currentTime) + " " + String(spot.lastChangeTimestamp) + " " + String(currentTime - spot.lastChangeTimestamp));
  spot.enableAllLEDs = currentTime - spot.lastChangeTimestamp < ALL_LED_TIMEOUT;
}

const int CUT_LIGHT_THRESHOLD = 50;
void enableCutLightsIfNeeded(ParkingSpot& spot) {
  spot.enableCutLights = abs(spot.config.intersectDistance - spot.doorIntersectDistance) < CUT_LIGHT_THRESHOLD;
}