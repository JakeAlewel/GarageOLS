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

const float EMERGENCY_WAVE_OFF_THRESHOLD = 2/3.0f;
const float WAVE_OFF_THRESHOLD = 1/13.0f;

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

    // Clear All LEDs
    writeLEDRange(spots[i].leds, 0, spots[i].NUM_OLS_LEDS, CRGB::Black);

    // Render LEDs
    if(spots[i].enableAllLEDs) {

      // Solve For Range
      int configDistance = spots[i].config.backInDistance;
      int currentDistance = spots[i].backInDistance;
      int step = (configDistance / 6);
      int warnBreakpoint = configDistance - (step * 2);
      float progress = (float)(configDistance - currentDistance) / warnBreakpoint;

      // Cut Lights
      CRGB::HTMLColorCode cutLightBarColor = (spots[i].enableCutLights && progress <= EMERGENCY_WAVE_OFF_THRESHOLD) ? CRGB::Green : CRGB::Red;
      writeLEDRange(spots[i].leds, 0, 7, cutLightBarColor);
      writeLEDRange(spots[i].leds, 12, 13, cutLightBarColor);
      writeLED(spots[i].leds, 35, cutLightBarColor);
      writeLEDRange(spots[i].leds, 40, 48, cutLightBarColor);
      CRGB::HTMLColorCode cutLightColor = (spots[i].enableCutLights && progress <= EMERGENCY_WAVE_OFF_THRESHOLD) ? CRGB::Green : CRGB::Black;
      writeLEDRange(spots[i].leds, 16, 17, cutLightColor);
      writeLEDRange(spots[i].leds, 37, 38, cutLightColor);

      // Wave off lights
      CRGB::HTMLColorCode emergencyWaveOffLightColor = CRGB::Black;
      CRGB::HTMLColorCode waveOffLightColor = CRGB::Black;
      if(progress > EMERGENCY_WAVE_OFF_THRESHOLD ) {
        emergencyWaveOffLightColor = CRGB::Red;
        waveOffLightColor = CRGB::Red;
      } else if (progress > WAVE_OFF_THRESHOLD) {
        waveOffLightColor = CRGB::Yellow;
      }

      writeLEDRange(spots[i].leds, 9, 10, emergencyWaveOffLightColor);
      writeLEDRange(spots[i].leds, 31, 32, emergencyWaveOffLightColor);
      
      writeLED(spots[i].leds, 8, waveOffLightColor);
      writeLED(spots[i].leds, 11, waveOffLightColor);
      writeLED(spots[i].leds, 14, waveOffLightColor);
      writeLED(spots[i].leds, 15, waveOffLightColor);
      writeLED(spots[i].leds, 33, waveOffLightColor);
      writeLED(spots[i].leds, 34, waveOffLightColor);
      writeLED(spots[i].leds, 36, waveOffLightColor);
      writeLED(spots[i].leds, 39, waveOffLightColor);

      // Meatball Lights
      float absProgress = abs(progress);
      CRGB::HTMLColorCode rangeLightColor = CRGB::Black;

      if(absProgress > EMERGENCY_WAVE_OFF_THRESHOLD ) {
        rangeLightColor = CRGB::Red;
      } else if (absProgress > WAVE_OFF_THRESHOLD) {
        rangeLightColor = CRGB::Yellow;
      } else {
        rangeLightColor = CRGB::Green;
      }

      int rangeLightsStart = 18;
      int rangeLightsCount = 13;
      for (int j = 0; j < 13; j++) {
        int isLit = progress * rangeLightsCount / 2 + (float)rangeLightsCount / 2;
        spots[i].leds[j + rangeLightsStart] = abs(isLit - j) <= 1 ? rangeLightColor : CRGB::Black;
      }
    }
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

void writeLED(CRGB leds[], int index, CRGB::HTMLColorCode value) {
  leds[index] = value;
}

void writeLEDRange(CRGB leds[], int start, int end, CRGB::HTMLColorCode value) {
  for (int i = start; i <= end; i++) {
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
const int UPDATE_TIMESTAMP_THRESHOLD = 2;
void updateDoorIntersectDistance(ParkingSpot& spot) {
  spot.doorIntersectDistance = getSensorValue(spot, false);

  if(abs(spot.lastChangeDoorIntersect - spot.doorIntersectDistance) > UPDATE_TIMESTAMP_THRESHOLD) {
    spot.lastChangeTimestamp = millis();
    spot.lastChangeDoorIntersect = spot.doorIntersectDistance;
  }
}

void updateBackInDistance(ParkingSpot& spot) {
  spot.backInDistance = getSensorValue(spot, true);

  if(abs(spot.lastChangeBackIn - spot.backInDistance) > UPDATE_TIMESTAMP_THRESHOLD) {
    spot.lastChangeTimestamp = millis();
    spot.lastChangeBackIn = spot.backInDistance;
  }
}

const unsigned long ALL_LED_TIMEOUT = (unsigned long)1000UL * 20UL; // 20 seconds
void enableAllLEDsIfNeeded(ParkingSpot& spot) {
  unsigned long currentTime = millis();
  // Handle the rollover of millis after ~41 days
  if(currentTime < spot.lastChangeTimestamp) {
    Serial.println(spot.lastChangeTimestamp);
    spot.lastChangeTimestamp = currentTime;
  }

  spot.enableAllLEDs = currentTime - spot.lastChangeTimestamp < ALL_LED_TIMEOUT;
}

const int CUT_LIGHT_THRESHOLD = 50;
void enableCutLightsIfNeeded(ParkingSpot& spot) {
  spot.enableCutLights = abs(spot.config.intersectDistance - spot.doorIntersectDistance) < CUT_LIGHT_THRESHOLD;
}