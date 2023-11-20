#include "types.h"
#include "lidar.h"
#include <FastLED.h>
#include <TFMPlus.h>
#include "printf.h"   // Modified to support Intel based Arduino
#include <EEPROM.h>
#include <SoftwareSerial.h>

const SoftwareSerial sensorSerial1 (2, 3, false);
const SoftwareSerial sensorSerial2 (4, 5, false);
// const SoftwareSerial sensorSerial3 (14, 13, false);
// const SoftwareSerial sensorSerial4 (16, 15, false);

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
const int RANGE_LIGHT_COUNT = 13;

const int CONFIG_BUTTON_PIN = 12;
bool configurationButtonPressed = false;

bool enableDebugLED = false;

const float EMERGENCY_WAVE_OFF_THRESHOLD = -0.66f;
const float WAVE_OFF_THRESHOLD = -0.1f;
const float ABS_EMERGENCY_WAVE_OFF_THRESHOLD = abs(EMERGENCY_WAVE_OFF_THRESHOLD);
const float ABS_WAVE_OFF_THRESHOLD = abs(WAVE_OFF_THRESHOLD);

const int MAX_BRIGHTNESS = 20;

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
    FastLED.setBrightness(0);
    writeLEDRange(spots[i].leds, 0, spots[i].NUM_OLS_LEDS, CRGB::Black);

    // Render LEDs
    if(spots[i].enableAllLEDs) {
      FastLED.setBrightness(MAX_BRIGHTNESS);

      // Solve For Range
      int configDistance = spots[i].config.backInDistance;
      int currentDistance = spots[i].backInDistance;
      float targetRange = 2.0f * (float)configDistance / 3.0f;
      float progress = (currentDistance - configDistance) / ((2.0f/3.0f) * targetRange);

      // Cut Lights
      CRGB::HTMLColorCode cutLightBarColor = (spots[i].enableCutLights && progress > EMERGENCY_WAVE_OFF_THRESHOLD) ? CRGB::Green : CRGB::Red;
      writeLEDRange(spots[i].leds, 0, 7, cutLightBarColor);
      writeLEDRange(spots[i].leds, 12, 13, cutLightBarColor);
      writeLED(spots[i].leds, 35, cutLightBarColor);
      writeLEDRange(spots[i].leds, 40, 48, cutLightBarColor);
      CRGB::HTMLColorCode cutLightColor = (spots[i].enableCutLights && progress > EMERGENCY_WAVE_OFF_THRESHOLD) ? CRGB::Green : CRGB::Black;
      writeLEDRange(spots[i].leds, 16, 17, cutLightColor);
      writeLEDRange(spots[i].leds, 37, 38, cutLightColor);

      // Wave off lights
      CRGB::HTMLColorCode emergencyWaveOffLightColor = CRGB::Black;
      CRGB::HTMLColorCode waveOffLightColor = CRGB::Black;
      if(progress < EMERGENCY_WAVE_OFF_THRESHOLD ) {
        emergencyWaveOffLightColor = CRGB::Red;
        waveOffLightColor = CRGB::Red;
      } else if (progress < WAVE_OFF_THRESHOLD) {
        waveOffLightColor = CRGB::Orange;
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

      if(absProgress > ABS_EMERGENCY_WAVE_OFF_THRESHOLD ) {
        rangeLightColor = CRGB::Red;
      } else if (absProgress > ABS_WAVE_OFF_THRESHOLD) {
        rangeLightColor = CRGB::Orange;
      } else {
        rangeLightColor = CRGB::Green;
      }

      int centerLEDIndex = RANGE_LIGHT_COUNT / 2;
      float progressCenteredLEDIndex = -progress * centerLEDIndex + centerLEDIndex;

      for (int j = 0; j < RANGE_LIGHT_COUNT; j++) {
        float distanceToCenter = abs(j - progressCenteredLEDIndex) / 1.75f;
        float clippedFade = clip(distanceToCenter * 255, 0, 255);
        
        spots[i].leds[18 + j] = blend(rangeLightColor, CRGB::Black, (uint8_t)clippedFade);
        fadeToBlackBy()
      }
    }
  }
  FastLED.show();

  // Update Config on Button Press
  updateConfigStateIfNeeded();

  // Blink Debug LED
  enableDebugLED = !enableDebugLED;
  digitalWrite(LED_BUILTIN, enableDebugLED ? HIGH : LOW);
}

float clip(float n, float lower, float upper) {
  return max(lower, min(n, upper));
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