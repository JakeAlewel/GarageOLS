#include <TFMPlus.h>
#include <SoftwareSerial.h>
#include <FastLED.h>

struct StoredConfiguration {
  int intersectDistance;
  int backInDistance;
};

struct ParkingSpotPins {
  int ols;
};

struct ParkingSpot {
  StoredConfiguration config;
  ParkingSpotPins pins;

  int doorIntersectDistance = 0;
  SoftwareSerial doorIntersectSerial;
  TFMPlus doorIntersectSensor;

  int backInDistance = 0;
  SoftwareSerial backInSerial;
  TFMPlus backInSensor;

  bool enableAllLEDs = false;
  bool enableCutLights = false;

  unsigned long lastChangeTimestamp;

  static const int NUM_OLS_LEDS = 50;
  CRGB leds[NUM_OLS_LEDS];

  ParkingSpot(ParkingSpotPins _pins, SoftwareSerial _backIn, SoftwareSerial _intersect): pins(_pins), backInSerial(_backIn), doorIntersectSerial(_intersect) {};
};