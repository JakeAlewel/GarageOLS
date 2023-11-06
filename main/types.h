#include <TFMPlus.h>
#include <SoftwareSerial.h>

struct StoredConfiguration {
  int intersectDistance;
  int backInDistance;
};

struct ParkingSpotPins {
  int cutLights;
  int notCutLights;
  int good;
  int warn;
  int bad;
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

  ParkingSpot(ParkingSpotPins _pins, SoftwareSerial _backIn, SoftwareSerial _intersect): pins(_pins), backInSerial(_backIn), doorIntersectSerial(_intersect) {};
};