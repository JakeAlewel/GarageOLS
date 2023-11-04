struct StoredConfiguration {
  int intersectDistance;
  int backInDistance;
};

struct ParkingSpotPins {
  int cutLights;
};

struct ParkingSpot {
  StoredConfiguration config;
  ParkingSpotPins pins;

  int doorIntersectDistance = 0;
  int backInDistance = 0;

  bool enableAllLEDs = false;
  bool enableCutLights = false;

  unsigned long lastChangeTimestamp;
};