#include "types.h"
#include <SoftwareSerial.h>
#include <TFMPlus.h>

void initializeSensorPins(ParkingSpot spots[]) {
  spots[0].backInSerial.begin(115200);
  spots[0].backInSensor.begin(&spots[0].backInSerial);
  spots[0].doorIntersectSerial.begin(115200);
  spots[0].doorIntersectSensor.begin(&spots[0].doorIntersectSerial);
}

int getSensorValue(ParkingSpot& spot, bool backIn) {
  String type = backIn ? "BackIn" : "Intersect";
  
  int distance, flux, temp = 0;
  SoftwareSerial& serial = backIn ? spot.backInSerial : spot.doorIntersectSerial;
  TFMPlus& sensor = backIn ? spot.backInSensor : spot.doorIntersectSensor;
  serial.listen();

  while(!distance) {
    sensor.getData(distance,flux, temp);
    if(!distance) {
      delay(50);
    }
  }
  
  return distance;
}