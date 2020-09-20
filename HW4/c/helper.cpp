#include "helper.h"

#include <Arduino.h>          // Serial

void printOffsetValues(const SensorData* offset)
{
  Serial.print("offset values, x: ");
  Serial.print(offset->x);
  Serial.print(", y: ");
  Serial.print(offset->y);
  Serial.print(", z: ");
  Serial.println(offset->z);
}

void printSensorData(const SensorData* data)
{
  Serial.print("x: \t");   Serial.print(data->x);
  Serial.print("\ty: \t"); Serial.print(data->y);
  Serial.print("\tz: \t"); Serial.println(data->z);
}

void printAngleData(const AngleData* data, int decimalPlaces)
{
  Serial.print("x: \t");   Serial.print(data->x, decimalPlaces);
  Serial.print("\ty: \t"); Serial.print(data->y, decimalPlaces);
  Serial.print("\tz: \t"); Serial.println(data->z, decimalPlaces);
}


void printAngles(double roll, double pitch)
{
  Serial.print("roll: ");
  Serial.print(roll);
  Serial.print("\tpitch: ");
  Serial.println(pitch);
}
