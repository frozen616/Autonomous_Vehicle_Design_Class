#include "helper.h"

#include <Arduino.h>          // Serial

void printOffsetValues(const SensorData* offset)
{
  Serial.print("offset values, xA: ");
  Serial.print(offset->x);
  Serial.print(", yA: ");
  Serial.print(offset->y);
  Serial.print(", zA: ");
  Serial.println(offset->z);

  Serial.print("offset values, xG: ");
  Serial.print(offset->xG);
  Serial.print(", yG: ");
  Serial.print(offset->yG);
  Serial.print(", zG: ");
  Serial.println(offset->zG);
}
