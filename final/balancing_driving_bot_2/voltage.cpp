#include "voltage.h"
#include <Arduino.h>

void voltageInit()
{
  analogReference(INTERNAL);
}
