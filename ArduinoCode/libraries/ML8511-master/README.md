ML8511 UV sensor library for Arduino.

![img](https://github.com/lnquy065/ML8511/blob/master/ml8511-2.png?raw=true)

### Usage

```c++
#include "lib/ML8511.h"

String uvString;

#define SENSOR_PIN_UV_VREF A1
#define SENSOR_PIN_UV_VO A0

ML8511 sensorUV(SENSOR_PIN_UV_VO, SENSOR_PIN_UV_VREF);

void run() {
  uvString = String(sensorUV.readUVIntensity());
}

```

### Schematic

![img](https://github.com/lnquy065/ML8511/blob/master/ml8511-Schematic.png?raw=true)
