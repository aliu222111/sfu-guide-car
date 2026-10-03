// Compiles the firmware sketch as C++ against the host Arduino core.
// FIRMWARE_INO is the sketch's absolute path, passed by firmware.py.
#include "Arduino.h"

#define STR2(x) #x
#define STR(x) STR2(x)
#include STR(FIRMWARE_INO)
