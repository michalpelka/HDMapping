// Force-included into the Hesai LiDAR SDK 2.0 and the files that use it on macOS (see cmake/dependencies.cmake).
#pragma once
#include "endian.h"
// Linux termios2 serial ioctls, used only by the SDK's RS485 client (JT16). Defined so the SDK builds; the ioctl
// then fails at run time. Reading captures never uses them.
#define TCGETS2 0
#define TCSETS2 0
#define CBAUD 0
