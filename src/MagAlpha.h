/***************************************************
  Arduino library for the MPS MagAlpha magnetic angle sensor
  Supports MagAlpha 3rd generation Sensors. MagAlpha sensor detects the
  absolute angular position of a permanent magnet, typically a diametrically
  magnetized cylinder on the rotating shaft.
  ----> http://www.monolithicpower.com/Products/Position-Sensors/Products-Overview
  Written by Mathieu Kaelin for Monolithic Power Systems.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#ifndef MAGALPHA_H
#define MAGALPHA_H

#if (ARDUINO >= 100)
     #include "Arduino.h"
#else
     #include "WProgram.h"
#endif
#include "MagAlphaGen3.h"

//SPI Mode: MagAlpha Gen3 support SPI mode 3 and 0 [SPI_MODE3, SPI_MODE0]
#define MA_SPI_MODE_0       SPI_MODE0
#define MA_SPI_MODE_3       SPI_MODE3

class MagAlpha: public MagAlphaGen3 {};

#endif //MAGALPHA_H
