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
#include <SPI.h>
#include "MagAlphaGen3.h"
#include "MagAlphaPartProperties.h"
 //SPI Mode: MagAlpha Gen3 support SPI mode 3 and 0 [SPI_MODE3, SPI_MODE0]
#define MA_SPI_MODE_0       SPI_MODE0
#define MA_SPI_MODE_3       SPI_MODE3

#include "MagAlphaBase.h"

/*====================================================================================*/
/*========================== MagAlphaGen3 Legacy =====================================*/
/*====================================================================================*/
class MagAlpha : public MagAlphaGen3 {
public:
    MagAlpha();
    void begin(uint8_t spiChipSelectPin);
    void begin(int32_t spiSclkFrequency, uint8_t spiMode, uint8_t spiChipSelectPin);
    void setSpiDataMode(uint8_t spiMode);
    uint16_t readAngleRaw();
    uint16_t readAngleRaw(bool* error);
};

#endif //MAGALPHA_H
