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

 //SPI Mode: MagAlpha Gen3 support SPI mode 3 and 0 [SPI_MODE3, SPI_MODE0]
#define MA_SPI_MODE_0       SPI_MODE0
#define MA_SPI_MODE_3       SPI_MODE3

class MagAlpha {
public:
    MagAlpha();
    void  begin(uint8_t spiChipSelectPin);
    void  begin(int32_t spiSclkFrequency, uint8_t spiMode, uint8_t spiChipSelectPin);
    void end();
    double readAngle();
    uint16_t readAngleRaw();
    uint16_t readAngleRaw(bool* error);
    uint16_t readAngleRaw16();
    uint8_t readAngleRaw8();
    uint8_t readRegister(uint8_t address);
    uint8_t writeRegister(uint8_t address, uint8_t value);
    void setSpiClockFrequency(uint32_t speedMaximum);
    void setSpiDataMode(uint8_t spiMode);
    void setSpiChipSelectPin(uint8_t spiChipSelectPin);
    double convertRawAngleToDegree(uint8_t rawAngleDataBitLength, uint16_t rawAngle);
private:
    uint32_t _speedMaximum;
    uint8_t _spiMode;
    uint8_t _spiChipSelectPin;
};

class MagAlphaSSI {
public:
    MagAlphaSSI();
    void  begin();
    void  begin(int32_t ssiSsckFrequency);
    void end();
    double readAngle();
    uint16_t readAngleRaw();
    uint16_t readAngleRaw(bool* error);
    void setSsiClockFrequency(uint32_t speedMaximum);
    double convertRawAngleToDegree(uint8_t rawAngleDataBitLength, uint16_t rawAngle);
private:
    uint32_t _speedMaximum;
};
#endif //MAGALPHA_H
