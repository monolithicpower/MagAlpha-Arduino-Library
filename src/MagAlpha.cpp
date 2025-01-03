/***************************************************
  Arduino library for the MPS MagAlpha magnetic angle sensor
  Supports MagAlpha 3rd generation Sensors. MagAlpha sensor detects the
  absolute angular position of a permanent magnet, typically a diametrically
  magnetized cylinder on the rotating shaft.
  ----> http://www.monolithicpower.com/Products/Position-Sensors/Products-Overview
  Written by Mathieu Kaelin for Monolithic Power Systems.
  MIT license, all text above must be included in any redistribution
****************************************************/

#include <SPI.h>
#include "MagAlpha.h"

//MagAlpha Read/Write Register Command
//#define READ_REG_COMMAND    (0b010 << 13)
//#define WRITE_REG_COMMAND   (0b100 << 13)

#define SSI_MODE            SPI_MODE1

/*====================================================================================*/
/*========================== MagAlphaGen3 Legacy =====================================*/
/*====================================================================================*/
MagAlpha::MagAlpha(){
}

void  MagAlpha::begin(uint8_t spiChipSelectPin){
    MagAlphaSPI::begin(10000000, MagAlphaSPIMode::MODE_3, spiChipSelectPin, &SPI);
}

void  MagAlpha::begin(int32_t spiSclkFrequency, uint8_t spiMode, uint8_t spiChipSelectPin){
    MagAlphaSPI::begin(spiSclkFrequency, (MagAlphaSPIMode)spiMode, spiChipSelectPin, &SPI);
}

void MagAlpha::setSpiDataMode(uint8_t spiMode){
    MagAlphaSPI::setSpiDataMode((MagAlphaSPIMode)spiMode);
}

uint16_t MagAlpha::readAngleRaw() {
    return MagAlphaGen3::readAngleRaw16();
}

uint16_t MagAlpha::readAngleRaw(bool* error) {
    return MagAlphaGen3::readAngleRaw(error);
}