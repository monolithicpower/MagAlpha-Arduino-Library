/***************************************************
  Arduino library for the MPS MagAlpha magnetic angle sensor
  Supports MagAlpha 7th generation Sensors. 
  Support Part Number includes: MA900
  MagAlpha sensor detects the
  absolute angular position of a permanent magnet, typically a diametrically
  magnetized cylinder on the rotating shaft.
  ----> http://www.monolithicpower.com/Products/Position-Sensors/Products-Overview
  Written by Mathieu Kaelin for Monolithic Power Systems.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "MagAlphaGen7.h"

//CRC table for poly = x^4 + x^3 + x^2 + 1 (0x1d or 0b11101 or 29)
const uint8_t crc4LookupTable[16] = {0, 13, 7, 10, 14, 3, 9, 4, 1, 12, 6, 11, 15, 2, 8, 5};

const uint8_t poly_table[256] = {0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
                                0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
                                0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
                                0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
                                0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
                                0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A, 
                                0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
                                0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
                                0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
                                0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
                                0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44, 
                                0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
                                0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
                                0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13, 
                                0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
                                0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3};

#define REG_ZERO0 0
#define REG_ZERO1 1
#define REG_SENT0 8
#define REG_SENT1 9
#define REG_SENT2 10
#define REG_INTF 15
#define REG_FILT 17
#define REG_SPEEDFILT 18
#define REG_IOMC 20
#define REG_ID0 27
#define REG_ID1 28
#define REG_ID2 29
#define REG_ID3 30
#define REG_PID 31
#define REG_TS1 39
#define REG_LOCK0 52
#define REG_LOCK1 53
#define REG_LOCK2 54
#define REG_LOCK3 55
#define REG_SECRET 79
#define REG_ANGLE 96
#define REG_SPEED0 98
#define REG_MTV0 100
#define REG_TEMP0 102
#define REG_READC 107
#define REG_STATUS 108
#define REG_MTP 120
#define REG_FAULT 125
#define REG_LOCKST 126
#define REG_LOCK 127

#define SENT_FORMAT_H1 0
#define SENT_FORMAT_H2 1
#define SENT_FORMAT_H3 2 
#define SENT_FORMAT_H4 3
#define SENT_FORMAT_H5 4
#define SENT_FORMAT_H6 5
#define SENT_FORMAT_H7 6

#define IOMC_PULL_DOWN 0
#define IOMC_ABZ_UVW_PWM 1
#define IOMC_ABZ_PWM 2
#define IOMC_ABZ_SSI_PWM 3
#define IOMC_SSI_UVW 4
#define IOMC_UVW_DIFF 5
#define IOMC_ABZ_PBI 6
#define IOMC_UVW 7
#define IOMC_MGL_UVW 8
#define IOMC_SENT 9
#define IOMC_ABZ_MGL 10
#define IOMC_PWM_PBI 11
#define IOMC_MGL_SSI 12
#define IOMC_TRIG 13
#define IOMC_GPIO_MODE 15

/*====================================================================================*/
/*====================================== SPI =========================================*/
/*====================================================================================*/

MagAlphaGen7::MagAlphaGen7(){
    _crcCheckEnabled = true;
    _crcInitValue = 10;
}

uint16_t MagAlphaGen7::readAngleRaw(bool* error){
    bool inversion = false;
    return readAngleRaw16(error, &inversion, false);
}

uint16_t MagAlphaGen7::readAngleRaw16(){
    bool error = false;
    bool inversion = false;
    return readAngleRaw16(&error, &inversion, false);
}

uint16_t MagAlphaGen7::readAngleRaw16(bool *error, bool *inversion, bool isShortRead){
    uint16_t readbackValue, valueToWrite, computedCrc, angle;
    bool errorDetected;
    bool inversionDetected;
    *error = false;
    *inversion = false;
    valueToWrite = 0x0300 | REG_READC;
    valueToWrite = appendCrc4(valueToWrite);
    digitalWrite(_spiChipSelectPin, LOW);
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *error = errorDetected?true:*error;
    *inversion = inversionDetected?true:*inversion;
    angle = readbackValue & 0xFFF0;

    if(!isShortRead){
        valueToWrite=readbackValue;
        readbackValue = _spi->transfer16(valueToWrite);
        checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
        *error=errorDetected?true:*error;
        *inversion=inversionDetected?true:*inversion;
        angle |= (readbackValue>>12)&0x000F;
    }
    digitalWrite(_spiChipSelectPin, HIGH);
    return angle;
}

uint8_t MagAlphaGen7::readAngleRaw8(){
    bool error = false;
    bool inversion = false;
    return (readAngleRaw16(&error, &inversion, true) >> 8);
}

uint8_t MagAlphaGen7::readRegister(uint8_t address){
    bool error, inversion;
    return readRegister(address, &error, &inversion);
}

uint8_t MagAlphaGen7::writeRegister(uint8_t address, uint8_t value){
    bool error, inversion, wrongHandshaking;
    return writeRegister(address, value, &error, &inversion, &wrongHandshaking);
}

void MagAlphaGen7::readRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister){
    bool error, inversion;
    readRegisterBurst(address, valueArray, numberOfRegister, &error, &inversion);
}

void MagAlphaGen7::writeRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister){
    bool error, inversion, wrongHandshaking;
    writeRegisterBurst(address, valueArray, numberOfRegister, &error, &inversion, &wrongHandshaking);
}

uint16_t MagAlphaGen7::detectSensorGeneration(){
    bool error, inversion;
    return readRegister(REG_PID, &error, &inversion) >> 4;
}

uint16_t MagAlphaGen7::getZero(){
    bool error, inversion;
    return (readRegister(REG_ZERO1, &error, &inversion) << 8) | readRegister(REG_ZERO0, &error, &inversion);
}

void MagAlphaGen7::setZero(uint16_t zero){
    bool error, inversion, wrongHandshaking;
    writeRegister(REG_ZERO0, zero&0x00FF, &error, &inversion, &wrongHandshaking);
    writeRegister(REG_ZERO1, zero>>8, &error, &inversion, &wrongHandshaking);
}

uint16_t MagAlphaGen7::setCurrentAngleAsZero(){
    uint16_t zero;
    setZero(0);
    delay(10);
    zero = readAngleRaw16();
    setZero(zero);
    return zero;
}

void MagAlphaGen7::restoreAllRegisters(){
    bool error, inversion, wrongHandshaking;
    uint8_t tempReg;
    tempReg = readRegister(REG_MTP, &error, &inversion);
    tempReg |= 0x10;
    writeRegister(REG_MTP, tempReg, &error, &inversion, &wrongHandshaking);
    //wait for MTP DONE Flag to be set to 1 (indicate MTP Store/Restore operation is finished)
    tempReg=0;
    while((tempReg & 0x80) == 0){
        tempReg = readRegister(REG_MTP, &error, &inversion);
    }
}

void MagAlphaGen7::storeAllRegisters(){
    //Store Page 0 and Page 1
    storeRegisterBlock(3); 
}

void MagAlphaGen7::storeRegisterBlock(uint8_t block){
    bool error, inversion, wrongHandshaking;
    uint8_t tempReg = 0;
    tempReg = readRegister(REG_MTP, &error, &inversion);
    if(block <= 3){
        tempReg &= 0xFC;
        tempReg |= 0x20 | block;
        writeRegister(REG_MTP, tempReg, &error, &inversion, &wrongHandshaking);
    }
    //wait for MTP DONE Flag to be set to 1 (indicate MTP Store/Restore operation is finished)
    tempReg=0;
    while((tempReg & 0x80) == 0){
        tempReg = readRegister(REG_MTP, &error, &inversion);
    }
}

void MagAlphaGen7::clearErrorFlags(){
    bool error, inversion, wrongHandshaking;
    writeRegister(REG_STATUS, 0X07, &error, &inversion, &wrongHandshaking); 
}

double MagAlphaGen7::readSpeed(){
    uint16_t angle;
    bool error, inversion;
    return twosComplement(readAngleSpeed(&angle, &error, &inversion), 16) * 3.57;
}

int16_t MagAlphaGen7::readTurn(){
    uint16_t angle;
    bool error, inversion;
    return twosComplement(readAngleMultiturn(&angle, &error, &inversion), 16);
}

void MagAlphaGen7::writeTurn(int16_t turn){
    bool error, inversion, wrongHandshaking;
    uint16_t multiturn = twosComplementInverse(turn, 16);
    writeRegister(REG_MTV0, multiturn&0x00FF, &error, &inversion, &wrongHandshaking);
    writeRegister(REG_MTV0+1, multiturn>>8, &error, &inversion, &wrongHandshaking);
}

double MagAlphaGen7::readTemperature(){
    uint16_t angle;
    bool error, inversion;
    return twosComplement(readAngleTemperature(&angle, &error, &inversion), 16) * 0.125;
}

uint16_t MagAlphaGen7::readAcquisitionCounter(){
    uint16_t angle;
    bool error, inversion;
    return readAngleCounter(&angle, &error, &inversion);
}

uint16_t MagAlphaGen7::readAngleCounter(uint16_t *angle, bool *error, bool *inversion) {
    uint16_t readbackValue, valueToWrite, computedCrc, counter;
    bool errorDetected;
    bool inversionDetected;
    *error=false;
    *inversion=false;
    valueToWrite=(0x0300)|REG_READC;
    valueToWrite=appendCrc4(valueToWrite);
    digitalWrite(_spiChipSelectPin, LOW);
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    *angle=readbackValue&0xFFF0;
    
    valueToWrite=readbackValue;
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    *angle |= (readbackValue>>12)&0x000F;
    counter = (readbackValue>>4)&0x00FF;  
    digitalWrite(_spiChipSelectPin, HIGH);
    return counter;
}

uint16_t MagAlphaGen7::readAngleSpeed(uint16_t *angle, bool *error, bool *inversion) {
    uint16_t readbackValue, valueToWrite, computedCrc, speed;
    bool errorDetected;
    bool inversionDetected;
    *error=false;
    *inversion=false;
    valueToWrite=(0x0300)|REG_SPEED0;
    valueToWrite=appendCrc4(valueToWrite);
    digitalWrite(_spiChipSelectPin, LOW);
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    *angle=readbackValue&0xFFF0;
    
    valueToWrite=readbackValue;
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    *angle |= (readbackValue>>12)&0x000F;
    speed = (readbackValue>>4)&0x00FF;

    valueToWrite=readbackValue;
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    speed |= (readbackValue<<4)&0xFF00;
    digitalWrite(_spiChipSelectPin, HIGH);
    return speed;
}

uint16_t MagAlphaGen7::readAngleMultiturn(uint16_t *angle, bool *error, bool *inversion) {
    uint16_t readbackValue, valueToWrite, computedCrc, multiturn;
    bool errorDetected;
    bool inversionDetected;
    *error=false;
    *inversion=false;
    valueToWrite=(0x0300)|REG_MTV0;
    valueToWrite=appendCrc4(valueToWrite);
    digitalWrite(_spiChipSelectPin, LOW);
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    *angle=readbackValue&0xFFF0;
    
    valueToWrite=readbackValue;
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    *angle |= (readbackValue>>12)&0x000F;
    multiturn = (readbackValue>>4)&0x00FF;

    valueToWrite=readbackValue;
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    multiturn |= (readbackValue<<4)&0xFF00;
    digitalWrite(_spiChipSelectPin, HIGH);
    return multiturn;
}

uint16_t MagAlphaGen7::readAngleTemperature(uint16_t *angle, bool *error, bool *inversion){
    uint16_t readbackValue, valueToWrite, computedCrc, temperature;
    bool errorDetected;
    bool inversionDetected;
    *error=false;
    *inversion=false;
    valueToWrite=(0x0300)|REG_TEMP0;
    valueToWrite=appendCrc4(valueToWrite);
    digitalWrite(_spiChipSelectPin, LOW);
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    *angle=readbackValue&0xFFF0;
    
    valueToWrite=readbackValue;
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    *angle |= (readbackValue>>12)&0x000F;
    temperature = (readbackValue>>4)&0x00FF;

    valueToWrite=readbackValue;
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    temperature |= (readbackValue<<4)&0xFF00;
    digitalWrite(_spiChipSelectPin, HIGH);
    return temperature;
}

uint8_t MagAlphaGen7::readRegister(uint8_t address, bool *error, bool *inversion) {
    uint16_t readbackValue, valueToWrite, computedCrc;
    bool errorDetected;
    bool inversionDetected;
    *error=false;
    *inversion=false;
    valueToWrite=(0x0300)|(address);
    valueToWrite=appendCrc4(valueToWrite);
    digitalWrite(_spiChipSelectPin, LOW);
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    
    //The previously received 16-bit data on CIPO (12-bit AngleMSB + 4-bit CRC4) is returned by the controller on COPI (handshaking protocol).
    //We do not compute the CRC of the previously received data we just send the exact same data+CRC back to the peripheral.
    valueToWrite=readbackValue;
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    digitalWrite(_spiChipSelectPin, HIGH);
    return ((readbackValue>>4)&0x00FF);
}

uint16_t MagAlphaGen7::readRegisterBurst(uint8_t address, uint8_t readbackValueArray[], uint16_t numberOfRegister, bool *error, bool *inversion) {
    uint16_t readbackValue, valueToWrite, computedCrc, angle;
    bool errorDetected;
    bool inversionDetected;
    *error=false;
    *inversion=false;
    valueToWrite=(0x0300)|(address);
    valueToWrite=appendCrc4(valueToWrite);
    digitalWrite(_spiChipSelectPin, LOW);
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    angle=readbackValue&0xFFF0;

    for (uint8_t i=0;i<numberOfRegister;++i) {
        valueToWrite=readbackValue;
        readbackValue = _spi->transfer16(valueToWrite);
        checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
        *error=errorDetected?true:*error;
        *inversion=inversionDetected?true:*inversion;
        readbackValueArray[i]=(readbackValue>>4)&0xFF;
        if (i==0) {
        angle=angle | ((readbackValue>>12)&0x000F);
        }
    }
    digitalWrite(_spiChipSelectPin, HIGH);
    return angle;
}

uint8_t MagAlphaGen7::writeRegister(uint8_t address, uint8_t value, bool *error, bool *inversion, bool *wrongHandshaking) {
    uint16_t readbackValue, valueToWrite, writtenValue, computedCrc;
    bool errorDetected, inversionDetected;
    
    *error=false;
    *inversion=false;
    *wrongHandshaking = false;
    
    valueToWrite=(0x0C00)|(address);
    valueToWrite=appendCrc4(valueToWrite);
    digitalWrite(_spiChipSelectPin, LOW);
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    writtenValue = valueToWrite;

    valueToWrite=(0x0000)|(value);
    valueToWrite=appendCrc4(valueToWrite);
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *wrongHandshaking = ((readbackValue&0x0FF0) != (writtenValue&0x0FF0))?true:*wrongHandshaking; //in this readback we get the angle LSBs instead of OpCode
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    writtenValue = valueToWrite;
    
    valueToWrite=0x0F00;
    valueToWrite=appendCrc4(valueToWrite);
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *wrongHandshaking = (readbackValue != writtenValue)?true:*wrongHandshaking;
    
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    digitalWrite(_spiChipSelectPin, HIGH);
    return (readbackValue & 0x0FF0) >> 4;
}

void MagAlphaGen7::writeRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister, bool *error, bool *inversion, bool *wrongHandshaking) {
    uint16_t readbackValue, valueToWrite, writtenValue, computedCrc;
    bool errorDetected, inversionDetected;
    
    *error=false;
    *inversion=false;
    *wrongHandshaking = false;
    
    valueToWrite=(0x0C00)|(address);
    valueToWrite=appendCrc4(valueToWrite);
    digitalWrite(_spiChipSelectPin, LOW);
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    writtenValue = valueToWrite;

    for (uint8_t i=0;i<numberOfRegister;++i) {
        valueToWrite=(0x0000)|(valueArray[i]);
        valueToWrite=appendCrc4(valueToWrite);
        readbackValue = _spi->transfer16(valueToWrite);
        checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
        if(i == 0){
            *wrongHandshaking = ((readbackValue&0x0FF0) != (writtenValue&0x0FF0))?true:*wrongHandshaking; //in this readback I get the angle LSBs instead of OpCode
        }
        else{
            *wrongHandshaking = (readbackValue != writtenValue)?true:*wrongHandshaking;
        } 
        *error=errorDetected?true:*error;
        *inversion=inversionDetected?true:*inversion;
        writtenValue = valueToWrite;
    }

    valueToWrite=0x0F00;
    valueToWrite=appendCrc4(valueToWrite);
    readbackValue = _spi->transfer16(valueToWrite);
    checkCrc4(readbackValue, &computedCrc, &errorDetected, &inversionDetected);
    *error=errorDetected?true:*error;
    *inversion=inversionDetected?true:*inversion;
    *wrongHandshaking = (readbackValue != writtenValue)?true:*wrongHandshaking;
    digitalWrite(_spiChipSelectPin, HIGH);
}

void MagAlphaGen7::setCrcCheckSetting(bool enable){
    bool error, inversion, wrongHandshaking;
    uint8_t tempReg;
    tempReg = readRegister(REG_INTF, &error, &inversion);
    if(enable){
        writeRegister(REG_INTF, tempReg|0x01, &error, &inversion, &wrongHandshaking);
        _crcCheckEnabled = true;
    }
    else{
        writeRegister(REG_INTF, tempReg&0xFE, &error, &inversion, &wrongHandshaking);
        _crcCheckEnabled = false;     
    }
}

uint16_t MagAlphaGen7::appendCrc4(uint16_t data){
    uint8_t crc4;                                   // 4-bit CRC (0-15)
    // Compute CRC4
    crc4 = _crcInitValue;                           // Initial Value for CRC4 (Seed)
    crc4 = (data>>8 & 0xF) ^ crc4LookupTable[crc4]; // 4 MSB first
    crc4 = (data>>4 & 0xF) ^ crc4LookupTable[crc4]; // 
    crc4 = (data>>0 & 0xF) ^ crc4LookupTable[crc4]; // 4 LSB
    // Concatenate 12-bit data with CRC4
    return (data<<4) | crc4;                      // 16-bit word to send <12-bit data><4-bit crc>
}

void MagAlphaGen7::checkCrc4(uint16_t readData, uint16_t *computedCrc, bool *errorDetected, bool *inversionDetected){
    *computedCrc=appendCrc4(readData>>4);
    //compare the data + CRC received VS the expected data + CRC
    if (readData==*computedCrc){
        *errorDetected=false;
        *inversionDetected = false;
    }
    else{
        *errorDetected=true;
        if(((~readData) & 0xF) == (*computedCrc & 0xF)){
            *inversionDetected = true; //if the CRC is not the same, check if it was inverted
        }
        else{
            *inversionDetected = false;
        } 
    }
}

uint16_t MagAlphaGen7::getNumberOfRegisters(){
    return 128;
}

void MagAlphaGen7::getLockedRegisters(uint8_t unlockedRegisters[]){
    uint8_t registerArray[4] = {0};
    readRegisterBurst(REG_LOCK0, registerArray, 4);
    for(uint8_t i=0; i<4; i++){
        for(uint8_t j=0; j<8; j++){
            unlockedRegisters[i*8+j] = ((registerArray[i]>>j)&0x01) ? 0xFF : 0x00;
        }
    }
}

void MagAlphaGen7::getPartNumber(char *partNumber){
    sprintf(partNumber, "MA900");
}

uint8_t MagAlphaGen7::getSiliconId(){
    return readRegister(REG_PID);
}

uint8_t MagAlphaGen7::getSiliconRevision(){
    uint8_t siliconRevision0, siliconRevision1, siliconRevision2, siliconRevision3;
    siliconRevision0 = readRegister(REG_ID0);
    siliconRevision1 = readRegister(REG_ID1);
    siliconRevision2 = readRegister(REG_ID2);
    siliconRevision3 = readRegister(REG_ID3);
    return siliconRevision0;
}

/*====================================================================================*/
/*====================================== I2C =========================================*/
/*====================================================================================*/

MagAlphaI2CGen7::MagAlphaI2CGen7(){
    _crcInitValue = 0;
    _crcCheckEnabled = true;
    _deviceAddress = 0x14;
}

uint16_t MagAlphaI2CGen7::readAngleRaw(bool* error){
    uint16_t angle;  
    uint8_t readRegisters[2] = {0};
    readRegisterQuickRead(readRegisters, 2, error);        
    return (readRegisters[1]<<8) | readRegisters[0];
}

uint16_t MagAlphaI2CGen7::readAngleRaw16(){
    bool error;
    return readAngleRaw(&error);
}

uint8_t MagAlphaI2CGen7::readAngleRaw8(){
    return readAngleRaw16()>>8;
}

uint8_t MagAlphaI2CGen7::readRegister(uint8_t address){
    bool error;
    return readRegister(address, &error);
}

uint8_t MagAlphaI2CGen7::writeRegister(uint8_t address, uint8_t value){
    uint8_t crc = _crcInitValue;
    _i2c->beginTransmission(_deviceAddress);
    crc = crc8(crc, (_deviceAddress<<1)+0);
    _i2c->write(byte(address));
    crc = crc8(crc, address);
    _i2c->write(byte(value));
    crc = crc8(crc, value);
    if(_crcCheckEnabled){
        _i2c->write(byte(crc));
    }
    _i2c->endTransmission();
    return value;
}

void MagAlphaI2CGen7::readRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister){
    bool error;
    readRegisterBurst(address, valueArray, numberOfRegister, &error);
}

void MagAlphaI2CGen7::writeRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister){
    uint8_t crc;
    _i2c->beginTransmission(_deviceAddress);
    _i2c->write(byte(address));
    for (uint8_t cnt = 0; cnt < numberOfRegister; cnt++){
        _i2c->write(byte(valueArray[cnt]));
        if (_crcCheckEnabled){
            crc = _crcInitValue;
            crc = crc8(crc, _deviceAddress<<1);
            crc = crc8(crc, (address + cnt));
            crc = crc8(crc, valueArray[cnt]);
            _i2c->write(byte(crc));
        }
    }
    _i2c->endTransmission();
}

uint16_t MagAlphaI2CGen7::detectSensorGeneration(){
    uint16_t chipId;
    chipId = readRegister(REG_PID);
    return (chipId>>4)&0xF;
}

uint16_t MagAlphaI2CGen7::getZero(){
    uint8_t readRegisters[2] = {0};
    readRegisterBurst(0, readRegisters, 2);
    return (readRegisters[1]<<8) | readRegisters[0];
}

void MagAlphaI2CGen7::setZero(uint16_t zero){
    uint8_t readRegisters[2] = {static_cast<uint8_t>(zero & 0xFF), static_cast<uint8_t>(zero >> 8)};
    writeRegisterBurst(0, readRegisters, 2);
}

uint16_t MagAlphaI2CGen7::setCurrentAngleAsZero(){
    uint16_t zero;
    setZero(0);
    delay(10);
    zero = readAngleRaw16();
    setZero(zero);
    return zero;
}

void MagAlphaI2CGen7::restoreAllRegisters(){
    uint8_t tempReg;
    tempReg = readRegister(REG_MTP);
    tempReg |= 0x10;
    writeRegister(REG_MTP, tempReg);
    //wait for MTP DONE Flag to be set to 1 (indicate MTP Store/Restore operation is finished)
    tempReg=0;
    while((tempReg & 0x80) == 0){
        tempReg = readRegister(REG_MTP);
    }
}

void MagAlphaI2CGen7::storeAllRegisters(){
    //Store Page 0 and Page 1
    storeRegisterBlock(3); 
}

void MagAlphaI2CGen7::storeRegisterBlock(uint8_t block){
    uint8_t tempReg = 0;
    tempReg = readRegister(REG_MTP);
    if(block <= 3){
        tempReg &= 0xFC;
        tempReg |= 0x20 | block;
        writeRegister(REG_MTP, tempReg);
    }
    //wait for MTP DONE Flag to be set to 1 (indicate MTP Store/Restore operation is finished)
    tempReg=0;
    while((tempReg & 0x80) == 0){
        tempReg = readRegister(REG_MTP);
    }
}

void MagAlphaI2CGen7::clearErrorFlags(){
    writeRegister(REG_STATUS, 0X07);
}

double MagAlphaI2CGen7::readSpeed(){
    uint16_t speed;
    uint8_t readRegisters[2] = {0};
    readRegisterBurst(REG_SPEED0, readRegisters, 2);
    speed = (readRegisters[1]<<8) | readRegisters[0];
    return twosComplement(speed, 16) * 3.57;
}

int16_t MagAlphaI2CGen7::readTurn(){
    uint16_t turn;
    uint8_t readRegisters[2] = {0};
    readRegisterBurst(REG_MTV0, readRegisters, 2);
    turn = (readRegisters[1]<<8) | readRegisters[0];
    return twosComplement(turn, 16);
}

void MagAlphaI2CGen7::writeTurn(int16_t turn){
    uint16_t multiturn = twosComplementInverse(turn, 16);
    uint8_t readRegisters[2] = {static_cast<uint8_t>(multiturn & 0xFF), static_cast<uint8_t>(multiturn >> 8)};
    writeRegisterBurst(REG_MTV0, readRegisters, 2);
}

double MagAlphaI2CGen7::readTemperature(){
    uint16_t temperature;
    uint8_t readRegisters[2] = {0};
    readRegisterBurst(REG_TEMP0, readRegisters, 2);
    temperature = (readRegisters[1]<<8) | readRegisters[0];
    return twosComplement(temperature, 16) * 0.125;
}

uint16_t MagAlphaI2CGen7::readAcquisitionCounter(){
    return readRegister(REG_READC);
}

uint8_t MagAlphaI2CGen7::readRegister(uint8_t address, bool *error){
    uint8_t crc = _crcInitValue;
    uint8_t readbackValue, crcReceived;
    uint8_t requestBytes;
    requestBytes=_crcCheckEnabled?2:1; //if crc is enabled, read 2 bytes from sensor
    _i2c->beginTransmission(_deviceAddress);
    crc = crc8(crc, (_deviceAddress<<1)+0);
    _i2c->write(byte(address));
    crc = crc8(crc, address);
    _i2c->endTransmission(false);
    _i2c->requestFrom(_deviceAddress, requestBytes, true);
    crc = crc8(crc, (_deviceAddress<<1)+1);
    readbackValue = _i2c->read();
    crc = crc8(crc, readbackValue);
    if(_crcCheckEnabled){
        crcReceived = _i2c->read();
        checkCrc8(crcReceived, crc, error);
    }
    return readbackValue;
}

uint16_t MagAlphaI2CGen7::readRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister, bool *error){
    uint8_t byteCount = 0;
    uint8_t crcReceived;
    uint8_t crc;
    *error=false;
    _i2c->beginTransmission(_deviceAddress);
    _i2c->write(byte(address));
    _i2c->endTransmission(false);
    if(_crcCheckEnabled){
        numberOfRegister*=2;
    }
    _i2c->requestFrom(_deviceAddress, numberOfRegister);
    while(_i2c->available()){
        valueArray[byteCount] = _i2c->read();
        if (_crcCheckEnabled){
            crc = _crcInitValue;
            crc = crc8(crc, _deviceAddress<<1);
            crc = crc8(crc, address + byteCount);
            crc = crc8(crc, (_deviceAddress<<1)+1);
            crc = crc8(crc, valueArray[byteCount]);
            crcReceived = _i2c->read();
            *error = (crc != crcReceived) ? true : *error;
        }
        byteCount++;
    }
    return byteCount;
}

uint16_t MagAlphaI2CGen7::readRegisterQuickRead(uint8_t valueArray[], uint16_t numberOfRegister, bool *error){
    uint8_t crc;
    uint8_t crcReceived;
    uint8_t byteCount = 0;
    uint8_t firstReg = 96;
    *error=false;
    if(_crcCheckEnabled){
        numberOfRegister*=2;
    }
    _i2c->requestFrom(_deviceAddress, numberOfRegister);
    while(_i2c->available()) {
        valueArray[byteCount] = _i2c->read();
        if (_crcCheckEnabled){
            crc = _crcInitValue;
            crc = crc8(crc, (_deviceAddress<<1)+1);
            crc = crc8(crc, (firstReg + byteCount));
            crc = crc8(crc, valueArray[byteCount]);
            crcReceived = _i2c->read();
            *error = (crc != crcReceived) ? true : *error;
        }
        byteCount++;
    }
    return byteCount;
}

void MagAlphaI2CGen7::setCrcCheckSetting(bool enable){
    bool error;
    uint8_t tempReg;
    tempReg = readRegister(REG_INTF, &error);
    if(enable){
        writeRegister(REG_INTF, tempReg|0x01);
        _crcCheckEnabled = true;
    }
    else{
        writeRegister(REG_INTF, tempReg&0xFE);
        _crcCheckEnabled = false;     
    }
}

uint8_t MagAlphaI2CGen7::crc8(uint8_t crc, uint8_t data){
    return poly_table[crc^data];
}

void  MagAlphaI2CGen7::checkCrc8(uint8_t &readData, uint8_t &computedCrc, bool *errorDetected){
    //compare the data + CRC received VS the expected data + CRC
    if (readData==computedCrc){
        *errorDetected=false;
    }
    else{
        *errorDetected=true;
    }
}

uint16_t MagAlphaI2CGen7::getNumberOfRegisters(){
    return 128;
}

void MagAlphaI2CGen7::getLockedRegisters(uint8_t unlockedRegisters[]){
    uint8_t registerArray[4] = {0};
    readRegisterBurst(REG_LOCK0, registerArray, 4);
    for(uint8_t i=0; i<4; i++){
        for(uint8_t j=0; j<8; j++){
            unlockedRegisters[i*8+j] = ((registerArray[i]>>j)&0x01) ? 0xFF : 0x00;
        }
    }
}

void MagAlphaI2CGen7::getPartNumber(char *partNumber){
    sprintf(partNumber, "MA900");
}

uint8_t MagAlphaI2CGen7::getSiliconId(){
    return readRegister(REG_PID);
}

uint8_t MagAlphaI2CGen7::getSiliconRevision(){
    uint8_t siliconRevision0, siliconRevision1, siliconRevision2, siliconRevision3;
    siliconRevision0 = readRegister(REG_ID0);
    siliconRevision1 = readRegister(REG_ID1);
    siliconRevision2 = readRegister(REG_ID2);
    siliconRevision3 = readRegister(REG_ID3);
    return siliconRevision0;
}
