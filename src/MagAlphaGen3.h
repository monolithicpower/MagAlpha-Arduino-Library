/***************************************************
  Arduino library for the MPS MagAlpha magnetic angle sensor
  Supports MagAlpha 3rd generation Sensors. 
  Support Part Number includes: MA102, MA302, MA310, MA330, 
  MA702, MA704, MA710, MA730, MA731, MA732, MA800, MA820, MA850, MAQ430, 
  MAQ470, MAQ473, MAP790-xxxx, MAP791-xxxx
  MagAlpha sensor detects the
  absolute angular position of a permanent magnet, typically a diametrically
  magnetized cylinder on the rotating shaft.
  ----> http://www.monolithicpower.com/Products/Position-Sensors/Products-Overview
  Written by Mathieu Kaelin for Monolithic Power Systems.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#ifndef MAGALPHAGEN3_H
#define MAGALPHAGEN3_H

#if (ARDUINO >= 100)
     #include "Arduino.h"
#else
     #include "WProgram.h"
#endif
#include "MagAlphaBase.h"

class MagAlphaGen3: public MagAlphaSPI {
public:
    MagAlphaGen3();
    uint16_t readAngleRaw(bool* error) override;
    uint16_t readAngleRaw16() override;
    uint8_t readAngleRaw8() override;
    uint8_t readRegister(uint8_t address) override;
    uint8_t writeRegister(uint8_t address, uint8_t value) override;
    void readRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister) override;
    void writeRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister) override;
    
    //Detect the sensor generation
    uint16_t detectSensorGeneration() override;

    //Set Key Parameters
    uint16_t getZero() override;
    void setZero(uint16_t zero) override;
    uint16_t setCurrentAngleAsZero() override;
    uint16_t getBct() override;
    void setBct(uint16_t bct) override;

    //Part Information
    uint16_t getNumberOfRegisters() override;
    void getLockedRegisters(uint8_t unlockedRegisters[]) override;
    void getPartNumber(char *partNumber) override;
    uint8_t getSiliconId() override;
    uint8_t getSiliconRevision() override;
    uint8_t getRegisterMapRevision() override;

class MagAlphaSSIGen3: public MagAlphaSSI {};

#endif //MAGALPHAGEN3_H
