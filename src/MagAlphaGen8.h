/***************************************************
  Arduino library for the MPS MagAlpha magnetic angle sensor
  Supports MagDiff 8th generation Sensors. 
  Support Part Number includes: MA980, MA981
  MagDiff sensor detects the
  absolute angular position of a permanent magnet, typically a diametrically
  magnetized cylinder on the rotating shaft.
  ----> http://www.monolithicpower.com/Products/Position-Sensors/Products-Overview
  Written by Mathieu Kaelin for Monolithic Power Systems.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#ifndef MAGALPHAGEN8_H
#define MAGALPHAGEN8_H

#if (ARDUINO >= 100)
     #include "Arduino.h"
#else
     #include "WProgram.h"
#endif
#include "MagAlphaBase.h"

class MagAlphaGen8: public MagAlphaSPI {
public:
    MagAlphaGen8();
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

    //Memory management
    void restoreAllRegisters() override;
    void storeAllRegisters() override;
    void storeRegister(uint8_t address) override;

    //Advanced features
    void clearErrorFlags() override;

    //Part Information
    uint16_t getNumberOfRegisters() override;
    void getLockedRegisters(uint8_t unlockedRegisters[]) override;
    void getPartNumber(char *partNumber) override;
    uint8_t getSiliconId() override;
    uint8_t getSiliconRevision() override;
    uint8_t getRegisterMapRevision() override;

};

class MagAlphaSSIGen8: public MagAlphaSSI {};

#endif //MAGALPHAGEN8_H
