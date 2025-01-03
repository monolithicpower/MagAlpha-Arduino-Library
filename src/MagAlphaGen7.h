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

#ifndef MAGALPHAGEN7_H
#define MAGALPHAGEN7_H

#if (ARDUINO >= 100)
     #include "Arduino.h"
#else
     #include "WProgram.h"
#endif
#include "MagAlphaBase.h"

class MagAlphaGen7: public MagAlphaSPI {
public:
    MagAlphaGen7();

    uint16_t readAngleRaw(bool *error) override;
    uint16_t readAngleRaw16() override;
    uint8_t readAngleRaw8() override;
    uint8_t readRegister(uint8_t address) override;
    uint8_t writeRegister(uint8_t address, uint8_t value) override;
    void readRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister) override;
    void writeRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister) override;

    //Detect the sensor generation
    //uint16_t detectSensorGeneration() override;

    //Set Key Parameters
    uint16_t getZero() override;
    void setZero(uint16_t zero) override;
    uint16_t setCurrentAngleAsZero() override;

    //Memory management
    void restoreAllRegisters() override;
    void storeAllRegisters() override;
    /**
     * @brief Store register block
     * @param block Register block to store.
     *          block=0, Do not store anything (Do not use) 
     *          block=1, Store Page 0
     *          block=2, Store Page 1
     *          block=3, Store Page 0 and Page 1
     */
    void storeRegisterBlock(uint8_t block) override;

    //Advanced features
    void clearErrorFlags() override;
    double readSpeed() override;
    int16_t readTurn() override;
    void writeTurn(int16_t turn) override;
    double readTemperature() override;
    uint16_t readAcquisitionCounter() override;

    //Part Information
    uint16_t getNumberOfRegisters() override;
    void getLockedRegisters(uint8_t unlockedRegisters[]) override;
    void getPartNumber(char *partNumber) override;
    uint8_t getSiliconId() override;
    uint8_t getSiliconRevision() override;

    uint16_t readAngleRaw16(bool *error, bool *inversion, bool isShortRead=false);
    uint16_t readAngleCounter(uint16_t *angle, bool *error, bool *inversion);
    uint16_t readAngleSpeed(uint16_t *angle, bool *error, bool *inversion);
    uint16_t readAngleMultiturn(uint16_t *angle, bool *error, bool *inversion);
    uint16_t readAngleTemperature(uint16_t *angle, bool *error, bool *inversion);
    uint8_t readRegister(uint8_t address, bool *error, bool *inversion);
    uint16_t readRegisterBurst(uint8_t address, uint8_t readbackValueArray[], uint16_t numberOfRegister, bool *error, bool *inversion);
    uint8_t writeRegister(uint8_t address, uint8_t value, bool *error, bool *inversion, bool *wrongHandshaking);
    void writeRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister, bool *error, bool *inversion, bool *wrongHandshaking);
    void setCrcCheckSetting(bool enable);
    uint16_t appendCrc4(uint16_t data);
    void checkCrc4(uint16_t readData, uint16_t *computedCrc, bool *errorDetected, bool *inversionDetected);
    
private:
    bool _crcCheckEnabled;
    uint8_t _crcInitValue;
};

class MagAlphaSSIGen7: public MagAlphaSSI {};

class MagAlphaI2CGen7: public MagAlphaI2C {
public:
    MagAlphaI2CGen7();

    uint16_t readAngleRaw(bool *error) override;
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
    /**
     * @brief Store register block
     * @param block Register block to store.
     *          block=0, Do not store anything (Do not use) 
     *          block=1, Store Page 0
     *          block=2, Store Page 1
     *          block=3, Store Page 0 and Page 1
     */
    void storeRegisterBlock(uint8_t block) override;

    //Advanced features
    void clearErrorFlags() override;
    double readSpeed() override;
    int16_t readTurn() override;
    void writeTurn(int16_t turn) override;
    double readTemperature() override;
    uint16_t readAcquisitionCounter() override;

    uint16_t getNumberOfRegisters() override;
    void getLockedRegisters(uint8_t unlockedRegisters[]) override;
    void getPartNumber(char *partNumber) override;
    uint8_t getSiliconId() override;
    uint8_t getSiliconRevision() override;

    uint8_t readRegister(uint8_t address, bool *error);
    uint16_t readRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister, bool *error);
    uint16_t readRegisterQuickRead(uint8_t valueArray[], uint16_t numberOfRegister, bool *error);
    void setCrcCheckSetting(bool enable);

    uint8_t crc8(uint8_t crc, uint8_t data);
    void checkCrc8(uint8_t &readData, uint8_t &computedCrc, bool *errorDetected);

private:
    bool _crcCheckEnabled;
    uint8_t _crcInitValue;
};

#endif //MAGALPHAGEN7_H