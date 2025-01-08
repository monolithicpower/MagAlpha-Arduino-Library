/***************************************************
  Arduino library for the MPS MagAlpha magnetic angle sensor
  ----> http://www.monolithicpower.com/Products/Position-Sensors/Products-Overview
  Written by Mathieu Kaelin for Monolithic Power Systems.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#ifndef MAGALPHABASE_H
#define MAGALPHABASE_H

#if (ARDUINO >= 100)
     #include "Arduino.h"
#else
     #include "WProgram.h"
#endif
#include <SPI.h>
#include <Wire.h>
#include <stdint.h>
#include "MagAlphaPartProperties.h"


class MagAlphaBase {
public:
    MagAlphaBase();

    double readAngle();
    uint16_t readAngleRaw();

    virtual uint16_t readAngleRaw(bool* error){return 0;}
    virtual uint16_t readAngleRaw16(){return 0;}
    virtual uint8_t readAngleRaw8(){return 0;}
    virtual uint8_t readRegister(uint8_t address){return 0;}
    virtual uint8_t writeRegister(uint8_t address, uint8_t value){return 0;}
    virtual void readRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister){}
    virtual void writeRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister){}

    //Detect the sensor generation
    virtual uint16_t detectSensorGeneration(){return 0;}

    //Set Key Parameters
    virtual uint16_t getZero(){return 0;}
    virtual void setZero(uint16_t zero){}
    virtual uint16_t setCurrentAngleAsZero(){return 0;}
    virtual uint16_t getBct(){return 0;}
    virtual void setBct(uint16_t bct){}

    //Memory management
    virtual void restoreAllRegisters(){}
    virtual void storeAllRegisters(){}
    virtual void storeRegisterBlock(uint8_t block){}
    virtual void storeRegister(uint8_t address){}

    //Advanced features
    virtual void clearErrorFlags(){}
    virtual double readSpeed(){return 0;}
    virtual int16_t readTurn(){return 0;}
    virtual void writeTurn(int16_t turn){}
    virtual double readTemperature(){return 0.0;}
    virtual uint16_t readAcquisitionCounter(){return 0;}

    //Part Information
    virtual uint16_t getNumberOfRegisters(){return 0;}
    virtual void getLockedRegisters(uint8_t unlockedRegisters[]){}
    virtual void getPartNumber(char *partNumber);
    virtual uint8_t getSiliconId(){return 0;}
    virtual uint8_t getSiliconRevision(){return 0;}
    virtual uint8_t getRegisterMapRevision(){return 0;}

    //Utility functions
    double convertRawAngleToDegree(uint8_t rawAngleDataBitLength, uint16_t rawAngle);
    int16_t twosComplement(uint16_t value, uint8_t numberOfBits);
    uint16_t twosComplementInverse(int16_t value, uint8_t numberOfBits);
protected:
    // bool _featureSuppored;
};

/*====================================================================================*/
/*============================== MagAlphaSPI =========================================*/
/*====================================================================================*/
class MagAlphaSPI : public MagAlphaBase {
public:
    MagAlphaSPI();
    void begin(int spiChipSelectPin, SPIClass *spi = &SPI);
    void begin(int32_t spiSclkFrequency, MagAlphaSPIMode spiMode, uint8_t spiChipSelectPin, SPIClass *spi = &SPI);
    void end();
    void setSpiClockFrequency(uint32_t clockFrequency);
    void setSpiDataMode(MagAlphaSPIMode spiMode);
    void setSpiChipSelectPin(uint8_t spiChipSelectPin);
    virtual uint16_t readAngleRaw16Quick();
protected:
    uint8_t _spiChipSelectPin;
    uint32_t _clockFrequency;
    uint8_t _spiMode;
    SPIClass *_spi;
};

/*====================================================================================*/
/*============================== MagAlphaSSI =========================================*/
/*====================================================================================*/
class MagAlphaSSI : public MagAlphaBase {
public:
    MagAlphaSSI();
    void begin();
    //void begin(int32_t ssiClkFrequency);
    void begin(SPIClass *ssi = &SPI);
    void begin(int32_t ssiSsckFrequency, SPIClass *ssi = &SPI);
    void begin(int32_t ssiSsckFrequency, MagAlphaSSIMode ssiMode, SPIClass *ssi = &SPI);
    void end();
    void setSsiClockFrequency(uint32_t clockFrequency);
    void setSSiMode(MagAlphaSSIMode ssiMode);
    double readAngle();
    uint16_t readAngleRaw();
    virtual uint16_t readAngleRaw(bool* error);
    virtual uint16_t readAngleRaw16();
    virtual uint8_t readAngleRaw8();
protected:
    uint32_t _clockFrequency;
    uint8_t _ssiMode;
    SPIClass *_ssi;
};

/*====================================================================================*/
/*============================== MagAlphaI2C =========================================*/
/*====================================================================================*/
class MagAlphaI2C : public MagAlphaBase {
public:
    MagAlphaI2C();
    void begin(uint8_t deviceAddress, uint32_t clockFrequency=400000, TwoWire *i2c = &Wire);
    void end();
    void setClockFrequency(uint32_t clockFrequency);
    void setDeviceAddress(uint8_t deviceAddress);
    uint8_t findDeviceAddress();
    uint8_t findAllDeviceAddresses(uint8_t detectedDeviceAddresses[], uint8_t arraySize);
protected:
    uint8_t _deviceAddress;
    uint32_t _clockFrequency;
    TwoWire *_i2c;
};

#endif //MAGALPHABASE_H
