/***************************************************
  Arduino library for the MPS MagAlpha magnetic angle sensor
  ----> http://www.monolithicpower.com/Products/Position-Sensors/Products-Overview
  Written by Mathieu Kaelin for Monolithic Power Systems.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "MagAlphaBase.h"

MagAlphaBase::MagAlphaBase(){
}

double MagAlphaBase::readAngle(){
  return readAngleRaw16()*(360.0/65536.0);
}

uint16_t MagAlphaBase::readAngleRaw(){
    return readAngleRaw16();
}

void MagAlphaBase::getPartNumber(char *partNumber){
    sprintf(partNumber, "Unknown Part Number");
}

double MagAlphaBase::convertRawAngleToDegree(uint8_t rawAngleDataBitLength, uint16_t rawAngle){
    double angleInDegree;
    angleInDegree = (rawAngle*360.0)/((double)pow(2, rawAngleDataBitLength));
    return angleInDegree;
}

int16_t MagAlphaBase::twosComplement(uint16_t value, uint8_t numberOfBits){
    int16_t signedValue = static_cast<int16_t>(value);
    if ((value & (1 << (numberOfBits - 1))) != 0){
        signedValue = value - (1 << numberOfBits);
    }
    return signedValue;
}

uint16_t MagAlphaBase::twosComplementInverse(int16_t value, uint8_t numberOfBits){
    uint16_t unsignedValue = static_cast<uint16_t>(value);
    if (value < 0){
        unsignedValue = value + (1 << numberOfBits);
    }
    return unsignedValue;
}

/*====================================================================================*/
/*============================== MagAlphaSPI =========================================*/
/*====================================================================================*/
MagAlphaSPI::MagAlphaSPI(){
}

void MagAlphaSPI::begin(int spiChipSelectPin, SPIClass *spi){
    begin(10000000, (MagAlphaSPIMode)SPI_MODE3, spiChipSelectPin, spi);
}

void MagAlphaSPI::begin(int32_t spiSclkFrequency, MagAlphaSPIMode spiMode, uint8_t spiChipSelectPin, SPIClass *spi){
    _spi = spi;
    _clockFrequency = spiSclkFrequency;
    _spiMode = (uint8_t)spiMode;
    setSpiChipSelectPin(spiChipSelectPin);
    _spi->begin();
    _spi->beginTransaction(SPISettings(_clockFrequency, MSBFIRST, _spiMode));
}

void MagAlphaSPI::end(){
    _spi->endTransaction();
    _spi->end();
}

void MagAlphaSPI::setSpiClockFrequency(uint32_t clockFrequency){
    _clockFrequency = clockFrequency;
    _spi->endTransaction();
    _spi->beginTransaction(SPISettings(_clockFrequency, MSBFIRST, _spiMode));
}

void MagAlphaSPI::setSpiDataMode(MagAlphaSPIMode spiMode){
    _spiMode = (uint8_t)spiMode;
    _spi->endTransaction();
    _spi->beginTransaction(SPISettings(_clockFrequency, MSBFIRST, _spiMode));
}

void MagAlphaSPI::setSpiChipSelectPin(uint8_t spiChipSelectPin){
    _spiChipSelectPin = spiChipSelectPin;
    pinMode(_spiChipSelectPin, OUTPUT);
    digitalWrite(_spiChipSelectPin, HIGH);
}

uint16_t MagAlphaSPI::readAngleRaw16Quick(){
    uint16_t angle;
    digitalWrite(_spiChipSelectPin, LOW);
    // angle = _spi->transfer16(0x0000); //Read 16-bit angle
    spi0_hw->dr = 0x0000;
    angle = spi0_hw->dr;
    digitalWrite(_spiChipSelectPin, HIGH);
    return angle;
}

/*====================================================================================*/
/*============================== MagAlphaSSI =========================================*/
/*====================================================================================*/
MagAlphaSSI::MagAlphaSSI(){
}

void MagAlphaSSI::begin(SPIClass *ssi){
    begin(1000000, MagAlphaSSIMode::MODE_A, ssi);
}

void MagAlphaSSI::begin(int32_t ssiSsckFrequency, SPIClass *ssi){
    begin(ssiSsckFrequency, MagAlphaSSIMode::MODE_A, ssi);
}

void MagAlphaSSI::begin(int32_t ssiSsckFrequency, MagAlphaSSIMode ssiMode, SPIClass *ssi){
    _ssi = ssi;
    _clockFrequency = ssiSsckFrequency;
    _ssiMode = (uint8_t)ssiMode;
    _ssi->begin();
    _ssi->beginTransaction(SPISettings(_clockFrequency, MSBFIRST, _ssiMode));
}

void MagAlphaSSI::end(){
    _ssi->endTransaction();
    _ssi->end();
}

void MagAlphaSSI::setSsiClockFrequency(uint32_t clockFrequency){
    _clockFrequency = clockFrequency;
    _ssi->endTransaction();
    _ssi->beginTransaction(SPISettings(_clockFrequency, MSBFIRST, _ssiMode));
}

void MagAlphaSSI::setSSiMode(MagAlphaSSIMode ssiMode){
    _ssiMode = (uint8_t)ssiMode;
    _ssi->endTransaction();
    _ssi->beginTransaction(SPISettings(_clockFrequency, MSBFIRST, _ssiMode));
}

double MagAlphaSSI::readAngle(){
    uint16_t angle;
    double angleInDegree;
    angle = readAngleRaw16();
    angleInDegree = (angle*360.0)/65536.0;
    return angleInDegree;
}

uint16_t MagAlphaSSI::readAngleRaw(){
    return readAngleRaw16();
}

uint16_t MagAlphaSSI::readAngleRaw(bool* error){
    uint16_t data1;
    uint8_t data2;
    uint8_t highStateCount = 0;
    data1 = _ssi->transfer16(0);
    data2 = _ssi->transfer(0);
    data1 = (data1 << 1);
    data1 = data1 + (data2 >> 7);
    data2 = ((data2 & 0x40) >> 6);
    //Count the number of 1 in the angle binary value
    for (int i=0;i<16;++i){
        if (data1 & (1 << i)){
            highStateCount++;
        }
    }
    //check if parity bit is correct
    if ((highStateCount % 2) == 0){
        if (data2 == 0){
            *error = false;
        }
        else{
            *error = true;
        }
    }
    else{
        if (data2 == 1){
            *error = false;
        }
        else{
            *error = true;
        }
    }
    return data1;
}

uint16_t MagAlphaSSI::readAngleRaw16(){
    uint16_t data1;
    uint8_t data2;
    data1 = _ssi->transfer16(0);
    data2 = _ssi->transfer(0);
    data1 = (data1 << 1);
    return data1 + (data2 >> 7);
}

uint8_t MagAlphaSSI::readAngleRaw8(){
    uint16_t data;
    data = _ssi->transfer16(0);
    return (data & 0x7F80) >> 7;
}


/*====================================================================================*/
/*============================== MagAlphaI2C =========================================*/
/*====================================================================================*/
MagAlphaI2C::MagAlphaI2C(){
}

void MagAlphaI2C::begin(uint8_t deviceAddress, uint32_t clockFrequency, TwoWire *i2c){
    _i2c = i2c;
    _deviceAddress = deviceAddress;
    _i2c->begin();
    setClockFrequency(clockFrequency);
}

void MagAlphaI2C::end(){
    _i2c->end();
}

void MagAlphaI2C::setClockFrequency(uint32_t clockFrequency){
    _clockFrequency=clockFrequency;
    _i2c->setClock(_clockFrequency);
    //100000 = Standard-mode (Sm)
    //400000 = Fast-mode (Fm)
    //1000000 = Fast-mode Plus (Fm)
    //3400000 = High-Speed mode (Hs-mode)
    //5000000 = Ulta Fast-mode (UFm) !!! Unidirectional Bus ONLY
    //10000 = low speed mode => not supported by the MKRZERO apparently
}

void MagAlphaI2C::setDeviceAddress(uint8_t deviceAddress){
    _deviceAddress = deviceAddress;
}

uint8_t MagAlphaI2C::findDeviceAddress(){
    for(int i=0; i<128; i++){
        _i2c->beginTransmission(i);
        if (_i2c->endTransmission() == 0){
            setDeviceAddress(i);
            return i;
        }
    }
    return 255;
}

uint8_t MagAlphaI2C::findAllDeviceAddresses(uint8_t detectedDeviceAddresses[], uint8_t arraySize){
    uint8_t numberOfDeviceDetected = 0;
    for(int i=0; i<128; i++){
        _i2c->beginTransmission(i);
        if (_i2c->endTransmission() == 0){
            if (numberOfDeviceDetected < arraySize){
                detectedDeviceAddresses[numberOfDeviceDetected] = i;
            }
            numberOfDeviceDetected++;
        }
    }
    if (numberOfDeviceDetected == 1){
        setDeviceAddress(detectedDeviceAddresses[0]);
    }
    return numberOfDeviceDetected;
}
