/***************************************************
  Arduino library for the MPS MagAlpha magnetic angle sensor
  Supports MagAlpha 4th generation Sensors. 
  Support Part Number includes: MA780, MA782, MA734
  MagAlpha sensor detects the
  absolute angular position of a permanent magnet, typically a diametrically
  magnetized cylinder on the rotating shaft.
  ----> http://www.monolithicpower.com/Products/Position-Sensors/Products-Overview
  Written by Mathieu Kaelin for Monolithic Power Systems.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "MagAlphaGen4.h"

//MagAlpha Read/Write Register Command
#define READ_REG_COMMAND                (0b010 << 13)
#define WRITE_REG_COMMAND               (0b100 << 13)
#define STORE_SINGLE_REGISTER_COMMAND   (0b111 << 13)
#define STORE_ALL_REGISTERS_COMMAND     (0b110 << 13)
#define RESTORE_ALL_REGISTERS_COMMAND   (0b101 << 13)
#define CLEAR_ERROR_FLAGS_COMMAND       (0b001 << 13)

#define REG_REGMAPID 23
#define REG_SID 24
#define REG_PID 25
#define REG_LOCK0 28
#define REG_LOCK1 29
#define REG_LOCK2 30
#define REG_LOCK3 31

MagAlphaGen4::MagAlphaGen4(){
}

uint16_t MagAlphaGen4::readAngleRaw16(){
    uint16_t angle;
    digitalWrite(_spiChipSelectPin, LOW);
    angle = _spi->transfer16(0x0000); //Read 16-bit angle
    digitalWrite(_spiChipSelectPin, HIGH);
    return angle;
}

uint8_t MagAlphaGen4::readAngleRaw8(){
    uint8_t angle;
    digitalWrite(_spiChipSelectPin, LOW);
    angle = _spi->transfer(0x00);     //Read 8-bit angle
    digitalWrite(_spiChipSelectPin, HIGH);
    return angle;
}

uint16_t MagAlphaGen4::readAngleRaw(bool* error){
    uint16_t angle;
    uint8_t parity;
    uint8_t highStateCount = 0;
    digitalWrite(_spiChipSelectPin, LOW);
    angle = _spi->transfer16(0x0000);
    parity = _spi->transfer(0x00);
    digitalWrite(_spiChipSelectPin, HIGH);
    parity = ((parity & 0x80) >> 7);
    //Count the number of 1 in the angle binary value
    for (int i=0;i<16;++i){
        if ((angle & (1 << i)) != 0){
            highStateCount++;
        }
    }
    //check if parity bit is correct
    if ((highStateCount % 2) == 0){
        if (parity == 0){
            *error = false;
        }
        else{
            *error = true;
        }
    }
    else{
        if (parity == 1){
            *error = false;
        }
        else{
            *error = true;
        }
    }
    return angle;
}

uint8_t MagAlphaGen4::readRegister(uint8_t address){
    uint8_t readbackRegisterValue;
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(READ_REG_COMMAND | ((address & 0x1F) << 8) | 0x00);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns before register readout
    digitalWrite(_spiChipSelectPin, LOW);
    readbackRegisterValue = (_spi->transfer16(0x0000) & 0x00FF);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
    return readbackRegisterValue;
}

uint8_t MagAlphaGen4::writeRegister(uint8_t address, uint8_t value){
    uint8_t readbackRegisterValue;
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(WRITE_REG_COMMAND | ((address & 0x1F) << 8) | value);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns before register readout
    digitalWrite(_spiChipSelectPin, LOW);
    readbackRegisterValue = (_spi->transfer16(0x0000) & 0x00FF);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
    return readbackRegisterValue;
}

void MagAlphaGen4::readRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister){
    uint8_t readbackRegisterValue;
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(READ_REG_COMMAND | (((address) & 0x1F) << 8) | 0x00);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns before register readout
    for (uint8_t i=0;i<numberOfRegister-1;++i){
        digitalWrite(_spiChipSelectPin, LOW);
        valueArray[i] = (_spi->transfer16(READ_REG_COMMAND | (((address+i+1) & 0x1F) << 8) | 0x00) & 0x00FF);
        digitalWrite(_spiChipSelectPin, HIGH);
        delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns before register readout
    }
    digitalWrite(_spiChipSelectPin, LOW);
    valueArray[numberOfRegister-1] = (_spi->transfer16(0x0000) & 0x00FF);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
}

void MagAlphaGen4::writeRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister){
    for (uint8_t i=0;i<numberOfRegister;++i){
        digitalWrite(_spiChipSelectPin, LOW);
        _spi->transfer16(WRITE_REG_COMMAND | (((address+i) & 0x1F) << 8) | valueArray[i]);
        digitalWrite(_spiChipSelectPin, HIGH);
        delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
    }
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(0x0000);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
}

uint16_t MagAlphaGen4::detectSensorGeneration(){
    uint16_t chipId;
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(0x6000);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns before register readout
    digitalWrite(_spiChipSelectPin, LOW);
    chipId = _spi->transfer16(0x0000) & 0x00FF;
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
    return chipId;
}

uint16_t MagAlphaGen4::getZero()
{
    return (readRegister(1)<<8) + readRegister(0);
}

void MagAlphaGen4::setZero(uint16_t zero){
    writeRegister(0, zero&0xFF);
    writeRegister(1, zero>>8);
}

uint16_t MagAlphaGen4::setCurrentAngleAsZero(){
    uint16_t zero;
    setZero(0);
    zero=readAngleRaw16();
    setZero(zero);
    return zero;
}

uint16_t MagAlphaGen4::getBct(){
    return readRegister(2);
}

void MagAlphaGen4::setBct(uint16_t bct){
    writeRegister(2, bct&0xFF);
}

void MagAlphaGen4::restoreAllRegisters(){
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(RESTORE_ALL_REGISTERS_COMMAND);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(240); //Wait for 240us
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(0x0000);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
}

void MagAlphaGen4::storeAllRegisters(){
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(STORE_ALL_REGISTERS_COMMAND);
    digitalWrite(_spiChipSelectPin, HIGH);
    delay(704); //Wait for 704ms
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(0x0000);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
}

void MagAlphaGen4::storeRegister(uint8_t address){
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(STORE_SINGLE_REGISTER_COMMAND | ((address & 0x1F) << 8) | 0x00);
    digitalWrite(_spiChipSelectPin, HIGH);
    delay(23); //Wait for 23ms
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(0x0000);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
}

void MagAlphaGen4::clearErrorFlags(){
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(CLEAR_ERROR_FLAGS_COMMAND);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tClearFault of 40ns before register readout
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(0x0000);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
}

uint16_t MagAlphaGen4::getNumberOfRegisters(){
    return 32;
}

void MagAlphaGen4::getLockedRegisters(uint8_t unlockedRegisters[]){
    uint8_t registerArray[4] = {0};
    readRegisterBurst(REG_LOCK0, registerArray, 4);
    for(uint8_t i=0; i<4; i++){
        for(uint8_t j=0; j<8; j++){
            unlockedRegisters[i*8+j] = ((registerArray[i]>>j)&0x01) ? 0xFF : 0x00;
        }
    }
}

void MagAlphaGen4::getPartNumber(char *partNumber){
    uint8_t partId = readRegister(REG_PID);
    switch(partId) {
        case 1:
            sprintf(partNumber, "MA780");
            break;
        case 2:
            sprintf(partNumber, "MA781");
            break;
        case 3:
            sprintf(partNumber, "MA782");
            break;
        case 248:
            sprintf(partNumber, "MA736");
            break;
        case 249:
            sprintf(partNumber, "MA734");
            break;
        case 250:
            sprintf(partNumber, "MA734");
            break;
        default:
            sprintf(partNumber, "Gen4 [%u] Unknown Part Number", partId);
    }
}

uint8_t MagAlphaGen4::getSiliconId(){
    return (readRegister(REG_SID)  & 0xF0) >> 4;
}

uint8_t MagAlphaGen4::getSiliconRevision(){
    return readRegister(REG_SID) & 0x0F;
}

uint8_t MagAlphaGen4::getRegisterMapRevision(){
    return readRegister(REG_REGMAPID);
}
