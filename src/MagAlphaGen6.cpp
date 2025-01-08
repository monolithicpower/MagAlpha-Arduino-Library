/***************************************************
  Arduino library for the MPS MagAlpha magnetic angle sensor
  Supports MagAlpha 6th generation Sensors. 
  Support Part Number includes: MA600
  MagAlpha sensor detects the
  absolute angular position of a permanent magnet, typically a diametrically
  magnetized cylinder on the rotating shaft.
  ----> http://www.monolithicpower.com/Products/Position-Sensors/Products-Overview
  Written by Mathieu Kaelin for Monolithic Power Systems.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "MagAlphaGen6.h"

//MagAlpha Read/Write Register Command
#define READ_REG_COMMAND    (0b11010010 << 8)
#define WRITE_REG_COMMAND   (0b1110101001010100)

#define REG_REGMAPID 30
#define REG_PID 31
#define REG_LOCK0 156
#define REG_LOCK1 157
#define REG_LOCK2 158
#define REG_LOCK3 159

MagAlphaGen6::MagAlphaGen6(){
}

uint16_t MagAlphaGen6::readAngleRaw16(){
    uint16_t angle;
    digitalWrite(_spiChipSelectPin, LOW);
    angle = _spi->transfer16(0x0000); //Read 16-bit angle
    digitalWrite(_spiChipSelectPin, HIGH);
    return angle;
}

uint8_t MagAlphaGen6::readAngleRaw8(){
    uint8_t angle;
    digitalWrite(_spiChipSelectPin, LOW);
    angle = _spi->transfer(0x00); //Read 8-bit angle
    digitalWrite(_spiChipSelectPin, HIGH);
    return angle;
}

uint16_t MagAlphaGen6::readAngleRaw(bool* error){
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

uint8_t MagAlphaGen6::readRegister(uint8_t address){
    uint8_t readbackRegisterValue;
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(READ_REG_COMMAND | address);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns before register readout
    digitalWrite(_spiChipSelectPin, LOW);
    readbackRegisterValue = _spi->transfer16(0x0000) & 0x00FF;
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
    return readbackRegisterValue;
}

uint8_t MagAlphaGen6::writeRegister(uint8_t address, uint8_t value){
    uint8_t readbackRegisterValue;
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(WRITE_REG_COMMAND);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns before register readout
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16((address << 8) | value);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns before register readout
    digitalWrite(_spiChipSelectPin, LOW);
    readbackRegisterValue = (_spi->transfer16(0x0000) & 0x00FF);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
    return readbackRegisterValue;
}

void MagAlphaGen6::readRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister){
    uint8_t readbackRegisterValue;
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(READ_REG_COMMAND | address);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns before register readout
    for (uint8_t i=0;i<numberOfRegister-1;++i){
        digitalWrite(_spiChipSelectPin, LOW);
        valueArray[i] = _spi->transfer16(READ_REG_COMMAND | (address+i+1)) & 0x00FF;
        digitalWrite(_spiChipSelectPin, HIGH);
        delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns before register readout
    }
    digitalWrite(_spiChipSelectPin, LOW);
    valueArray[numberOfRegister-1] = _spi->transfer16(0x0000) & 0x00FF;
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
}

void MagAlphaGen6::writeRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister){
    for (uint8_t i=0;i<numberOfRegister;++i){
        digitalWrite(_spiChipSelectPin, LOW);
        _spi->transfer16(WRITE_REG_COMMAND);
        digitalWrite(_spiChipSelectPin, HIGH);
        delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
        digitalWrite(_spiChipSelectPin, LOW);
        _spi->transfer16(((address+i) << 8) | valueArray[i]);
        digitalWrite(_spiChipSelectPin, HIGH);
        delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
    }
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(0x0000);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
}

/*
uint16_t MagAlphaGen6::detectSensorGeneration(){
    uint16_t chipId;
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(0xD300);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
    digitalWrite(_spiChipSelectPin, LOW);
    chipId = _spi->transfer16(0x0000)&0xFF;
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
    return chipId;
}
*/

uint16_t MagAlphaGen6::getZero()
{
    return (readRegister(1)<<8) + readRegister(0);
}
void MagAlphaGen6::setZero(uint16_t zero){
    writeRegister(0, zero&0xFF);
    writeRegister(1, zero>>8);
}

uint16_t MagAlphaGen6::setCurrentAngleAsZero(){
    uint16_t zero;
    setZero(0);
    zero=readAngleRaw16();
    setZero(zero);
    return zero;
}

uint16_t MagAlphaGen6::getBct(){
    return readRegister(2);
}

void MagAlphaGen6::setBct(uint16_t bct){
    writeRegister(2, bct&0xFF);
}

void MagAlphaGen6::restoreAllRegisters(){
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(0b1110101001010110);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(240); //Wait for 240us
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(0x0000);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
}

void MagAlphaGen6::storeAllRegisters(){
    storeRegisterBlock(0);
    storeRegisterBlock(1);
}

void MagAlphaGen6::storeRegisterBlock(uint8_t block){
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(0b1110101001010101);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(0b1110101000000000 | (block & 0x1));
    digitalWrite(_spiChipSelectPin, HIGH);
    delay(600); //Wait for 600ms
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(0x0000);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
}

void MagAlphaGen6::clearErrorFlags(){
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(0b1101011100000000);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(0x0000);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
}

double MagAlphaGen6::readSpeed(){
    uint16_t angle;
    uint16_t multiturnOrSpeed;
    //The choice between multiturn and speed is done by settings MTSP 
    //for multiturn MTSP=0, for speed MTSP=1. This is a volatile settings
    //MTSP = 1, PRT = 0, PRTS = 0, APRT=0, FTA(1:0)=0, FTM=0
    writeRegister(28, 0x80);
    readAngleAndMultiturnOrSpeedRaw(&angle, &multiturnOrSpeed);
    return twosComplement(multiturnOrSpeed, 16) * 5.722;
}

int16_t MagAlphaGen6::readTurn(){
    uint16_t angle;
    uint16_t multiturnOrSpeed;
    //The choice between multiturn and speed is done by settings MTSP 
    //for multiturn MTSP=0, for speed MTSP=1. This is a volatile settings
    //MTSP = 0, PRT = 0, PRTS = 0, APRT=0, FTA(1:0)=0, FTM=0
    writeRegister(28, 0x00);
    readAngleAndMultiturnOrSpeedRaw(&angle, &multiturnOrSpeed);
    return twosComplement(multiturnOrSpeed, 16);
}

void MagAlphaGen6::readAngleAndMultiturnOrSpeedRaw(uint16_t *angle, uint16_t *multiturnOrSpeed){
    digitalWrite(_spiChipSelectPin, LOW);
    *angle = _spi->transfer16(0x0000); //Read 16-bit angle
    *multiturnOrSpeed = _spi->transfer16(0x0000); //Read 16-bit angle
    digitalWrite(_spiChipSelectPin, HIGH);
}

void MagAlphaGen6::writeTurn(int16_t turn){
    uint16_t mtoffset = twosComplementInverse(turn, 16);
    writeRegister(18, mtoffset&0xFF);
    writeRegister(19, mtoffset>>8);
}

uint16_t MagAlphaGen6::getNumberOfRegisters(){
    return 160;
}

void MagAlphaGen6::getLockedRegisters(uint8_t unlockedRegisters[]){
    uint8_t i, j;
    uint8_t registerArray[4] = {0};
    readRegisterBurst(REG_LOCK0, registerArray, 4);
    for(i=0; i<3; i++){
        for(j=0; j<8; j++){
            unlockedRegisters[i*8+j] = ((registerArray[i]>>j)&0x01) ? 0xFF : 0x00;
        }
    }
    i = 3;
    for(j=0; j<2; j++){
        unlockedRegisters[i*8+j] = ((registerArray[i]>>j)&0x01) ? 0xFF : 0x00;
    }
    for (j=32; j<64; j++){
        unlockedRegisters[j] = ((registerArray[3]>>2)&0x01) ? 0xFF : 0x00;
    }
}

void MagAlphaGen6::getPartNumber(char *partNumber){
    sprintf(partNumber, "MA600");
}

uint8_t MagAlphaGen6::getSiliconId(){
    return readRegister(REG_PID) & 0x3F;
}

uint8_t MagAlphaGen6::getSiliconRevision(){
    return readRegister(REG_PID) & 0x3F;
}

uint8_t MagAlphaGen6::getRegisterMapRevision(){
    return readRegister(REG_REGMAPID) & 0x7F;
}







// void MagAlphaGen6::setSpiClockFrequency(uint32_t speedMaximum){
//     _speedMaximum = speedMaximum;
//     SPI.beginTransaction(SPISettings(_speedMaximum, MSBFIRST, _spiMode));
// }

// void MagAlphaGen6::setSpiDataMode(uint8_t spiMode){
//     _spiMode = spiMode;
//     SPI.beginTransaction(SPISettings(_speedMaximum, MSBFIRST, _spiMode));
// }

// void MagAlphaGen6::setSpiChipSelectPin(uint8_t spiChipSelectPin){
//     _spiChipSelectPin = spiChipSelectPin;
//     pinMode(_spiChipSelectPin, OUTPUT);
//     digitalWrite(_spiChipSelectPin, HIGH);
// }

// double MagAlphaGen6::convertRawAngleToDegree(uint8_t rawAngleDataBitLength, uint16_t rawAngle){
//     double angleInDegree;
//     angleInDegree = (rawAngle*360.0)/((double)pow(2, rawAngleDataBitLength));
//     return angleInDegree;
// }


// MagAlphaSSIGen6::MagAlphaSSIGen6(){
// }
// void  MagAlphaSSIGen6::begin(){
//     _speedMaximum = 1000000;
//     SPI.begin();
//     SPI.beginTransaction(SPISettings(_speedMaximum, MSBFIRST, SSI_MODE));
// }

// void  MagAlphaSSIGen6::begin(int32_t ssiSsckFrequency){
//     _speedMaximum = ssiSsckFrequency;
//     SPI.begin();
//     SPI.beginTransaction(SPISettings(_speedMaximum, MSBFIRST, SSI_MODE));
// }

// void MagAlphaSSIGen6::end(){
//     SPI.end();
// }

// double MagAlphaSSIGen6::readAngle(){
//     uint16_t angle;
//     double angleInDegree;
//     angle = readAngleRaw();
//     angleInDegree = (angle*360.0)/65536.0;
//     return angleInDegree;
// }

// uint16_t MagAlphaSSIGen6::readAngleRaw(){
//     uint16_t angle;
//     uint8_t angle0;
//     uint8_t angle1;
//     uint8_t angle2;

//     angle0 = SPI.transfer(0x00);
//     angle1 = SPI.transfer(0x00);
//     angle2 = SPI.transfer(0x00);

//     angle = ((angle0 & 0x7F) << 9) | (angle1 << 1) | ((angle2 & 0x80) >> 7);
//     return angle;
// }

// uint16_t MagAlphaSSIGen6::readAngleRaw(bool* error){
//     uint16_t angle;
//     uint8_t parity;
//     uint8_t highStateCount = 0;
//     uint8_t angle0;
//     uint8_t angle1;
//     uint8_t angle2;

//     angle0 = SPI.transfer(0x00);
//     angle1 = SPI.transfer(0x00);
//     angle2 = SPI.transfer(0x00);

//     angle = ((angle0 & 0x7F) << 9) | (angle1 << 1) | ((angle2 & 0x80) >> 7);
//     parity = ((angle2 & 0x40) >> 6);
//     //Count the number of 1 in the angle binary value
//     for (int i=0;i<16;++i){
//         if ((angle & (1 << i)) != 0){
//             highStateCount++;
//         }
//     }
//     //check if parity bit is correct
//     if ((highStateCount % 2) == 0){
//         if (parity == 0){
//             *error = false;
//         }
//         else{
//             *error = true;
//         }
//     }
//     else{
//         if (parity == 1){
//             *error = false;
//         }
//         else{
//             *error = true;
//         }
//     }
//     return angle;
// }

// void MagAlphaSSIGen6::setSsiClockFrequency(uint32_t speedMaximum){
//     _speedMaximum = speedMaximum;
//     SPI.beginTransaction(SPISettings(_speedMaximum, MSBFIRST, SSI_MODE));
// }

// double MagAlphaSSIGen6::convertRawAngleToDegree(uint8_t rawAngleDataBitLength, uint16_t rawAngle){
//     double angleInDegree;
//     angleInDegree = (rawAngle*360.0)/((double)pow(2, rawAngleDataBitLength));
//     return angleInDegree;
// }
