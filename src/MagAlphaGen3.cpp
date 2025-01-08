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

#include "MagAlphaGen3.h"

//MagAlpha Read/Write Register Command
#define READ_REG_COMMAND    (0b010 << 13)
#define WRITE_REG_COMMAND   (0b100 << 13)

#define REG_REGMAPID 23
#define REG_SID 24
#define REG_PID 25
#define REG_LOCK0 28
#define REG_LOCK1 29
#define REG_LOCK2 30
#define REG_LOCK3 31

MagAlphaGen3::MagAlphaGen3(){
}

uint16_t MagAlphaGen3::readAngleRaw16(){
    uint16_t angle;
    digitalWrite(_spiChipSelectPin, LOW);
    angle = _spi->transfer16(0x0000); //Read 16-bit angle
    digitalWrite(_spiChipSelectPin, HIGH);
    return angle;
}

uint8_t MagAlphaGen3::readAngleRaw8(){
    uint8_t angle;
    digitalWrite(_spiChipSelectPin, LOW);
    angle = _spi->transfer(0x00);     //Read 8-bit angle
    digitalWrite(_spiChipSelectPin, HIGH);
    return angle;
}

uint16_t MagAlphaGen3::readAngleRaw(bool* error){
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

uint8_t MagAlphaGen3::readRegister(uint8_t address){
    uint8_t readbackRegisterValue;
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(READ_REG_COMMAND | ((address & 0x1F) << 8) | 0x00);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 750ns before register readout
    digitalWrite(_spiChipSelectPin, LOW);
    readbackRegisterValue = ((_spi->transfer16(0x0000) & 0xFF00) >> 8);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 750ns after register readout
    return readbackRegisterValue;
}

uint8_t MagAlphaGen3::writeRegister(uint8_t address, uint8_t value){
    uint8_t readbackRegisterValue;
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(WRITE_REG_COMMAND | ((address & 0x1F) << 8) | value);
    digitalWrite(_spiChipSelectPin, HIGH);
    delay(20); //Wait for 20ms
    digitalWrite(_spiChipSelectPin, LOW);
    readbackRegisterValue = ((_spi->transfer16(0x0000) & 0xFF00) >> 8);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 750ns after register readout
    return readbackRegisterValue;
}

void MagAlphaGen3::readRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister){
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(READ_REG_COMMAND | (((address) & 0x1F) << 8) | 0x00);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 750ns before register readout
    for (uint8_t i=0;i<numberOfRegister-1;++i){
        digitalWrite(_spiChipSelectPin, LOW);
        valueArray[i] = (_spi->transfer16(READ_REG_COMMAND | (((address+i+1) & 0x1F) << 8) | 0x00) >> 8);
        digitalWrite(_spiChipSelectPin, HIGH);
        delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 750ns before register readout
    }
    digitalWrite(_spiChipSelectPin, LOW);
    valueArray[numberOfRegister-1] = (_spi->transfer16(0x0000) >> 8);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 750ns after register readout
}

void MagAlphaGen3::writeRegisterBurst(uint8_t address, uint8_t valueArray[], uint16_t numberOfRegister){
    for (uint8_t i=0;i<numberOfRegister;++i){
        digitalWrite(_spiChipSelectPin, LOW);
        _spi->transfer16(WRITE_REG_COMMAND | (((address+i) & 0x1F) << 8) | valueArray[i]);
        digitalWrite(_spiChipSelectPin, HIGH);
        delay(20); //Wait for 20ms
    }
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(0x0000);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 120ns after register readout
}

/*
uint16_t MagAlphaGen3::detectSensorGeneration(){
    uint16_t chipId;
    digitalWrite(_spiChipSelectPin, LOW);
    _spi->transfer16(0xFF00);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 750ns before register readout
    digitalWrite(_spiChipSelectPin, LOW);
    chipId = _spi->transfer16(0x0000);
    digitalWrite(_spiChipSelectPin, HIGH);
    delayMicroseconds(1); //Wait for 1us (=1000 ns) to respect tIdleReg of 750ns after register readout
    return chipId;
}
*/

uint16_t MagAlphaGen3::getZero()
{
    return (readRegister(1)<<8) + readRegister(0);
}
void MagAlphaGen3::setZero(uint16_t zero){
    writeRegister(0, zero&0xFF);
    writeRegister(1, zero>>8);
}

uint16_t MagAlphaGen3::setCurrentAngleAsZero(){
    uint16_t angle, zero;
    setZero(0);
    angle=readAngleRaw16();
    zero=65536-angle;
    setZero(zero);
    return zero;
}

uint16_t MagAlphaGen3::getBct(){
    return readRegister(2);
}

void MagAlphaGen3::setBct(uint16_t bct){
    writeRegister(2, bct&0xFF);
}

uint16_t MagAlphaGen3::getNumberOfRegisters(){
    return 32;
}

void MagAlphaGen3::getLockedRegisters(uint8_t unlockedRegisters[]){
    uint8_t registerArray[4] = {0};
    readRegisterBurst(REG_LOCK0, registerArray, 4);
    for(uint8_t i=0; i<4; i++){
        for(uint8_t j=0; j<8; j++){
            unlockedRegisters[i*8+j] = ((registerArray[i]>>j)&0x01) ? 0xFF : 0x00;
        }
    }
}

void MagAlphaGen3::getPartNumber(char *partNumber){
    uint8_t partId = readRegister(REG_PID);
    switch(partId) {
        case 1:
            sprintf(partNumber, "MA102");
            break;
        case 2:
            sprintf(partNumber, "MA302");
            break;
        case 21:
            sprintf(partNumber, "MA302GQ-E");
            break;
        case 3:
            sprintf(partNumber, "MA310");
            break;
        case 4:
            sprintf(partNumber, "MA702");
            break;
        case 5:
            sprintf(partNumber, "MA704");
            break;
        case 6:
            sprintf(partNumber, "MA710");
            break;
        case 7:
            sprintf(partNumber, "MA730");
            break;
        case 8:
            sprintf(partNumber, "MA800");
            break;
        case 9:
            sprintf(partNumber, "MA820");
            break;
        case 10:
            sprintf(partNumber, "MA850");
            break;
        case 12:
            sprintf(partNumber, "MAQ430");
            break;
        case 14:
            sprintf(partNumber, "MAQ470");
            break;
        case 16:
            sprintf(partNumber, "MA330");
            break;
        case 17:
            sprintf(partNumber, "MA731");
            break;
        case 18:
            sprintf(partNumber, "MA732");
            break;
        case 20:
            sprintf(partNumber, "MP9961");
            break;
        case 22:
            sprintf(partNumber, "MAQ473");
            break;
        case 23:
            sprintf(partNumber, "MA735");
            break;
        case 24:
            sprintf(partNumber, "MAQ800");
            break;
        case 25:
            sprintf(partNumber, "MAQ820");
            break;
        case 26:
            sprintf(partNumber, "MAQ850");
            break;
        case 128:
            sprintf(partNumber, "MAP790-0000");
            break;
        case 129:
            sprintf(partNumber, "MAP790-0001");
            break;
        case 132:
            sprintf(partNumber, "MAP790-0004");
            break;
        case 250:
            sprintf(partNumber, "MAP790-0010");
            break;
        case 251:
            sprintf(partNumber, "MAP790-0002");
            break;
        case 252:
            sprintf(partNumber, "MAP791-0004");
            break;
        default:
            sprintf(partNumber, "Gen3 [%u] Unknown Part Number", partId);
    }
}

uint8_t MagAlphaGen3::getSiliconId(){
    return (readRegister(REG_SID)  & 0xF0) >> 4;
}

uint8_t MagAlphaGen3::getSiliconRevision(){
    return readRegister(REG_SID) & 0x0F;
}

uint8_t MagAlphaGen3::getRegisterMapRevision(){
    return readRegister(REG_REGMAPID);
}

// void MagAlphaGen3::setSpiClockFrequency(uint32_t speedMaximum){
//     _speedMaximum = speedMaximum;
//     SPI.beginTransaction(SPISettings(_speedMaximum, MSBFIRST, _spiMode));
// }

// void MagAlphaGen3::setSpiDataMode(uint8_t spiMode){
//     _spiMode = spiMode;
//     SPI.beginTransaction(SPISettings(_speedMaximum, MSBFIRST, _spiMode));
// }

// void MagAlphaGen3::setSpiChipSelectPin(uint8_t spiChipSelectPin){
//     _spiChipSelectPin = spiChipSelectPin;
//     pinMode(_spiChipSelectPin, OUTPUT);
//     digitalWrite(_spiChipSelectPin, HIGH);
// }

// double MagAlphaGen3::convertRawAngleToDegree(uint8_t rawAngleDataBitLength, uint16_t rawAngle){
//     double angleInDegree;
//     angleInDegree = (rawAngle*360.0)/((double)pow(2, rawAngleDataBitLength));
//     return angleInDegree;
// }

// MagAlphaSSI::MagAlphaSSI(){
// }
// void  MagAlphaSSI::begin(){
//     _speedMaximum = 1000000;
//     SPI.begin();
//     SPI.beginTransaction(SPISettings(_speedMaximum, MSBFIRST, SSI_MODE));
// }

// void  MagAlphaSSI::begin(int32_t ssiSsckFrequency){
//     _speedMaximum = ssiSsckFrequency;
//     SPI.begin();
//     SPI.beginTransaction(SPISettings(_speedMaximum, MSBFIRST, SSI_MODE));
// }

// void MagAlphaSSI::end(){
//     SPI.end();
// }

// double MagAlphaSSI::readAngle(){
//     uint16_t angle;
//     double angleInDegree;
//     angle = readAngleRaw();
//     angleInDegree = (angle*360.0)/65536.0;
//     return angleInDegree;
// }

// uint16_t MagAlphaSSI::readAngleRaw(){
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

// uint16_t MagAlphaSSI::readAngleRaw(bool* error){
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

// void MagAlphaSSI::setSsiClockFrequency(uint32_t speedMaximum){
//     _speedMaximum = speedMaximum;
//     SPI.beginTransaction(SPISettings(_speedMaximum, MSBFIRST, SSI_MODE));
// }

// double MagAlphaSSI::convertRawAngleToDegree(uint8_t rawAngleDataBitLength, uint16_t rawAngle){
//     double angleInDegree;
//     angleInDegree = (rawAngle*360.0)/((double)pow(2, rawAngleDataBitLength));
//     return angleInDegree;
// }
