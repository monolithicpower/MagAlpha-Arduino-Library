#include <SPI.h>

#include <MagAlphaGen8.h>
#include <MagAlphaPartProperties.h>

#define SPI_CS_PIN              (7)
#define SPI_SCLK_FREQUENCY      (1000000)

MagAlphaGen8 magalpha;

double angle_real, speed;
uint16_t zero, angle_raw, turn;
uint8_t register_value[2];
bool crc_error;

void setup() {

    magalpha.begin(SPI_SCLK_FREQUENCY, MagAlphaSPIMode::MODE_0, SPI_CS_PIN, &SPI);
    // Other ways of use of begin() that will set default settings to the SPI peripheral
    // magalpha.begin(SPI_CS_PIN, &SPI);

    //magalpha.setSpiClockFrequency(SPI_SCLK_FREQUENCY);
    //magalpha.setSpiDataMode(MagAlphaSPIMode::MODE0);
    //magalpha.setSpiChipSelectPin(SPI_CS_PIN);
    Serial.begin(115200);
    while(!Serial);
}

void loop() {

    // Read & Write registers
    register_value[0] = magalpha.writeRegister(0, 0xAA);
    Serial.println(register_value[0], HEX);
    register_value[0] = magalpha.readRegister(0);
    Serial.println(register_value[0], HEX);
    register_value[0] = magalpha.writeRegister(0, 0x00);

    // Read & Write registers in burst mode
    register_value[0] = 0xAA;
    register_value[1] = 0x55;
    magalpha.writeRegisterBurst(0, register_value, 2);
    magalpha.readRegisterBurst(0, register_value, 2);
    for(uint8_t i = 0; i < sizeof(register_value); i++) {
        Serial.print("Register ");
        Serial.print(i);
        Serial.print(" = ");
        Serial.println(register_value[i], HEX);
    }
    register_value[0] = 0x00;
    register_value[1] = 0x00;
    magalpha.writeRegisterBurst(0, register_value, 2);

    // Set & Get zero setting
    zero = magalpha.setCurrentAngleAsZero();
    Serial.print("Zero setting = ");
    Serial.println(zero, HEX);
    magalpha.setZero(0x0000);
    zero = magalpha.getZero();
    Serial.print("Zero setting = ");
    Serial.println(zero, HEX);

    // Memory management
    magalpha.writeRegister(0, 0xAA);
    magalpha.storeRegister(0);
    magalpha.writeRegister(0, 0x00);
    magalpha.restoreAllRegisters();
    if(magalpha.readRegister(0) != 0xAA) {
        Serial.println("An error occured during restoreAllRegisters");
    }
    magalpha.writeRegister(0, 0x00);
    magalpha.storeAllRegisters();
    magalpha.restoreAllRegisters();
    if(magalpha.readRegister(0) != 0x00) {
        Serial.println("An error occured during storeAllRegisters");
    }

    // Clear error flags
    magalpha.clearErrorFlags();

    speed = magalpha.readSpeed();
    Serial.println(speed);
    turn = magalpha.readTurn();
    Serial.println(turn);

    // Read angle
    angle_raw = magalpha.readAngleRaw8();
    Serial.print("Angle raw (8 bits) = ");
    Serial.println(angle_raw, HEX);
    angle_raw = magalpha.readAngleRaw16();
    Serial.print("Angle raw (16 bits) = ");
    Serial.println(angle_raw, HEX);
    angle_raw = magalpha.readAngleRaw(&crc_error);
    if(crc_error) {
        Serial.println("An error occured during readAngleRaw");
    } else {
        Serial.print("readAngleRaw succeded, Angle raw (16 bits) = ");
        Serial.println(angle_raw, HEX);
    }

    angle_real = magalpha.readAngle();
    Serial.print("Angle in degree = ");
    Serial.println(angle_real, DEC);

    // This delay has been added only for visual purpose of the terminal
    delay(25);
}
