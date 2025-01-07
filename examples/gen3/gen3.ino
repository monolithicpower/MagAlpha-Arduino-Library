#include <SPI.h>

#include <MagAlphaGen3.h>
#include <MagAlphaPartProperties.h>

#define SPI_CS_PIN              (7)
#define SPI_SCLK_FREQUENCY      (1000000)

MagAlphaGen3 magalpha;

double angle_real;
uint16_t zero, bct, angle_raw;
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

    // Set & Get BCT setting
    magalpha.setBct(0xAA55);
    bct = magalpha.getBct();
    Serial.print("BCT = ");
    Serial.println(bct, HEX);
    magalpha.setBct(0x00);

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