#include <MagAlpha.h>

//Check https://www.arduino.cc/en/reference/SPI for SPI signals connections

#define UART_BAUDRATE       115200        //UART data rate in bits per second (baud)
#define SPI_SCLK_FREQUENCY  10000000      //SPI SCLK Clock frequency in Hz
#define SPI_CS_PIN          0             //SPI CS pin

MagAlpha magAlpha;

void setup() {
  // put your setup code here, to run once:
  //Set the SPI SCLK frequency, SPI Mode and CS pin
  magAlpha.begin(SPI_SCLK_FREQUENCY, MA_SPI_MODE_3, SPI_CS_PIN);
  //Set the Serial Communication used to report the angle
  Serial.begin(UART_BAUDRATE);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t angleRaw16;
  bool error;
  //Read the angle
  angleRaw16 = magAlpha.readAngleRaw(&error);
  Serial.print(angleRaw16, DEC);
  if (error){
    Serial.print("\t => Communication Error Detected");
  }
  else{
    Serial.print("\t => Communication Succeeded");
  }
  Serial.println();
  //Wait before the next angle measurement (not needed by the sensor, only targeted to make the output easier to read)
  delay(25);
}
