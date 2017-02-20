#include <MagAlpha.h>

//Check https://www.arduino.cc/en/reference/SPI for SPI signals connections
//Connect SSI SSCK to SPI SCLK signal
//Connect SSI SSD to SPI MISO signal

#define UART_BAUDRATE       115200        //UART data rate in bits per second (baud)
#define SSI_SSCK_FREQUENCY  1000000       //SSI SSCK Clock frequency in Hz

MagAlphaSSI magAlphaSsi;

void setup() {
  // put your setup code here, to run once:
  //Set the SSI SSCK frequency
  magAlphaSsi.begin(SSI_SSCK_FREQUENCY);
  //Set the Serial Communication used to report the angle
  Serial.begin(UART_BAUDRATE);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t angleRaw16;
  bool error;
  //Read the angle
  angleRaw16 = magAlphaSsi.readAngleRaw(&error);
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
