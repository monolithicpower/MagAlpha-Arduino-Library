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
  double angle;
  //Read the angle
  angle = magAlphaSsi.readAngle();
  Serial.println(angle, 3);
  //Wait before the next angle measurement (not needed by the sensor, only targeted to make the output easier to read)
  delay(25);
}
