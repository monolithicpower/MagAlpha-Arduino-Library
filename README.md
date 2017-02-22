# MagAlpha library
Arduino library for the MPS MagAlpha magnetic angle sensor.

## About
MagAlpha sensor detects the absolute angular position of a permanent magnet, typically a diametrically magnetized cylinder on the rotating shaft.

For more information on the MagAlpha sensor family:
* [MagAlpha Product Overview](http://www.monolithicpower.com/Products/Position-Sensors/Products-Overview)
* [MagAlpha Support Materials](http://www.monolithicpower.com/Design-Support/Position-Sensors-Design-Support)

## Supported sensors
Supports all 3rd generation MagAlpha magnetic angle sensors from [Monolithic Power Systems](https://www.monolithicpower.com/).

| Applications | Part Numbers |
| ------------| ------------ |
| Turning knob applications (potentiometer replacement) | MA800, MA820, MA850 |
| Rotary encoders (optical encoder replacement, Servo motors, ...) | MA702, MA704, MA710, MA730 |
| Position controlled motor drivers (FOC, ...) | MA302, MA310 |
| Motor commutation (hall switches replacement) | MA102 |


## License
Written by Mathieu Kaelin for Monolithic Power Systems.
MIT license, all text above must be included in any redistribution.


## Connections
### Power supply
| Arduino  | MagAlpha |
| -------- | -------- |
| GND      | GND      |
| **+3.3V** (Not 5V)| VDD  |

| Warning |
| :-------: |
| Unlike most Arduino & Genuino boards, the **MagAlpha runs at 3.3V**. Even if the I/O can tolerate 5V, check that the voltage applied to VDD is at 3.3V. **Applying a voltages higher than 3.3V to the VDD pin could damage the sensor**.|

### Serial communication
All MagAlpha have a SPI communication interface. Some sensors like the MA702 also have an additional SSI (2-wire) interface.

#### SPI (4-wire interface)
| Arduino  | MagAlpha |
| -------- | -------- |
| MOSI     | MOSI     |
| MISO     | MISO     |
| SCK      | SCLK     |
| any available digital pin (default: pin 0) | CS |

#### SSI (2-wire interface)
| Arduino  | MagAlpha |
| -------- | -------- |
| MISO     | SSD      |
| SCK      | SSCK     |

#### Arduino SPI pin mapping
| Arduino / Genuino Board | MOSI | MISO | SCK | CS  | Voltage Level |
| ----------------------- | ---- | ---- | ---- | :---: | ------------- |
| Zero                    | ICSP-4 | ICSP-1 | ICSP-3 | defined by the user, any digital pin (default: pin 0)  | +3.3V |
| MKRZero                 | 8 | 10 | 9 | defined by the user, any digital pin (default: pin 0)  | +3.3V |
| MKR1000                 | 8 | 10 | 9 | defined by the user, any digital pin (default: pin 0)  | +3.3V |
| 101                     | 11 or ICSP-4 | 12 or ICSP-1 | 13 or ICSP-3 | defined by the user, any digital pin (default: pin 0)  | +3.3V |
| Due                     | ICSP-4 | ICSP-1 | ICSP-3 | defined by the user, any digital pin (default: pin 0)  | +3.3V |
| Uno or Duemilanove      | 11 or ICSP-4 | 12 or ICSP-1 | 13 or ICSP-3 | defined by the user, any digital pin. **Default pin must be changed because pin 0 is also used by the Serial port**. (use pin 7 for example)  | +5V |
| Mega1280 or Mega2560    | 51 or ICSP-4 | 50 or ICSP-1 | 52 or ICSP-3 | defined by the user, any digital pin (default: pin 0)  | +5V |
| Leonardo                | ICSP-4 | ICSP-1 | ICSP-3 | defined by the user, any digital pin (default: pin 0)  | +5V |

Check [Arduino SPI library reference page](https://www.arduino.cc/en/Reference/SPI) for more information on the SPI signal connections.

## Setup
Install the library directly from within the Arduino IDE by using the Library Manager (Sketch => Include Library => Manage Libraries...).

It is also possible to import the library Zip file (check release tab) from the Arduino IDE (Sketch => Include Library => Add .ZIP Library...).

The library can also be manually installed by copying the MagAlpha library folder in your arduinosketchfolder/libraries/ folder. You may need to create the libraries subfolder if its your first library. Restart the IDE to see the library.

Check this tutorial on Arduino library installation for more information:
* [All About Arduino Libraries](http://learn.adafruit.com/adafruit-all-about-arduino-libraries-install-use)
