# MagAlpha library
Arduino library for the MPS MagAlpha magnetic angle sensor.

Supports MagAlpha 3rd generation Sensors. MagAlpha sensor detects the absolute angular position of a permanent magnet, typically a diametrically magnetized cylinder on the rotating shaft.

For more information on the MagAlpha sensor family:
* [MagAlpha Product Overview](http://www.monolithicpower.com/Products/Position-Sensors/Products-Overview)
* [MagAlpha Support Materials](http://www.monolithicpower.com/Design-Support/Position-Sensors-Design-Support)

Check [Arduino SPI library reference page](https://www.arduino.cc/en/Reference/SPI) for the SPI signal connections.

| Warning |
| ------- |
| Unlike most Arduino & Genuino boards, the MagAlpha runs at 3.3V. Even if the I/O can tolerate 5V, check that the voltage applied to VDD is at 3.3V. Applying a voltages higher than 3.3V to the VDD pin could damage the sensor.|

Written by Mathieu Kaelin for Monolithic Power Systems.
MIT license, all text above must be included in any redistribution

Place the MagAlpha library folder in your arduinosketchfolder/libraries/ folder.
You may need to create the libraries subfolder if its your first library. Restart the IDE.

You can also check this tutorial on Arduino library installation:
* [All About Arduino Libraries](http://learn.adafruit.com/adafruit-all-about-arduino-libraries-install-use)
