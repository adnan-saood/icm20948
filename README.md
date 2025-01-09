# ICM-20948 for ESP-IDF

This is a modified copy of the the SparkFun Arduino library for the TDK InvenSense ICM-20948 Inertial Measurement Unit 9-Degree Of Freedom sensor from [SparkFun_ICM-20948_ArduinoLibrary](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary). It bases on the Portable C99 code and uses the I2C and SPI driver of ESP-IDF. The code bases on Version 1.3 of the SparkFun Arduino library. 

Version 1.2 of the library includes support for the InvenSense Digital Motion Processor (DMP™). You can find further details in [DMP.md](docs/DMP.md).

# Supported Functions
* I2C
* SPI
* DMP

# Example

Not all examples are ported. For further examples, please look at the original project.

1. Clone Repository
2. Goto to example directory (for example `spi_agmt`)
3. Modify pins (MISO, MOSI, CS...) in `main/main.c`
4. `idf.py set-target ...`
5. `idf.py build flash monitor`

# Related projects
* [SparkFun_ICM-20948_ArduinoLibrary](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary)
  * I2C
  * SPI
  * DMP support via SPI
  * Arduino
* [hellvesper/icm20948-i2c](https://github.com/hellvesper/icm20948-i2c)
  * I2C
  * DMP support via i2c
  * ESP-IDF
* [wollewald/ICM20948_WE](https://github.com/wollewald/ICM20948_WE)
  * I2C
  * SPI
  * Arduino
  * no DMP support
* [isouriadakis/Arduino_ICM20948_DMP_Full-Function](https://github.com/isouriadakis/Arduino_ICM20948_DMP_Full-Function)
  * I2C
  * SPI
  * Ardino
  * DMP support
