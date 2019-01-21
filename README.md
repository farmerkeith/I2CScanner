# I2CScanner
Simple software to scan an I2C bus and report devices found.
Run this code on your Arduino IDE with a microcontroller connected to any I2C devices. 
The software checks every I2C address from 0 to 127 and reports any devices found. 
The I2C addresses used by a number of devices are recognised. Where the same address is used by several different devices, the possible devices are listed. 
If it finds a BMP280 or BME280 it reports which of these the device is by reading its identification code.
