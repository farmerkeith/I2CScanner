// --------------------------------------
// Scan and Identify I2C devices by farmerkeith
// 21 January 2019
// Based on i2c_scanner Version 6
//
// This sketch tests all standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//
// Recognised devices are listed as their device type

#include "Wire.h"

void setup() {
  Serial.begin(115200);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.print("\nI2C Scan_ID");
  Serial.println("\nScans for I2C devices and names any that it knows about");
  Wire.begin();
  pinMode(BUILTIN_LED, OUTPUT);

} // end of void setup()

void loop() {
  byte error, address;
  int nDevices;

  digitalWrite(BUILTIN_LED, LOW);
  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 128; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.print(" !");
      nDevices++; // increment device count
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
    if (error == 0) {
      byte data1 = 0;
      switch (address) {
        case 0x1D: 
          Serial.println(" ADXL345 digital accelerometer");
          break;
        case 0x1E:
          Serial.println(" HMC5883L 3-axis digital compass");
          break;
        case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26:
          Serial.println(" PCF8574 I/O expander");
          break;
        case 0x27:
          Serial.println(" LCD with I2C backpack");
          Serial.println(" PCF8574 I/O expander");
          break;
        case 0x31: case 0x32: case 0x33: case 0x34: case 0x35: case 0x36: case 0x37: case 0x38:
          Serial.println(" PCF8574A I/O expander");
          break;
        case 0x40:
          Serial.println(" HTU21D digital humidity and temperature sensor");
          break;
        case 0x48: case 0x49: case 0x4A: case 0x4B: 
          Serial.println(" ADS1113, ADS1114, ADS1115, ADS1013, ADS1014, ADS1015");
          break;
        case 0x50: case 0x51: case 0x52: case 0x54: case 0x55: case 0x56: case 0x57:
          Serial.println(" AT24C32/64 Eeprom family");
          break;
        case 0x53:  
          Serial.println(" ADXL345 digital accelerometer");
          Serial.println(" or AT24C32/64 Eeprom family");
        case 0x68:
          Serial.println(" DS3231 or DS1307 Real Time Clock");
          Serial.println(" or MPU9250 gyroscope, accelerometer, magnetometer");
          Serial.println(" or L3G4200D gyroscope");
          break;
        case 0x69: // same device also on 0x68
        // also need to study pass-through mode of MPU9250
          Serial.println(" MPU9250 gyroscope, accelerometer, magnetometer");
          Serial.println(" or L3G4200D gyroscope");
          break;
        case 0x76: case 0x77:
          Serial.println(" BMP280 or BME280");
          Wire.beginTransmission(address);
          // Select register
          Wire.write(0xD0); // 0xD0 hex address of ID
          // Stop I2C Transmission
          Wire.endTransmission();
          // Request 1 bytes of data
          Wire.requestFrom(address, 1);
          // Read 1 byte of data
          if (Wire.available() == 1)  {
            data1 = Wire.read();
          } // end of if (Wire.available() == 3)
          Serial.print ("Device ID=");
          Serial.print(data1, HEX);
          if (data1 == 0x58) Serial.println(" = BMP280");
          else if (data1 == 0x60) Serial.println(" = BME280");
          else Serial.println(" ID not in list");
          break;
        default:
          Serial.println("device not in list");
          break;
      }
    }
  } // end of for (address = 1; address < 128; address++ )

  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
    digitalWrite(BUILTIN_LED, HIGH);
  }
  delay(5000);           // wait 5 seconds for next scan
} // end of void loop()


