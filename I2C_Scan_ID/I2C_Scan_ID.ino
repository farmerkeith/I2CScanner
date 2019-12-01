// --------------------------------------
// Scan and Identify I2C devices by farmerkeith
// modified 01 December 2019
// Based on i2c_scanner Version 6
//
// This sketch tests all standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//
// Recognized devices are listed as their device type
// Could be greatly enhanced using https://learn.adafruit.com/i2c-addresses/the-list

#include <Wire.h>

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial);             // Leonardo: wait for serial monitor

  uint8_t rtn = I2C_ClearBus(); // clear the I2C bus first before calling Wire.begin()
  if (rtn != 0) {
    Serial.println(F("I2C bus error. Could not clear"));
    if (rtn == 1) {
      Serial.println(F("SCL clock line held low"));
    } else if (rtn == 2) {
      Serial.println(F("SCL clock line held low by slave clock stretch"));
    } else if (rtn == 3) {
      Serial.println(F("SDA data line held low"));
    }
  } else { // bus clear
    // now we can start Arduino Wire as a master
  }
  Serial.println();  // clear any junk from boot
  Serial.print(F("\nI2C Scan_ID"));
  Serial.println(F("\nScans for I2C devices and names any that it knows about"));
  Wire.begin();
  Serial.println(F("setup finished\n"));

  //#ifdef ESP8266
  //Wire.setClockStretchLimit(1500);  // only needed if using pre-2.6.3 libraries
  //#endif
} // end of setup()


void loop() {
  byte error, address;
  uint8_t nDevices;
  byte numBytes = 1;  //gets rid of the 'ambiguous' warning

  Serial.println(F("Scanning..."));

  nDevices = 0;
  for (address = 8; address < 120; address++ )  //don't scan reserved addresses <8 or >119
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmission to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    delayMicroseconds(10);
    if (error == 0) {
      Serial.print(F("I2C device found at address 0x"));
      if (address < 16) Serial.print(F("0"));
      Serial.print(address, HEX);
      Serial.print(F(" !"));
      nDevices++; // increment device count
    } else if (error == 4) {
      Serial.print(F("Unknown error at address 0x"));
      if (address < 16) Serial.print(F("0"));
      Serial.println(address, HEX);
    }
    if (error == 0) {
      byte data1 = 0;
      switch (address) {
        case 0x1D:
          Serial.println(F(" ADXL345 digital accelerometer"));
          break;
        case 0x1E:
          Serial.println(F(" HMC5883L 3-axis digital compass"));
          break;
        case 0x20: case 0x21:
          Serial.println(F(" PCF8574 or MCP23017 I/O expander"));
          Serial.println(F(" or FXAS21002 gyro"));
          break;
        case 0x22: case 0x23: case 0x24: case 0x25: case 0x26:
          Serial.println(F(" PCF8574 or MCP23017 I/O expander"));
          break;
        case 0x27:
          Serial.println(F(" PCF8574 or MCP23017 I/O expander"));
          Serial.println(F(" or LCD with I2C backpack"));
          break;
        case 0x28:
          Serial.println(F(" BNO055 9-DOF Absolute Orientation sensor"));
          break;
        case 0x29:
          Serial.println(F(" TSC3472 color sensor"));
          Serial.println(F(" or BNO055 9-DOF Absolute Orientation sensor"));
          break;
        case 0x38:
          Serial.println(F(" PCF8574A I/O expander"));
          break;
        case 0x39:
          Serial.println(F(" PCF8574A I/O expander"));
          Serial.println(F(" or TSC3472 color sensor"));
          break;
        case 0x3A: case 0x3B:
          Serial.println(F(" PCF8574A I/O expander"));
          break;
        case 0x3C: case 0x3D:
          Serial.println(F(" PCF8574A I/O expander"));
          Serial.println(F(" or OLED with SSD1306 controller"));
          break;
        case 0x3E:
          Serial.println(F(" PCF8574A I/O expander"));
          break;
        case 0x3F:
          Serial.println(F(" PCF8574A I/O expander"));
          Serial.println(F(" or LCD with I2C backpack"));
          break;
        case 0x40:
          Serial.println(F(" HTU21D digital humidity & temperature sensor or INA219 current monitor"));
          break;
        case 0x41: case 0x42: case 0x43:
          Serial.println(F(" INA219 current monitor"));
          break;
        case 0x44: case 0x45:
          Serial.println(F(" INA219 current monitor"));
          Serial.println(F(" or SHT3x-DIS temperature & humidity sensor"));
          break;
        case 0x46: case 0x47:
          Serial.println(F(" INA219 current monitor"));
          break;
        case 0x48:
          Serial.println(F(" ADS1113, ADS1114, ADS1115, ADS1013, ADS1014, ADS1015"));
          Serial.println(F(" or INA219 current monitor"));
          Serial.println(F(" or PN532 NFC/RFID controller "));
          break;
        case 0x49: case 0x4A: case 0x4B:
          Serial.println(F(" ADS1113, ADS1114, ADS1115, ADS1013, ADS1014, ADS1015"));
          Serial.println(F(" or INA219 current monitor"));
          break;
        case 0x4C: case 0x4D: case 0x4E: case 0x4F:
          Serial.println(F(" INA219 current monitor"));
          break;
        case 0x50: case 0x51: case 0x52: case 0x54: case 0x55: case 0x56: case 0x57:
          Serial.println(F(" AT24C32/64 EEPROM family"));
          break;
        case 0x53:
          Serial.println(F(" ADXL345 digital accelerometer"));
          Serial.println(F(" or AT24C32/64 EEPROM family"));
          break;
        case 0x5A: case 0x5B:
          Wire.beginTransmission(address);
          // Select register
          Wire.write(0x20); // 0x20 hex address of ID register
          // Stop I2C Transmission
          Wire.endTransmission();
          delayMicroseconds(10);
          // Request 1 bytes of data
          Wire.requestFrom(address, numBytes);
          // Read 1 byte of data
          if (Wire.available() == 1)  {
            data1 = Wire.read();
          }
          if (data1 == 0x81) {
            Serial.print(F("Device ID="));
            Serial.print(data1, HEX);
            Serial.println(F(" AMS CCS811 eCO2 TVOC sensor"));
          }
          else Serial.println(F("Unknown device"));
          break;
        case 0x5C:
          Serial.println(F(" AM2315 temperature & humidity sensor"));
          break;
        case 0x68:
          Serial.println(F(" DS3231 or DS1307 Real Time Clock"));
          Serial.println(F(" or MPU9250 gyroscope, accelerometer, magnetometer"));
          Serial.println(F(" or L3G4200D gyroscope"));
          break;
        case 0x69: // same device also on 0x68
          // also need to study pass-through mode of MPU9250
          Serial.println(F(" MPU9250 gyroscope, accelerometer, magnetometer"));
          Serial.println(F(" or L3G4200D gyroscope"));
          break;
        case 0x76: case 0x77:
          Serial.println(F(" BMP180, BMP280, BME280 or BME680 or MS5607,MS5611,MS5637"));
          // note: address 0x77 may be BMP085,BMA180 and may not be MS5607 or MS5637 CHECK
          Wire.beginTransmission(address);
          // Select register
          Wire.write(0xD0); // 0xD0 hex address of Bosch chip_ID
          // Stop I2C Transmission
          Wire.endTransmission();
          delayMicroseconds(10);
          // Request 1 bytes of data
          Wire.requestFrom(address, numBytes);
          // Read 1 byte of data
          if (Wire.available() == 1)  {
            data1 = Wire.read();
          } // end of if Wire.available()
          Serial.print(F("Device ID="));
          Serial.print(data1, HEX);
          if (data1 == 0x58) Serial.println(F(" = BMP280"));
          else if (data1 == 0x60) Serial.println(F(" = BME280"));
          else if (data1 == 0x55) Serial.println(F(" = BMP180"));
          else if (data1 == 0x61) Serial.println(F(" = BME680"));
          else Serial.println(F(" ID not in list"));
          break;
        default:
          Serial.println(F("device not in list"));
          break;
      }
    }
  } // end of for (address = bottom; address < top; address++ )

  if (nDevices == 0) {
    Serial.println(F("No I2C devices found\n"));
  } else {
    Serial.println(F("scan complete\n"));
  }
  digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);  // toggle the LED so we know the scanner isn't hung
  delay(2000);           // wait 2 seconds for next scan
} // end of loop()

/**
   I2C_ClearBus
   (http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html)
   (c)2014 Forward Computing and Control Pty. Ltd.
   NSW Australia, www.forward.com.au
   This code may be freely used for both private and commerical use

  This routine turns off the I2C bus and clears it
  on return SCA and SCL pins are tri - state inputs.
              You need to call Wire.begin() after this to re - enable I2C
              This routine does NOT use the Wire library at all.

              returns 0 if bus cleared
                      1 if SCL held low.
                      2 if SDA held low by slave clock stretch for > 2sec
                      3 if SDA held low after 20 clocks.
*/
uint8_t I2C_ClearBus() {
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  delay(2500);
  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 & similar slow parts to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to finish uploading the program before existing sketch
  // confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master.
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  uint8_t clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
    // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    uint8_t counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}
