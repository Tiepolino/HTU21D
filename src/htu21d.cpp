/*******************************************************************************
 * HTU21D.cpp
 *
 * Interface for the HTU21D, Temperature and Relative Humidity sensor
 *
 * Author				: Tiebolt van der Linden
 * Created			: 2022-08-29 / 15.06
 * Last Changed	: 2022-08-31 / 21.36
 *
 * ToDo
 *    
 * History
 *    20220831 - Code completed
 *		20220829 - Initial Version
 */

#include "htu21d.h"

/**
 * @brief Construct a new HTU21D::HTU21D object
 * 
 * @param wire - TwoWire object to be used
 */
HTU21D::HTU21D(TwoWire *wire) : _wire(wire) {
  _temperature = 0.0; _humidity = 0.0;
  _temp_valid = false; _humid_valid = false;
}

#if defined (ESP8266) || defined (ESP32)
/**
 * @brief Check if the sensor is available and working
 * 
 * @param i2cAddress - The i2c address of the sensor
 * @param sda        - SDA pin on the micro controller
 * @param scl        - SCL pint on the micro controller
 * 
 * @return true      - on success
 * @return false     - on failure (lastError will be set)
 */
bool HTU21D::begin(uint8_t i2cAddress, uint8_t sda, uint8_t scl) {
  uint8_t result = 0;

  while(millis() < HTU21D_INIT_DELAY);                                          // Make sure the sensor is initialized

  // ---- Check if we need to setup I2C pins ----
  if ((sda < 0xff) && (scl < 0xff)) {
    _wire->begin(sda, scl);
  } else {
    _wire->begin();
  }

  _i2cAddress = i2cAddress;                                                     // Set the i2c address
  _lastError = HTU21D_ERROR_NONE;                                               // Just starting so no error

  // ---- Check if i2cAddress exists ----
  _wire->beginTransmission(_i2cAddress);                                        // Try sending to the i2c address
  result = _wire->endTransmission();                                            // Get the result from the sensor
  if (result != 0) {                                                            // Check if the sensor is there
    _lastError = HTU21D_ERROR_NOSENSOR;                                         // If not set lastError
    return false;
  }

  // ---- Check for HTU21D sensor ----
  _wire->beginTransmission(_i2cAddress);                                        // Start communication with the sensor
  _wire->write(HTU21D_CMD_READ_USER_REG);                                       // Read out the user register
  result = _wire->endTransmission();                                            // Get the transmission result
  if (result != 0) {                                                            // If not set lastError
    _lastError = HTU21D_ERROR_I2CWRITE;
    return false;
  }
  delay(1);                                                                     // Give the sensor some time to process
  _wire->requestFrom((uint8_t) _i2cAddress, (uint8_t) 1);                       // Get the user register data
  result = _wire.read();
  return ((result & 0x02) == 0x02);                                             // Bit 2 is 1 by default, all others are 0
}
#else
/**
 * @brief Check if the sensor is available and working
 * 
 * @param i2cAddress - i2c Address of the sensor
 * 
 * @return true      - on success
 * @return false     - on failure (lastError will be set)
 */
bool HTU21D::begin(uint8_t i2cAddress) {
  uint8_t result = 0;
  _i2cAddress = i2cAddress;                                                     // Set the i2c address

  // ---- Check if there is a sensor at the i2c address ----
  while(millis() < HTU21D_INIT_DELAY);                                          // Make sure the sensor is initialized
  _wire->begin();                                                               // Startup the i2c bus
  _wire->beginTransmission(_i2cAddress);                                        // Try sending to the i2c address
  result = _wire->endTransmission();                                            // Get the result from the sensor
  if (result != 0) {                                                            // Check if the sensor is there
    _lastError = HTU21D_ERROR_NOSENSOR;                                         // If not set lastError
    return false;
  }

  // ---- Check for HTU21D sensor ----
  _wire->beginTransmission(_i2cAddress);                                        // Start communication with the sensor
  _wire->write(HTU21D_CMD_READ_USER_REG);                                       // Read out the user register
  result = _wire->endTransmission();                                            // Get the transmission result
  if (result != 0) {                                                            // If not set lastError
    _lastError = HTU21D_ERROR_I2CWRITE;
    return false;
  }
  delay(1);                                                                     // Give the sensor some time to process
  _wire->requestFrom((uint8_t) _i2cAddress, (uint8_t) 1);                       // Get the user register data
  result = _wire->read();

  softReset();                                                                  // Datasheet suggest we do this

  return ((result & 0x02) == 0x02);                                             // Bit 2 is 1 by default, all others are 0
}
#endif

/**
 * @brief Sets the resolution the sensor uses for measurements
 * 
 * @param resolution - The resolution to be used
 * @return true      - On success
 * @return false     - On failure (lastError will be set)
 */
bool HTU21D::setResolution(uint8_t resolution) {
  uint8_t userReg = 0;

  _lastError = HTU21D_ERROR_NONE;                                               // Reset lastError

  if (!_readUserRegister(&userReg)) return false;                               // Read the current value of the user register
  userReg = (userReg & 0x7E) | (resolution & 0x81);                             // Only change the resolution bits
  if (!_writeUserRegister(userReg)) return false;                               // Write the new value into the user register

  return true;
}

/**
 * @brief Enable or disable the on-chip heater
 * 
 * @param state   - true will enable the heater, false will turn it off
 * @return true   - On success
 * @return false  - On Failure (lastError will be set)
 */
bool HTU21D::enableHeater(bool state) {
  uint8_t userReg = 0;

  _lastError = HTU21D_ERROR_NONE;                                               // Reset lastError

  if (!_readUserRegister(&userReg)) return false;                               // Read the current value of the user register
  userReg = (userReg & 0xFB) | state << 2;                                      // Only change the heater bit
  if (!_writeUserRegister(userReg)) return false;                               // Write the new value into the user register

  return true;
}

/**
 * @brief Returns the current state of the on-chip heater
 * 
 * @return true  - Heater is enabled
 * @return false - Heater is disabled or error (check lastError for errors)
 */
bool HTU21D::heaterEnabled(void) {
  uint8_t userReg = 0;

  _lastError = HTU21D_ERROR_NONE;                                               // Reset lastError

  if (!_readUserRegister(&userReg)) return false;                               // Read the current value of the user register
  
  return ((userReg & HTU21D_HEATER) == HTU21D_HEATER);                          // Return the current state of the heater
}

/**
 * @brief Enable or disable the OTP reload function
 * 
 * @param state   - True will enable OTP reload, false will disable it
 * @return true   - On success
 * @return false  - On failure (lastError will be set)
 */
bool HTU21D::enableOTP(bool state) {
  uint8_t userReg = 0;

  _lastError = HTU21D_ERROR_NONE;                                               // Reset lastError

  if (!_readUserRegister(&userReg)) return false;                               // Read the current value of the user register
  userReg = (userReg & 0xFD) | state << 1;                                      // Only change the OTP bit
  if (!_writeUserRegister(userReg)) return false;                               // Write the new value into the user register

  return true;
}

/**
 * @brief Get the current state of the OTP reload bit
 * 
 * @return true  - OTP reload is enabled
 * @return false - OTP reload is disabled or error (check lastError for errors)
 */
bool HTU21D::otpEnabled(void) {
  uint8_t userReg = 0;

  _lastError = HTU21D_ERROR_NONE;                                               // Reset lastError

  if (!_readUserRegister(&userReg)) return false;                               // Read the current value of the user register
  
  return ((userReg & HTU21D_OTP) == HTU21D_OTP);                                // Return the current OTP reload setting
}

/**
 * @brief Checks if VDD is above or below 2.25V
 * 
 * @return true   - VDD is above 2.25V
 * @return false  - VDD is below 2.25V or error (check lastError for errors)
 */
bool HTU21D::vddLow(void) {
  uint8_t userReg = 0;

  _lastError = HTU21D_ERROR_NONE;                                               // Reset lastError

  if (!_readUserRegister(&userReg)) return false;                               // Read the current value of the user register
  return ((userReg & HTU21D_VDD_LEVEL) == HTU21D_VDD_LEVEL);                    // Return the VDD status
}

/**
 * @brief Returns the measured temperature in the requested scale
 * 
 * @param scale   - The scale to present the temperature in
 * @return float  - Measured temperature in requested scale
 */
float HTU21D::getTemperature(uint8_t scale) {
  if (!_temp_valid) { _lastError = HTU21D_ERROR_VALUE_INVALID; }

  _temp_valid = false;                                                          // The measurement just got used

  switch(scale) {
    case _CELCIUS :
        return _temperature;                                                    // Return the temperature in Celcius
        break;
    case _FAHRENHEIT :
        return (_temperature * 1.8) + 32;                                       // Return the temperature in Farenheit
        break;
    case _KELVIN :
        return _temperature + 273.15;                                           // Return the temperature in Kelvin
        break;
  }

  return 0;
}

/**
 * @brief Returns the measured humidity
 * 
 * @return float - The measured humidity
 */
float HTU21D::getHumidity(void) {
  if (!_humid_valid) { _lastError = HTU21D_ERROR_VALUE_INVALID; }

  _humid_valid = false;                                                         // The measurement just got used
  return _humidity;
}

/**
 * @brief Reads both the temperature and humidity from the sensor
 * 
 * @return true   - On success
 * @return false  - Of failure (lastError will be set)
 */
bool HTU21D::readSensorData(void) {
  uint16_t buffer = 0;

  _lastError = HTU21D_ERROR_NONE;                                               // Just started, no errors

  // ---- Get the temperature from the sensor ----
  if (!_readData(HTU21D_CMD_MEASURE_TEMP, &buffer, HTU21D_TIMEOUT))
    return false;
  _temperature = ((float) buffer * (175.72 / 65535.0)) - 46.85;                 // Calculate the temperature
  _temp_valid = true;                                                           // We have a valid temperature

  // ---- Get the humidity from the sensor ----
  if (!_readData(HTU21D_CMD_MEASURE_HUMID, &buffer, HTU21D_TIMEOUT))            // Read the humidity from the sensor
    return false;
  _humidity = ((float) buffer * (125.0 / 65535.0)) - 6.0;                       // Calculate the humidity
  _humid_valid = true;                                                          // We have a valid temperature
  
  return true;
}

/**
 * @brief Soft resets the sensor
 * 
 * @return true  - On succes
 * @return false - On failure (lastError will be set)
 */
bool HTU21D::softReset(void) {
  _lastError = HTU21D_ERROR_NONE;                                               // Clear lastError

  _wire->beginTransmission(_i2cAddress);                                        // Start communication with the sensor
  _wire->write(HTU21D_CMD_SOFT_RESET);                                          // Issue a soft reset command

  // ---- Check if transmission was successfull ----
  if (_wire->endTransmission() != 0) {
    _lastError = HTU21D_ERROR_I2CWRITE;
    return false;
  }

  return true;
}

/**
 * @brief Returns the last error that has occured
 * 
 * @return uint8_t - The number of the last error
 */
uint8_t HTU21D::getLastError(void) {
  return _lastError;
}

/**
 * @brief Reads the value of the user register
 * 
 * @param userReg - uint8_t pointer to store the value
 * @return true   - On success
 * @return false  - On Failure (lastError will be set)
 */
bool HTU21D::_readUserRegister(uint8_t *userReg) {
  _lastError = HTU21D_ERROR_NONE;                                               // Reset lastError

  _wire->beginTransmission(_i2cAddress);                                        // Start communication with the sensor
  _wire->write(HTU21D_CMD_READ_USER_REG);                                       // Issue the read user register command to the sensor
  
  // ---- Check if transmission was successfull ----
  if (_wire->endTransmission() != 0) {
    _lastError = HTU21D_ERROR_I2CWRITE;
    return false;
  }

  _wire->requestFrom((uint8_t) _i2cAddress, (uint8_t) 1);                       // Read the result from the sensor
  *userReg = _wire->read();                                                     // Write the result

  return true;
}

/**
 * @brief Update the user register
 * 
 * @param userReg - New data to be written into the user register
 * @return true   - On success
 * @return false  - On failure (lastError will be set)
 */
bool HTU21D::_writeUserRegister(uint8_t userReg) {
  _lastError = HTU21D_ERROR_NONE;                                               // Reset lastError

  _wire->beginTransmission(_i2cAddress);                                        // Start communication with the sensor
  _wire->write(HTU21D_CMD_WRITE_USER_REG);                                      // Issue the write user register command to the sensor
  _wire->write(userReg);                                                        // Write the new value into the user register

  // ---- Check if transmission was successfull ----
  if (_wire->endTransmission() != 0) {
    _lastError = HTU21D_ERROR_I2CWRITE;
    return false;
  }

  return true;
}

/**
 * @brief Sends a command to the sensor and reads the result
 * 
 * @param cmd     - The command to be sent to the sensor
 * @param data    - uint16_t pointer where to store the data
 * @param timeout - Max time in ms to wait for a response from the sensor
 * 
 * @return true   - On success
 * @return false  - On failure (lastError will be set)
 */
bool HTU21D::_readData(uint8_t cmd, uint16_t *data, uint8_t timeout) {          // Send a command to the sensor and read the result
  unsigned long start = 0;
  uint8_t buffer[2] = { 0x00 };
  uint8_t crc = 0;
  
  _lastError = HTU21D_ERROR_NONE;                                               // Clear the last error status

  _wire->beginTransmission(_i2cAddress);                                        // Start communication with the sensor
  _wire->write(cmd);                                                            // Write the command
  if (_wire->endTransmission() != 0) {                                          // Get the result from the sensor
    _lastError = HTU21D_ERROR_I2CWRITE;
    return false;
  }

  start = millis();                                                             // Store the start time of the measurement
  while((millis() - start) < timeout) {                                         // As long as we are within the timeout period
    delay(5);                                                                   // Give the sensor some time

    if (_wire->requestFrom((uint8_t) _i2cAddress, (uint8_t) 3) == 3){           // Check if 3 bytes are available
      buffer[0] = _wire->read();                                                // MSB
      buffer[1] = _wire->read();                                                // LSB
      crc = _wire->read();                                                      // CRC

      if ((_crc8((uint8_t *) &buffer, 2) != crc)) {                             // Check if CRC is valid
        _lastError = HTU21D_ERROR_CRC;
        return false;
      }
  
      *data = (uint16_t) buffer[0] << 8 | buffer[1];                            // Store the value
      return true;
    }
  }

  _lastError = HTU21D_ERROR_TIMEOUT;
  return false;
}

/**
 * @brief Calculate a checksum over the result from the sensor
 * 
 * @param ptr       - Pointer to the data buffer containing the result
 * @param size      - Number of bytes in the buffer
 * @return uint8_t  - The calculated CRC8 value for this buffer
 */
uint8_t HTU21D::_crc8(uint8_t *ptr, size_t size) {
  uint8_t crc = HTU21D_CRC_INIT;                                                // CRC base value

  // ---- Calculate the CRC value  ----
  while (size--) {
    crc ^= *ptr++;
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ HTU21D_CRC_POLYNOMINAL;
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}
