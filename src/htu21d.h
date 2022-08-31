#ifndef __SENSENET_HTU21D__
#define __SENSENET_HTU21D__

#include <Arduino.h>
#include <Wire.h>

// ---- Sensor Settings ----
#define HTU21D_I2C_ADDRESS                      0x40                            // Address of this sensor
#define HTU21D_CRC_INIT                         0x00                            // Init value for the CRC check
#define HTU21D_CRC_POLYNOMINAL                  0x31                            // Seed for the CRC polynominal
#define HTU21D_INIT_DELAY                         15                            // Sensor needs 1 sec. init time
#define HTU21D_CMD_SIZE                            1                            // Commands consist of 2 bytes
#define HTU21D_TIMEOUT                            75                            // Wait for max 75ms for a measurement to complete

// ---- HTU21D Commands ----
#define HTU21D_CMD_MEASURE_TEMP_HM              0xE3                            // Start a temperature measurement and hold master untill ready
#define HTU21D_CMD_MEASURE_HUMID_HM             0xE5                            // Start a humidity measurement and hold master untill ready
#define HTU21D_CMD_MEASURE_TEMP                 0xF3                            // Start a temperature measurement
#define HTU21D_CMD_MEASURE_HUMID                0xF5                            // Start a humidity measurement
#define HTU21D_CMD_WRITE_USER_REG               0xE6                            // Write the user register
#define HTU21D_CMD_READ_USER_REG                0xE7                            // Read the user register
#define HTU21D_CMD_SOFT_RESET                   0xFE                            // Soft reset the sensor

// ---- HTU21D Configuration ----
#define HTU21D_RES_12_14                        0x00                            // Humid 12-bit, Temp 14-bit
#define HTU21D_RES_08_12                        0x01                            // Humid 8-bit, Temp 12-bit
#define HTU21D_RES_10_13                        0x80                            // Humid 10-bit, Temp 13-bit
#define HTU21D_RES_11_11                        0x81                            // Humid 11-bit, Temp 11-bit
#define HTU21D_HEATER                           0x04                            // Enable the on-chip heater
#define HTU21D_OTP                              0x02                            // ‘0’ loads default settings after each time a measurement command is issued.

// ---- HTU21D Status ----
#define HTU21D_VDD_LEVEL                        0x40                            // ‘0’: VDD>2.25V, ‘1’: VDD<2.25V

// ---- Temperature Scales ----
#define _CELCIUS                                0x01                            // Use Celcius scale for temperature values
#define _FAHRENHEIT                             0x02                            // Use Farenheit scale for temperature values
#define _KELVIN                                 0x03                            // Use Kelvin scale for temperature valuess

// ---- HTU21D Error States ----
#define HTU21D_ERROR_NONE                       0x00                            // No error
#define HTU21D_ERROR_NOSENSOR                   0x01                            // Sensor not found
#define HTU21D_ERROR_CRC                        0x02                            // CRC error
#define HTU21D_ERROR_WAIT                       0x03                            // Measurement not ready
#define HTU21D_ERROR_I2CWRITE                   0x04                            // I2C Write error
#define HTU21D_ERROR_I2CREAD                    0x05                            // I2C Read error
#define HTU21D_ERROR_TIMEOUT                    0x06                            // Timout reading from sensor
#define HTU21D_ERROR_VALUE_INVALID              0x07                            // The value is invalid, perform a measurement first

class HTU21D {
  public:
    HTU21D(TwoWire *wire = &Wire);                                              // Constructor of the SCD4x class
    #if defined (ESP8266) || defined (ESP32)
      bool begin(uint8_t i2cAddress = HTU21D_I2C_ADDRESS, uint8_t sda = 0xff, uint8_t scl = 0xff);
    #else
      bool begin(uint8_t i2cAddress = HTU21D_I2C_ADDRESS);                      // Starts the sensor
    #endif

    // ---- Configuration functions ----
    bool setResolution(uint8_t resolution);                                     // Sets the measurement resolution
    bool enableHeater(bool state);                                              // Enable or Disable the sensor heater
    bool heaterEnabled(void);                                                   // Get the current status of the sensor heater
    bool enableOTP(bool state);                                                 // Enable or Disable OTP
    bool otpEnabled(void);                                                      // Get the current OTP status
    bool vddLow(void);                                                          // Check if vdd is below 2.25V

    // ---- Measurement functions ----
    float getTemperature(uint8_t scale = _CELCIUS);                             // Reads the stored temperature or gets it from te sensor
    float getHumidity(void);                                                    // Reads the stored temperature of gets it from the sensor
    bool readSensorData(void);                                                  // Initiates a temperature and humidity measurement and stores the results
    
    // ---- Utility functions ----
    bool softReset(void);                                                       // Reinitializes the sensor
    uint8_t getLastError(void);                                                 // Returns the code of the last error that occured
  
  private:
    TwoWire *_wire = nullptr;                                                   // Handle to I2C class
    uint8_t _i2cAddress = 0;                                                    // Sensor address
    uint8_t _lastError = HTU21D_ERROR_NONE;                                     // The last error that has occured
    float _temperature = 0.0, _humidity = 0.0;                                  // Variables for Temperature and Humidity
    bool _temp_valid = false, _humid_valid = false;                             // Keeps track of if stored values are still valid

    bool _readUserRegister(uint8_t *userReg);                                   // Read the value of the user register
    bool _writeUserRegister(uint8_t userReg);                                   // Write new value into the user register
    bool _readData(uint8_t cmd, uint16_t *data, uint8_t timeout);               // Read measurements from the sensor
    uint8_t _crc8(uint8_t *ptr, size_t size);                                   // Calculate the checksum over the data
};

#endif // __SENSENET_HTU21D__