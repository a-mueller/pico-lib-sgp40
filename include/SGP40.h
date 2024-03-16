/*
  This is a library written for the SPG40 Originally by Paul Clark @ SparkFun Electronics, December 5th, 2020
  "ported" to pico sdk AMU August 6th, 2023


  VOC Index Algorithm provided by Sensirion:
  https://github.com/Sensirion/embedded-sgp

  CRC lookup table from Bastian Molkenthin  http://www.sunshine2k.de/coding/javascript/crc/crc_js.html

  Copyright (c) 2015 Bastian Molkenthin, 2023 Abraham Mueller

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/


#ifndef SGP40_h
#define SGP40_h

#include "hardware/i2c.h"
#include <stdint.h>
#include <stdio.h>

extern "C" {
#include "sensirion_arch_config.h"
#include "sensirion_voc_algorithm.h"
};


typedef enum {
  SGP40_SUCCESS = 0,
  SGP40_ERR_BAD_CRC,
  SGP40_ERR_I2C_ERROR,
  SGP40_MEASURE_TEST_FAIL
} SGP40ERR;

//SGP40 I2C Commands
const uint8_t sgp40_measure_raw[2] = {0x26, 0x0F};
const uint8_t sgp40_measure_test[2] = {0x28, 0x0E};
const uint8_t sgp40_heater_off[2] = {0x36, 0x15};
const uint8_t sgp40_soft_reset[2] = {0x00, 0x06};
const uint8_t sgp40_read_serial[2] = {0x36, 0x82};
//That's all folks!

//SGP40 Measure Test Results
const uint16_t sgp40_measure_test_pass = 0xD400;
const uint16_t sgp40_measure_test_fail = 0x4B00;

class SGP40
{
    // user-accessible "public" interface
  public:

    //default constructor
    SGP40();

    //Start I2C communication using specified port
    bool init(i2c_inst_t *i2cInstance); //If user doesn't specify then Wire will be used

    //Sensor runs on chip self test
    //Returns SUCCESS (0) if successful or other error code if unsuccessful
    SGP40ERR measureTest(void);

    //Perform a soft reset
    //Returns SUCCESS (0) if successful
    SGP40ERR softReset(void);

    //Turn the heater off
    //Returns SUCCESS (0) if successful
    SGP40ERR heaterOff(void);

    //Measure Raw Signal
    //Returns SUCCESS (0) if successful
    //The raw signal is returned in SRAW_ticks
    //The user can provide Relative Humidity and Temperature parameters if desired
    //See the SGP40 datasheet for more details
    SGP40ERR measureRaw(uint16_t *SRAW_ticks, float RH = 50, float T = 25);

    //Get VOC Index
    //Returns -100 if an error occurs
    //The user can provide Relative Humidity and Temperature parameters if desired
    int32_t getVOCindex(float RH = 50, float T = 25);

  private:
    //This stores the requested i2c port
    i2c_inst_t *i2c;

    //SGP40's I2C address
    const uint8_t _SGP40Address = 0x59;

    //Storage for the Sensirion VOC Algorithm parameters
    VocAlgorithmParams vocAlgorithmParameters;

    //Generates CRC8 for SGP40 from lookup table or polynomial (if SGP40_LOOKUP_TABLE is undefined)
    uint8_t _CRC8(uint16_t twoBytes);

};

#endif
