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


#include "SGP40.h"


//Constructor
SGP40::SGP40(){}

//Returns true if successful or false if no sensor detected
bool SGP40::init(i2c_inst_t *i2cInstance ){
  i2c = i2cInstance;
  
  VocAlgorithm_init(&vocAlgorithmParameters); //Initialize the VOC parameters
  
  SGP40ERR result = measureTest();
  return (result == SGP40_SUCCESS);
}

//Sensor runs on chip self test
//Returns SUCCESS (0) if successful or other error code if unsuccessful
SGP40ERR SGP40::measureTest(void) {

  int i2cResult = i2c_write_blocking(i2c, _SGP40Address, sgp40_measure_test, sizeof(sgp40_measure_test), false);
  if (i2cResult != sizeof(sgp40_measure_test)) {
    return SGP40_ERR_I2C_ERROR;
  }

  //Hang out while measurement is taken. v1.1 of the datasheet says 320ms
  sleep_ms(320);

  //Comes back in 3 bytes, data(MSB) / data(LSB) / Checksum
  uint8_t firstByte;
  int i2cReadResult = i2c_read_blocking(i2c, _SGP40Address, &firstByte, 1, false);
  if (i2cReadResult != 1) {
    return SGP40_ERR_I2C_ERROR; //Error out
  }

  if (firstByte != 0xD4) {
    return SGP40_MEASURE_TEST_FAIL; //self test results incorrect
  }

  return SGP40_SUCCESS;
}

//Perform a soft reset
//Returns SUCCESS (0) if successful
SGP40ERR SGP40::softReset(void) {
  int i2cResult = i2c_write_blocking(i2c, _SGP40Address, sgp40_soft_reset, sizeof(sgp40_soft_reset), false);
  if (i2cResult != sizeof(sgp40_soft_reset)){
    return SGP40_ERR_I2C_ERROR;
  }
  return SGP40_SUCCESS;
}

//Turn the heater off
//Returns SUCCESS (0) if successful
SGP40ERR SGP40::heaterOff(void) {
  int i2cResult = i2c_write_blocking(i2c, _SGP40Address, sgp40_heater_off, sizeof(sgp40_heater_off), false);

  sleep_ms(1); // From datasheet

  if (i2cResult != sizeof(sgp40_heater_off)){
    return SGP40_ERR_I2C_ERROR;
  }

  return SGP40_SUCCESS;
}

//Measure Raw Signal
//Returns SUCCESS (0) if successful
//The raw signal is returned in SRAW_ticks
//The user can provide Relative Humidity and Temperature parameters if desired:
//RH_ticks and T_ticks are 16 bit
//RH_ticks = %RH * 65535 / 100
//T_ticks = (DegC + 45) * 65535 / 175
//%RH: Minimum=0 (RH_ticks = 0x0000)  Maximum=100 (RH_ticks = 0xFFFF)
//DegC: Minimum=-45 (T_ticks = 0x0000)  Minimum=130 (T_ticks = 0xFFFF)
//Default %RH = 50 (RH_ticks = 0x8000)
//Default DegC = 25 (T_ticks = 0x6666)
//See the SGP40 datasheet for more details
SGP40ERR SGP40::measureRaw(uint16_t *SRAW_ticks, float RH, float T) {
  uint16_t RH_ticks, T_ticks;

  // Check RH and T are within bounds (probably redundant, but may prevent unexpected wrap-around errors)
  if (RH < 0) {
    RH = 0;
  }
  if (RH > 100) {
    RH = 100;
  }
  if (T < -45) {
    T = -45;
  }
  if (T > 130) {
    T = 130;
  }

  RH_ticks = (uint16_t)(RH * 65535 / 100); // Convert RH from %RH to ticks
  T_ticks = (uint16_t)((T + 45) * 65535 / 175); // Convert T from DegC to ticks
  
  uint8_t sgp40_measure_buffer[8];
  sgp40_measure_buffer[0] = sgp40_measure_raw[0];
  sgp40_measure_buffer[1] = sgp40_measure_raw[1];
  sgp40_measure_buffer[2] = RH_ticks >> 8;
  sgp40_measure_buffer[3] = RH_ticks & 0xFF;
  sgp40_measure_buffer[4] = _CRC8(RH_ticks);
  sgp40_measure_buffer[5] = T_ticks >> 8;
  sgp40_measure_buffer[6] = T_ticks & 0xFF;
  sgp40_measure_buffer[7] = _CRC8(T_ticks);

  int i2cResult = i2c_write_blocking(i2c, _SGP40Address, sgp40_measure_buffer, sizeof(sgp40_measure_buffer), false);
  if (i2cResult != sizeof(sgp40_measure_buffer)) {
    return SGP40_ERR_I2C_ERROR;
  }

  //Hang out while measurement is taken. datasheet says 30ms
  sleep_ms(30);

  //Comes back in 3 bytes, data(MSB) / data(LSB) / Checksum
  uint8_t resultBuffer[3];
  int i2cReadResult = i2c_read_blocking(i2c, _SGP40Address, resultBuffer, sizeof(resultBuffer), false);
  if (i2cReadResult != 3){
    return SGP40_ERR_I2C_ERROR; //Error out
  }

  uint16_t result = ((uint16_t)resultBuffer[0]) << 8; //store MSB in result
  result |= ((uint16_t)resultBuffer[1] & 0xFF); //store LSB in result

  *SRAW_ticks = result; //Pass the result

  return SGP40_SUCCESS;
}

//Get VOC Index
//Returns -100 if an error occurs
//The user can provide Relative Humidity and Temperature parameters if desired
int32_t SGP40::getVOCindex(float RH, float T){
  int32_t vocIndex;
  uint16_t SRAW_ticks;

  SGP40ERR result = measureRaw(&SRAW_ticks, RH, T);

  if (result != SGP40_SUCCESS) {
    return -100; //fail...
  }

  VocAlgorithm_process(&vocAlgorithmParameters, SRAW_ticks, &vocIndex);

  return vocIndex;
}

//Given an array and a number of bytes, this calculate CRC8 for those bytes
//CRC is only calc'd on the data portion (two bytes) of the four bytes being sent
//From: http://www.sunshine2k.de/articles/coding/crc/understanding_crc.html
//Tested with: http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
//x^8+x^5+x^4+1 = 0x31
uint8_t SGP40::_CRC8(uint16_t data) {
  uint8_t crc = 0xFF; //Init with 0xFF

  crc ^= (data >> 8); // XOR-in the first input byte

  for (uint8_t i = 0 ; i < 8 ; i++)
  {
    if ((crc & 0x80) != 0)
      crc = (uint8_t)((crc << 1) ^ 0x31);
    else
      crc <<= 1;
  }

  crc ^= (uint8_t)data; // XOR-in the last input byte

  for (uint8_t i = 0 ; i < 8 ; i++)
  {
    if ((crc & 0x80) != 0)
      crc = (uint8_t)((crc << 1) ^ 0x31);
    else
      crc <<= 1;
  }

  return crc; //No output reflection
}