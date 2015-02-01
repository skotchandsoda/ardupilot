/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  Acuity AR2500 laser driver.
  Copyright 2015 Scott Cheloha.  All rights reserved.
*/

#include <AP_HAL.h>
#include <AP_Common.h>

#include "AP_Laser.h"

#include <cstring>

extern const AP_HAL::HAL& hal;

// Currently we only test on the Pixhawk/PX4.
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
// do something important ... what exactly, I don't know.
#endif

/*
  constructor
 */
AP_Laser_AR2500::AP_Laser_AR2500(AP_Laser &laser) :
    AP_Laser_Backend(laser),
    _instance(0),
    distance(0),
    signal_quality(0),
    internal_temperature(0),
    raw_binary_data(0),
{

  uint32_t baudrate = 19200;

  // Start our target serial port at the desired rate
  hal.uartE->begin(baudrate,128,128);

  // Set up autostart settings for sensor and force a reset
  // We want 19200 baud, with a measurement frequency of 10000,
  // averaging over 1000 measurements, output of distance/signal quality/temp
  // in the binary format, and distance tracking to be on.
  char *autostart_settings = "AS BR19200 MF10000 SA1000 SD2 3 DT\n";
  size_t as_len = strlen(autostart_settings);
  hal.uartE->write((uint8_t *) autostart_settings, as_len);

  // Force a device reset
  hal.uartE->write((uint8_t *) "DR\n", 3);

  // Wait a bit ... let the laser get back up and running
  usleep(500);

  // Register the laser with the frontend
  _instance = _frontend.register_sensor();
}

// Read data from the UART buffer
void AP_Laser_AR2500::accumulate(void)
{
  // Wait an indefinite length of time for the UART to be available
  while (!hal.uartE->available()) {
    usleep(100);
  }

  // Read in four bytes of raw data.  Might need to do some checking for terminators,
  // but right now just assume all of the bytes are at the correct offset for reading
  uint8_t *p = (uint8_t *) &raw_binary_data;
  for (int i = 0; i < 4; i++) {
    *(p + i) = hal.uartE->read();
  }

  // Convert the binary data to something we can use
  convert();
}


/*
  transfer data to the frontend
 */
void AP_Laser_AR2500::update(void)
{
  accumulate();
  _copy_to_frontend(_instance, distance, signal_quality, internal_temperature);
}

// // Send command to Read Pressure
// void AP_Baro_BMP085::Command_ReadPress()
// {
//     // Mode 0x34+(OVERSAMPLING << 6) is osrs=3 when OVERSAMPLING=3 => 25.5ms conversion time
//     hal.i2c->writeRegister(BMP085_ADDRESS, 0xF4,
//                            0x34+(OVERSAMPLING << 6));
//     _last_press_read_command_time = hal.scheduler->millis();
// }

// // Read Raw Pressure values
// bool AP_Baro_BMP085::ReadPress()
// {
//     uint8_t buf[3];

//     if (hal.i2c->readRegisters(BMP085_ADDRESS, 0xF6, 3, buf) != 0) {
//         _retry_time = hal.scheduler->millis() + 1000;
//         hal.i2c->setHighSpeed(false);
//         return false;
//     }

//     RawPress = (((uint32_t)buf[0] << 16)
//              | ((uint32_t)buf[1] << 8)
//              | ((uint32_t)buf[2])) >> (8 - OVERSAMPLING);
//     return true;
// }

// // Send Command to Read Temperature
// void AP_Baro_BMP085::Command_ReadTemp()
// {
//     hal.i2c->writeRegister(BMP085_ADDRESS, 0xF4, 0x2E);
//     _last_temp_read_command_time = hal.scheduler->millis();
// }

// // Read Raw Temperature values
// void AP_Baro_BMP085::ReadTemp()
// {
//     uint8_t buf[2];
//     int32_t _temp_sensor;

//     if (hal.i2c->readRegisters(BMP085_ADDRESS, 0xF6, 2, buf) != 0) {
//         hal.i2c->setHighSpeed(false);
//         return;
//     }
//     _temp_sensor = buf[0];
//     _temp_sensor = (_temp_sensor << 8) | buf[1];

//     RawTemp = _temp_sensor;
// }


// Convert raw binary data into real units for distance, signal quality, and sensor temperature.
// Conversion rules taken from Actuity AR2500 Laser Sensor User's Manual
void AP_Laser_AR2500::convert_raw_data()
{
  distance = (raw_binary_data >> 16) * 0.01;
  signal_quality = ((uint8_t) (raw_binary_data >> 8)) * 2;
  internal_temperature = ((int8_t) raw_binary_data) - 40;
}
