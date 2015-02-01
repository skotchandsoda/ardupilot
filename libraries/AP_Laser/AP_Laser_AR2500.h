/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AP_LASER_AR2500_H__
#define __AP_LASER_AR2500_H__

#include "AP_Laser.h"

class AP_Laser_AR2500 : public AP_Laser_Backend
{
public:
    // Constructor
    AP_Laser_AR2500(AP_Laser &laser);

    /* AP_Laser public interface: */
    void update(void);
    void accumulate(void);

private:
    // Which instance are we?
    uint8_t _instance;

    // Measured things from the sensor of interest to the frontend
    float   distance; // distance to target in meters
    uint8_t signal_quality; // signal quality of sensor on 0-255
    int8_t  internal_temperature; // internal sensor temperature in degrees celsius

    // Data read from serial in binary form (not useable until converted via convert())
    uint32_t raw_binary_data;

    // Various settings within the AR2500
    /* uint32_t baud_rate; // write/read rate for the AR2500's RS422 serial port */
    /* uint16_t measurement_frequency; // frequency of measurement for sensor in Hz */
    /* uint16_t averaging_value; // number of measurements taken before writing to serial */
    /* char[3] serial_data_format; // Output format of the AR2500 */

    // Command to copy the data from raw_binary_data into distance, signal_quality
    // and internal_temperature in correct units.
    void convert_raw_data();
};

#endif // __AP_LASER_AR2500_H__
