/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_LASER_BACKEND_H__
#define __AP_LASER_BACKEND_H__

#include "AP_Laser.h"

class AP_Laser_Backend
{
public:
    AP_Laser_Backend(AP_Laser &laser);

    // each driver must provide an update method to copy accumulated
    // data to the frontend
    virtual void update() = 0;

    // accumulate function. This is used for backends that don't use a
    // timer, and need to be called regularly by the main code to
    // trigger them to read the sensor
    virtual void accumulate(void) {}

protected:
    // reference to frontend object
    AP_Laser &_frontend;

    void _copy_to_frontend(uint8_t instance, float distance, uint8_t signal_quality, uint8_t internal_temperature);
};

#endif // __AP_LASER_BACKEND_H__
