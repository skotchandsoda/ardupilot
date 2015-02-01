/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Laser.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_Laser_Backend::AP_Laser_Backend(AP_Laser &laser) :
    _frontend(laser)
{}

/*
  copy latest data to the frontend from a backend
 */
void AP_Laser_Backend::_copy_to_frontend(uint8_t instance, float distance, uint8_t signal_quality, uint8_t internal_temperature)
{
    // Cannot copy to nonexistant instance
    if (instance >= _frontend._num_sensors) {
        return;
    }

    // Provided the instance exists, copy the data as requested
    _frontend.sensors[instance].distance = distance;
    _frontend.sensors[instance].signal_quality = signal_quality;
    _frontend.sensors[instance].internal_temperature = internal_temperature;
    _frontend.sensors[instance].last_update_ms = hal.scheduler->millis();
}
