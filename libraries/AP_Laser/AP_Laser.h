/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_LASER_H__
#define __AP_LASER_H__

#include <AP_HAL.h>
#include <AP_Param.h>
#include <Filter.h>
#include <DerivativeFilter.h>

// maximum number of sensor instances
#if HAL_CPU_CLASS == HAL_CPU_CLASS_16
#define LASER_MAX_INSTANCES 1
#else
#define LASER_MAX_INSTANCES 2
#endif

// maximum number of drivers. Note that a single driver can provide
// multiple sensor instances
#if HAL_CPU_CLASS == HAL_CPU_CLASS_16
#define LASER_MAX_DRIVERS 1
#else
#define LASER_MAX_DRIVERS 2
#endif

class AP_Laser_Backend;

class AP_Laser
{
    friend class AP_Laser_Backend;

public:
    // constructor
    AP_Laser();

    // initialise the laser object, loading backend drivers
    void init(void);

    // update the laser object, asking backends to push data to
    // the frontend
    void update(void);

    // healthy - returns true if sensor and derived altitude are good
    bool healthy(void)
        const { return healthy(_primary); }
    bool healthy(uint8_t instance)
        const { return sensors[instance].healthy
                    && sensors[instance].alt_ok
                    && sensors[instance].calibrated; }

    // check if all lasers are healthy - used for SYS_STATUS report
    bool all_healthy(void) const;

    // signal quality on 0-255
    uint8_t get_signal_quality(void)
        const {return get_signal_quality(_primary); }
    uint8_t get_signal_quality(uint8_t instance)
        const { return sensors[instance].signal_quality; }

    // sensor temperature in degrees C
    uint8_t get_sensor_temperature(void)
        const { return get_temperature(_primary); }
    uint8_t get_sensor_temperature(uint8_t instance)
        const { return sensors[instance].internal_temperature; }

    // accumulate a reading on sensors. Some backends without their
    // own thread or a timer may need this.
    void accumulate(void);

    // calibrate the laser.
    // This must be called on startup if the
    // altitude/climb_rate/acceleration interfaces are ever used
    void calibrate(void);

    // update the laser calibration.
    // Can be used for incremental preflight update of laser
    void update_calibration(void);

    // get current altitude in meters relative to altitude at the time
    // of the last calibrate() call
    float get_altitude(void)
        const { return get_altitude(_primary); }
    float get_altitude(uint8_t instance)
        const { return sensors[instance].distance; }

    // get current climb rate in m/s. A positive number means going up
    float get_climb_rate(void);

    // get last time sample was taken (in ms)
    uint32_t get_last_update(void) const { return get_last_update(_primary); }
    uint32_t get_last_update(uint8_t instance) const { return sensors[_primary].last_update_ms; }

    // settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    // Commented out because laser does not use internal temperature for calibration
    //   -- Scott (01/20/2015)
    //
    // float get_calibration_temperature(void) const { return get_calibration_temperature(_primary); }
    // float get_calibration_temperature(uint8_t instance) const;

    // HIL (and SITL) interface, setting altitude
    void setHIL(float altitude_msl);

    // Commented out because laser does not track pressure and temperature.
    //   -- Scott (01/20/2015)
    // // HIL (and SITL) interface, setting pressure and temperature
    // void setHIL(uint8_t instance, float pressure, float temperature);

    // register a new sensor, claiming a sensor slot. If we are out of
    // slots it will panic
    uint8_t register_sensor(void);

    // return number of registered sensors
    uint8_t num_instances(void)
        const { return _num_sensors; }

private:
    // how many drivers do we have?
    uint8_t _num_drivers;
    AP_Laser_Backend *drivers[LASER_MAX_DRIVERS];

    // how many sensors do we have?
    uint8_t _num_sensors;

    // what is the primary sensor at the moment?
    uint8_t _primary;

    struct sensor {
        uint32_t last_update_ms;        // last update time in ms
        bool healthy:1;                 // true if sensor is healthy
        bool alt_ok:1;                  // true if calculated altitude is ok
        bool calibrated:1;              // true if calibrated successfully

        float distance;                 // distance from sensor to target
        uint8_t signal_quality;         // reported sensor signal quality
        int8_t internal_temperature;    // internal sensor temperature in degrees C

    } sensors[LASER_MAX_INSTANCES];

    AP_Int8                             _alt_offset;
    // float                               _last_altitude_EAS2TAS;
    // float                               _EAS2TAS;
    //    uint8_t                             _external_temperature;
    //    uint8_t                             _last_external_temperature_ms;
    DerivativeFilterFloat_Size7         _climb_rate_filter;

    //    void SimpleAtmosphere(const float alt, float &sigma, float &delta, float &theta);
};

#include "AP_Laser_Backend.h"
#include "AP_Laser_AR2500.h"

#endif // __AP_LASER_H__
