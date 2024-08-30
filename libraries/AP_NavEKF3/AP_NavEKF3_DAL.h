#pragma once

#include "AP_NavEKF3_feature.h"

#if AP_NAVEKF3_DAL_ENABLED

#include <AP_DAL/AP_DAL.h>

/*
 * Wraps AP_DAL, modifying data read by NavEKF3
 */

class AP_NavEKF3_DAL {
public:

    AP_NavEKF3_DAL(AP_DAL &_dal) :
        dal{_dal} { }

    void start_frame(AP_DAL::FrameType frametype) { return dal.start_frame(frametype); }
    void end_frame(void) { return dal.end_frame(); };
    uint64_t micros64() const { return dal.micros64(); }
    uint32_t millis() const { return dal.millis(); }

    void log_event3(AP_DAL::Event event) { dal.log_event3(event); }
    void log_SetOriginLLH3(const Location &loc) { dal.log_SetOriginLLH3(loc); }
    void log_SetLatLng(const Location &loc, float posAccuracy, uint32_t timestamp_ms) { return dal.log_SetLatLng(loc, posAccuracy, timestamp_ms); }

    void log_writeDefaultAirSpeed3(const float aspeed, const float uncertainty);
    void log_writeEulerYawAngle(float yawAngle, float yawAngleErr, uint32_t timeStamp_ms, uint8_t type) { dal.log_writeEulerYawAngle(yawAngle, yawAngleErr, timeStamp_ms, type); }

    float get_EAS2TAS(void) const { return dal.get_EAS2TAS(); }
    bool airspeed_sensor_enabled(void) const { return dal.airspeed_sensor_enabled(); }

    void *malloc_type(size_t size, enum AP_DAL::Memory_Type mem_type) const {
        return dal.malloc_type(size, mem_type);
    }

    AP_DAL_InertialSensor &ins() { return dal.ins(); }

    AP_DAL_Baro &baro() { return dal.baro(); }

#if AP_GPS_ENABLED
    AP_DAL_GPS &gps() {
#if AP_NAVEKF3_DISABLE_GPS_ENABLED
        if (gps_disabled) {
            return disabled_gps;
        }
#endif
        return dal.gps();
    }
#endif

#if AP_RANGEFINDER_ENABLED
    AP_DAL_RangeFinder *rangefinder() { return dal.rangefinder(); }
#endif

    AP_DAL_Airspeed *airspeed() { return dal.airspeed(); }

#if AP_BEACON_ENABLED
    AP_DAL_Beacon *beacon() { return dal.beacon(); }
#endif

#if HAL_VISUALODOM_ENABLED
    AP_DAL_VisualOdom *visualodom() { return dal.visualodom(); }
#endif

    AP_DAL_Compass &compass() { return dal.compass(); }

    AP_DAL::VehicleClass get_vehicle_class(void) const { return dal.get_vehicle_class(); }

    bool get_fly_forward(void) const { return dal.get_fly_forward(); }

    bool get_takeoff_expected(void) const { return dal.get_takeoff_expected(); }
    bool get_touchdown_expected(void) const { return dal.get_touchdown_expected(); }

    // for EKF usage to enable takeoff expected to true
    void set_takeoff_expected() { return dal.set_takeoff_expected(); }

    const Vector3f &get_trim() const { return dal.get_trim(); }

    const Matrix3f &get_rotation_vehicle_body_to_autopilot_body(void) const {
        return dal.get_rotation_vehicle_body_to_autopilot_body();
    }

    const class Location &get_home(void) const { return dal.get_home(); }

    uint32_t get_time_flying_ms(void) const { return dal.get_time_flying_ms(); }

    bool ekf_low_time_remaining(AP_DAL::EKFType etype, uint8_t core) { return dal.ekf_low_time_remaining(etype, core); }

    bool get_armed() const { return dal.get_armed(); }

    uint32_t available_memory() const { return dal.available_memory(); }

    bool opticalflow_enabled(void) const { return dal.opticalflow_enabled(); }

    bool wheelencoder_enabled(void) const { return dal.wheelencoder_enabled(); }

    void writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset, float heightOverride) { return dal.writeOptFlowMeas(rawFlowQuality, rawFlowRates, rawGyroRates, msecFlowMeas, posOffset, heightOverride); }

    // log external nav data
    void writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms) { return dal.writeExtNavData(pos, quat, posErr, angErr, timeStamp_ms, delay_ms, resetTime_ms); }
    void writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms) { dal.writeExtNavVelData(vel, err, timeStamp_ms, delay_ms); }

    // log wheel odometry data
    void writeWheelOdom(float delAng, float delTime, uint32_t timeStamp_ms, const Vector3f &posOffset, float radius) { dal.writeWheelOdom(delAng, delTime, timeStamp_ms, posOffset, radius); }
    void writeBodyFrameOdom(float quality, const Vector3f &delPos, const Vector3f &delAng, float delTime, uint32_t timeStamp_ms, uint16_t delay_ms, const Vector3f &posOffset) { dal.writeBodyFrameOdom(quality, delPos, delAng, delTime, timeStamp_ms, delay_ms, posOffset); }

    int snprintf(char* str, size_t size, const char *format, ...) const;

#if AP_NAVEKF3_DISABLE_GPS_ENABLED
    // Start methods just for the AP_NavEKF3_DAL:
    void force_gps_disable(bool gps_disable) {
        dal.log_event3(gps_disable?AP_DAL::Event::EK3GPSDisable:AP_DAL::Event::EK3GPSEnable);
        gps_disabled = gps_disable;
    }
#endif

private:
    AP_DAL &dal;

#if AP_NAVEKF3_DISABLE_GPS_ENABLED
    // an DAL GPS object which returns nothing:
    AP_DAL_GPS disabled_gps;
    bool gps_disabled;  // trueif we should ise disabled_gps
#endif
};
#endif  // AP_NAVEKF3_DAL_ENABLED
