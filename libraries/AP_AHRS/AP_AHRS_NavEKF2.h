#pragma once

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  EKF2-based AHRS (Attitude Heading Reference System) interface for
 *  ArduPilot
 *
 */

#include "AP_AHRS_NavEKF.h"

#if HAL_NAVEKF2_AVAILABLE

#include <AP_NavEKF2/AP_NavEKF2.h>

class AP_AHRS_NavEKF2 : public AP_AHRS_NavEKF {
public:

    using AP_AHRS_NavEKF::AP_AHRS_NavEKF;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_AHRS_NavEKF2);

    // reset the current gyro drift estimate
    //  should be called if gyro offsets are recalculated
    void reset_gyro_drift() override {
        EKF2.resetGyroBias();
    }

    // Methods
    bool initialised() const override {
        // initialisation complete 10sec after ekf has started
        return (_ekf2_started && (AP_HAL::millis() - start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS));
    }
    void update() override;
    void get_results(Estimates &results) override;
    void reset() override {
        if (!_ekf2_started) {
            return;
        }
        _ekf2_started = EKF2.InitialiseFilter();
    }

    int8_t get_primary_core_index() const override {
        return EKF2.getPrimaryCoreIndex();
    }

    int8_t get_primary_core_IMU_index() const override {
        return EKF2.getPrimaryCoreIMUIndex();
    }

    // dead-reckoning support
    virtual bool get_position(struct Location &loc) const override {
        return EKF2.getLLH(loc);
    }

    // return a wind estimation vector, in m/s
    Vector3f wind_estimate() const override {
        Vector3f wind;
        EKF2.getWind(-1,wind);
        return wind;
    }

    bool airspeed_vector_true(Vector3f &vec) const override {
        return EKF2.getAirSpdVec(-1, vec);
    }

    // return a synthetic airspeed estimate (one derived from sensors
    // other than an actual airspeed sensor), if available. return
    // true if we have a synthetic airspeed.  ret will not be modified
    // on failure.
    bool synthetic_airspeed(float &ret) const override WARN_IF_UNUSED {
        return false;
    }

    // return a ground vector estimate in meters/second, in North/East order
    Vector2f groundspeed_vector() override {
        Vector3f vec;

        EKF2.getVelNED(-1,vec);

        return vec.xy();
    }

    bool use_compass() override {
        return EKF2.use_compass();
    }

    // return the quaternion defining the rotation from NED to XYZ (body) axes
    bool get_quaternion(Quaternion &quat) const override WARN_IF_UNUSED {
        if (!_ekf2_started) {
            return false;
        }
        EKF2.getQuaternion(-1, quat);
        return true;
    }

    void estimate_wind(void);

    // is the AHRS subsystem healthy?
    bool healthy() const override;

    bool get_velocity_NED(Vector3f &vec) const override {
        EKF2.getVelNED(-1, vec);
        return true;
    }

    // Get a derivative of the vertical position in m/s which is kinematically consistent with the vertical position is required by some control loops.
    // This is different to the vertical velocity from the EKF which is not always consistent with the vertical position due to the various errors that are being corrected for.
    bool get_vert_pos_rate(float &velocity) const override {
        velocity = EKF2.getPosDownDerivative(-1);
        return true;
    }

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    // requires_position should be true if horizontal position configuration should be checked (not used)
    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const override;

    // relative-origin functions for fallback in AP_InertialNav
    bool set_origin(const Location &loc) override {
        return EKF2.setOriginLLH(loc);
    }
    bool get_origin(Location &ret) const override {
        return EKF2.getOriginLLH(-1,ret);
    }
    bool get_relative_position_NED_origin(Vector3f &vec) const override {
        Vector2f posNE;
        float posD;
        if (!EKF2.getPosNE(-1,posNE) || !EKF2.getPosD(-1,posD)) {
            return false;
        }
        // position is valid
        vec.x = posNE.x;
        vec.y = posNE.y;
        vec.z = posD;
        return true;
    }

    bool get_relative_position_NE_origin(Vector2f &posNE) const override {
        return EKF2.getPosNE(-1, posNE);
    }

    bool get_relative_position_D_origin(float &posD) const override {
        return EKF2.getPosD(-1,posD);
    }

    bool get_hagl(float &height) const override {
        return EKF2.getHAGL(height);
    }

    bool get_filter_status(nav_filter_status &status) const override {
        EKF2.getFilterStatus(-1,status);
        return true;
    };
    void get_filter_faults(int8_t core, uint16_t faults) const {
        EKF2.getFilterFaults(-1, faults);
    }

    bool get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const override {
        return EKF2.getInnovations(-1, velInnov, posInnov, magInnov, tasInnov, yawInnov);
    }

    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override {
        Vector2f offset;
        return EKF2.getVariances(-1, velVar, posVar, hgtVar, magVar, tasVar, offset);
    }

    void send_ekf_status_report(mavlink_channel_t chan) const override {
        EKF2.send_status_report(chan);
    }

    bool started() const override { return _ekf2_started; }

    bool get_mag_field_NED(Vector3f &vec) const override {
        EKF2.getMagNED(-1,vec);
        return true;
    }

    bool get_mag_field_correction(Vector3f &vec) const override {
        EKF2.getMagXYZ(-1,vec);
        return true;
    }

    bool get_mag_offsets(uint8_t mag_idx, Vector3f &magOffsets) const override {
        return EKF2.getMagOffsets(mag_idx, magOffsets);
    }

    bool configured_to_use_gps_for_posxy() const {
        return EKF2.configuredToUseGPSForPosXY();
    }

    void  writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset) {
        EKF2.writeOptFlowMeas(rawFlowQuality, rawGyroRates, rawGyroRates, msecFlowMeas, posOffset);
    }

    void writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms) {
        EKF2.writeExtNavData(pos, quat, posErr, angErr, timeStamp_ms, delay_ms, resetTime_ms);
    }

    void writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms) {
        return EKF2.writeExtNavVelData(vel, err, timeStamp_ms, delay_ms);
    }

    void getControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const {
        return EKF2.getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);
    }

    bool get_hgt_ctrl_limit(float& limit) const override {
        return EKF2.getHeightControlLimit(limit);
    }

    void writeDefaultAirSpeed(float airspeed) {
        return EKF2.writeDefaultAirSpeed(airspeed);
    }

    void getCorrectedDeltaVelocityNED(Vector3f& ret, float& dt) const override;

    uint8_t activeCores() const override {
        return EKF2.activeCores();
    }

    void getQuaternionBodyToNED(int8_t instance, Quaternion &quat) const override {
        return EKF2.getQuaternionBodyToNED(instance, quat);
    }

    uint32_t getLastYawResetAngle(float &yawAng) override {
        return EKF2.getLastYawResetAngle(yawAng);
    };

    // return the amount of NE position change in metres due to the last reset
    // returns the time of the last reset or 0 if no reset has ever occurred
    uint32_t getLastPosNorthEastReset(Vector2f &pos) override WARN_IF_UNUSED {
        return EKF2.getLastPosNorthEastReset(pos);
    };

    uint32_t getLastVelNorthEastReset(Vector2f &vel) const override {
        return EKF2.getLastVelNorthEastReset(vel);
    }

    uint32_t getLastPosDownReset(float &posDelta) override {
        return EKF2.getLastPosDownReset(posDelta);
    }

    void set_terrain_hgt_stable(bool stable) override {
        EKF2.setTerrainHgtStable(stable);
    }

    void check_lane_switch(void) override {
        return EKF2.checkLaneSwitch();
    }

    void request_yaw_reset(void) override {
        EKF2.requestYawReset();
    }

    void Log_Write(void) {
        EKF2.Log_Write();
    }

    bool is_ext_nav_used_for_yaw(void) const override {
        return EKF2.isExtNavUsedForYaw();
    }

    void set_baro_alt_noise(float noise) {
        EKF2.set_baro_alt_noise(noise);
    }

    // this is only out here for parameter purposes; it should not be
    // accessed directly
    NavEKF2 EKF2;

private:
    bool _ekf2_started;

    uint32_t start_time_ms;

    Matrix3f dcm_matrix;
};

#endif
