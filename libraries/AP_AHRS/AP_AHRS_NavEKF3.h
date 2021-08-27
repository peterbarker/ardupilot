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
 *  EKF3-based AHRS (Attitude Heading Reference System) interface for
 *  ArduPilot
 *
 */

#include "AP_AHRS_NavEKF.h"

#if HAL_NAVEKF3_AVAILABLE

#include <AP_NavEKF3/AP_NavEKF3.h>

class AP_AHRS_NavEKF3 : public AP_AHRS_NavEKF {
public:

    using AP_AHRS_NavEKF::AP_AHRS_NavEKF;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_AHRS_NavEKF3);

    // reset the current gyro drift estimate
    //  should be called if gyro offsets are recalculated
    void reset_gyro_drift() override {
        EKF3.resetGyroBias();
    }

    // Methods
    bool initialised() const override {
        // initialisation complete 10sec after ekf has started
        return (_ekf3_started && (AP_HAL::millis() - start_time_ms > AP_AHRS_NAVEKF_SETTLE_TIME_MS));
    }

    void update() override;
    void get_results(Estimates &results) override;
    void reset() override {
        if (!_ekf3_started) {
            return;
        }
        _ekf3_started = EKF3.InitialiseFilter();
    }

    int8_t get_primary_core_index() const override {
        return EKF3.getPrimaryCoreIndex();
    }

    int8_t get_primary_core_IMU_index() const override {
        return EKF3.getPrimaryCoreIMUIndex();
    }

    // dead-reckoning support
    virtual bool get_position(struct Location &loc) const override {
        return EKF3.getLLH(loc);
    }

    // return a wind estimation vector, in m/s
    Vector3f wind_estimate() const override {
        Vector3f wind;
        EKF3.getWind(-1,wind);
        return wind;
    }

    // return an airspeed estimate if available. return true
    // if we have an estimate
    bool airspeed_estimate(uint8_t airspeed_index, float &airspeed_ret) const override;

    bool airspeed_vector_true(Vector3f &vec) const override {
        return EKF3.getAirSpdVec(-1, vec);
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

        EKF3.getVelNED(-1,vec);

        return vec.xy();
    }

    bool use_compass() override {
        return EKF3.use_compass();
    }

    // return the quaternion defining the rotation from NED to XYZ (body) axes
    bool get_quaternion(Quaternion &quat) const override WARN_IF_UNUSED {
        if (!_ekf3_started) {
            return false;
        }
        EKF3.getQuaternion(-1, quat);
        return true;
    }

    void estimate_wind(void);

    // is the AHRS subsystem healthy?
    bool healthy() const override;

    bool get_velocity_NED(Vector3f &vec) const override {
        EKF3.getVelNED(-1, vec);
        return true;
    }

    // Get a derivative of the vertical position in m/s which is kinematically consistent with the vertical position is required by some control loops.
    // This is different to the vertical velocity from the EKF which is not always consistent with the vertical position due to the various errors that are being corrected for.
    bool get_vert_pos_rate(float &velocity) const override {
        velocity = EKF3.getPosDownDerivative(-1);
        return true;
    }

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    // requires_position should be true if horizontal position configuration should be checked (not used)
    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const override;

    // relative-origin functions for fallback in AP_InertialNav
    bool set_origin(const Location &loc) override {
        return EKF3.setOriginLLH(loc);
    }
    bool get_origin(Location &ret) const override {
        return EKF3.getOriginLLH(-1,ret);
    }

    bool get_relative_position_NED_origin(Vector3f &vec) const override {
        Vector2f posNE;
        float posD;
        if (!EKF3.getPosNE(-1,posNE) || !EKF3.getPosD(-1,posD)) {
            return false;
        }
        // position is valid
        vec.x = posNE.x;
        vec.y = posNE.y;
        vec.z = posD;
        return true;
    }

    bool get_relative_position_NE_origin(Vector2f &posNE) const override {
        return EKF3.getPosNE(-1, posNE);
    }

    bool get_relative_position_D_origin(float &posD) const override {
        return EKF3.getPosD(-1,posD);
    }

    bool get_hagl(float &height) const override {
        return EKF3.getHAGL(height);
    }

    bool get_filter_status(nav_filter_status &status) const override {
        EKF3.getFilterStatus(-1,status);
        return true;
    }
    void get_filter_faults(int8_t core, uint16_t faults) const {
        EKF3.getFilterFaults(-1, faults);
    }

    void send_ekf_status_report(mavlink_channel_t chan) const override {
        EKF3.send_status_report(chan);
    }

    bool get_innovations(Vector3f &velInnov, Vector3f &posInnov, Vector3f &magInnov, float &tasInnov, float &yawInnov) const override {
        return EKF3.getInnovations(-1, velInnov, posInnov, magInnov, tasInnov, yawInnov);
    }

    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override {
        Vector2f offset;
        return EKF3.getVariances(-1, velVar, posVar, hgtVar, magVar, tasVar, offset);
    }

    bool started() const override { return _ekf3_started; }

    bool get_mag_field_NED(Vector3f &vec) const override {
        EKF3.getMagNED(-1,vec);
        return true;
    }

    bool get_mag_field_correction(Vector3f &vec) const override {
        EKF3.getMagXYZ(-1,vec);
        return true;
    }

    bool get_mag_offsets(uint8_t mag_idx, Vector3f &magOffsets) const override {
        return EKF3.getMagOffsets(mag_idx, magOffsets);
    }

    bool configured_to_use_gps_for_posxy() const {
        return EKF3.configuredToUseGPSForPosXY();
    }

    void  writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset) {
        EKF3.writeOptFlowMeas(rawFlowQuality, rawGyroRates, rawGyroRates, msecFlowMeas, posOffset);
    }
    void  writeBodyFrameOdom(float quality, const Vector3f &delPos, const Vector3f &delAng, float delTime, uint32_t timeStamp_ms, uint16_t delay_ms, const Vector3f &posOffset) {
        EKF3.writeBodyFrameOdom(quality, delPos, delAng, delTime, timeStamp_ms, delay_ms, posOffset);
    }

    void writeExtNavData(const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint16_t delay_ms, uint32_t resetTime_ms) {
        EKF3.writeExtNavData(pos, quat, posErr, angErr, timeStamp_ms, delay_ms, resetTime_ms);
    }

    void writeExtNavVelData(const Vector3f &vel, float err, uint32_t timeStamp_ms, uint16_t delay_ms) {
        return EKF3.writeExtNavVelData(vel, err, timeStamp_ms, delay_ms);
    }

    void getControlLimits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler) const {
        return EKF3.getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);
    }

    bool get_hgt_ctrl_limit(float& limit) const override {
        return EKF3.getHeightControlLimit(limit);
    }

    void writeDefaultAirSpeed(float airspeed, float uncertainty) {
        return EKF3.writeDefaultAirSpeed(airspeed, uncertainty);
    }

    void getCorrectedDeltaVelocityNED(Vector3f& ret, float& dt) const override;

    uint8_t activeCores() const override {
        return EKF3.activeCores();
    }

    void getQuaternionBodyToNED(int8_t instance, Quaternion &quat) const override {
        return EKF3.getQuaternionBodyToNED(instance, quat);
    }

    uint32_t getLastYawResetAngle(float &yawAng) override {
        return EKF3.getLastYawResetAngle(yawAng);
    };

    uint32_t getLastVelNorthEastReset(Vector2f &vel) const override {
        return EKF3.getLastVelNorthEastReset(vel);
    }

    uint32_t getLastPosDownReset(float &posDelta) override {
        return EKF3.getLastPosDownReset(posDelta);
    }

    void set_terrain_hgt_stable(bool stable) override {
        EKF3.setTerrainHgtStable(stable);
    }

    void check_lane_switch(void) override {
        return EKF3.checkLaneSwitch();
    }

    void request_yaw_reset(void) override {
        EKF3.requestYawReset();
    }

    void Log_Write(void) {
        EKF3.Log_Write();
    }

    bool is_ext_nav_used_for_yaw(void) const override {
        return EKF3.using_external_yaw();
    }

    bool using_external_yaw() const {
        return EKF3.using_external_yaw();
    }

    bool is_vibration_affected() const {
        return EKF3.isVibrationAffected(-1);
    }

    bool get_vel_innovations_and_variances_for_source(uint8_t source, Vector3f &innovations, Vector3f &variances) const override {
        return EKF3.getVelInnovationsAndVariancesForSource(-1, (AP_NavEKF_Source::SourceXY)source, innovations, variances);
    }

    uint8_t get_active_airspeed_index() const {
        return EKF3.getActiveAirspeed(get_primary_core_index());
    }

    void set_posvelyaw_source_set(uint8_t source_set_idx) {
        EKF3.setPosVelYawSourceSet(source_set_idx);
    }

    void set_baro_alt_noise(float noise) {
        EKF3.set_baro_alt_noise(noise);
    }

private:
    NavEKF3 EKF3;
    bool _ekf3_started;

    uint32_t start_time_ms;

    Matrix3f dcm_matrix;
};

#endif
