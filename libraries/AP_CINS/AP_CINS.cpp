/*
  CINS state estimator for AP_AHRS, developed by Mr Patrick Wiltshire and Dr Pieter Van Goor 
 */

#include "AP_CINS.h"
#include <AP_DAL/AP_DAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

// gains tested for 5Hz GPS
#define CINS_GAIN_GPSPOS_ATT (1.0E-5)
#define CINS_GAIN_GPSVEL_ATT (1.0E-4)

// Gains for new CINS method
#define CINS_GAIN_GPSPOS_POS (10.0)
#define CINS_GAIN_GPSVEL_VEL (10.0)
#define CINS_GAIN_MAG (0.0)//(0.5)
#define CINS_GAIN_Q11 (0.001)
#define CINS_GAIN_Q22 (0.001)
// Gains for the Gyro and Accel Biases
#define CINS_GAIN_POS_GYR_BIAS (1E-7)
#define CINS_GAIN_VEL_GYR_BIAS (1E-6)
#define CINS_GAIN_MAG_GYR_BIAS (0.0)//(0.005)
#define CINS_SAT_GYR_BIAS (0.02) // 0.0873 rad/s is 5 deg/s
#define CINS_GAIN_POS_ACC_BIAS (1E-3)
#define CINS_GAIN_VEL_ACC_BIAS (1E-3)
#define CINS_GAIN_MAG_ACC_BIAS (0.0)//(0.005)
#define CINS_SAT_ACC_BIAS (0.5)
// Naive GPS Time Delay settings
#define CINS_GPS_DELAY (0.05) // seconds of delay

// table of user settable parameters
const AP_Param::GroupInfo AP_CINS::var_info[] = {

    // @Param: GPATT
    // @DisplayName: CINS GPS position attitude gain
    // @Description: CINS GPS position attitude gain
    // @Range: 0.0001 0.001
    // @User: Advanced
    AP_GROUPINFO("GPATT", 1, AP_CINS, gains.gpspos_att, CINS_GAIN_GPSPOS_ATT),

    // @Param: GVATT
    // @DisplayName: CINS GPS velocity attitude gain
    // @Description: CINS GPS velocity attitude gain
    // @Range: 0.0001 0.001
    // @User: Advanced
    AP_GROUPINFO("GVATT", 2, AP_CINS, gains.gpsvel_att, CINS_GAIN_GPSVEL_ATT),

    // @Param: GPLAG
    // @DisplayName: CINS GPS lag
    // @Description: CINS GPS lag
    // @Range: 0.0 0.2
    // @User: Advanced
    // @Units: s
    AP_GROUPINFO("GPLAG", 3, AP_CINS, gains.gps_lag, CINS_GPS_DELAY),

    // @Param: GPPOS
    // @DisplayName: CINS GPS position gain
    // @Description: CINS GPS position gain
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("GPPOS", 4, AP_CINS, gains.gps_pos, CINS_GAIN_GPSPOS_POS),

    // @Param: GPVEL
    // @DisplayName: CINS GPS velocity gain
    // @Description: CINS GPS velocity gain
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("GPVEL", 5, AP_CINS, gains.gps_vel, CINS_GAIN_GPSVEL_VEL),

    // @Param: MGATT
    // @DisplayName: CINS magnetometer attitude gain
    // @Description: CINS magnetometer (compass) attitude gain
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("MGATT", 6, AP_CINS, gains.mag_att, CINS_GAIN_MAG),
    
    // @Param: INTQV
    // @DisplayName: CINS internal velocity Q gain
    // @Description: CINS internal velocity Q gain
    // @Range: 0.0001 0.1
    // @User: Advanced
    AP_GROUPINFO("INTQV", 7, AP_CINS, gains.Q11, CINS_GAIN_Q11),

    // @Param: INTQP
    // @DisplayName: CINS internal position Q gain
    // @Description: CINS internal position Q gain
    // @Range: 0.0001 0.1
    // @User: Advanced
    AP_GROUPINFO("INTQP", 8, AP_CINS, gains.Q22, CINS_GAIN_Q22),

    // @Param: GPPGB
    // @DisplayName: CINS GPS position - gyro bias gain
    // @Description: CINS GPS position - gyro bias gain
    // @Range: 0.0000001 0.000001
    // @User: Advanced
    AP_GROUPINFO("GPPGB", 9, AP_CINS, gains.gps_pos_gyr_bias, CINS_GAIN_POS_GYR_BIAS),
    
    // @Param: GPVGB
    // @DisplayName: CINS GPS velocity - gyro bias gain
    // @Description: CINS GPS velocity - gyro bias gain
    // @Range: 0.0000001 0.000001
    // @User: Advanced
    AP_GROUPINFO("GPVGB", 10, AP_CINS, gains.gps_vel_gyr_bias, CINS_GAIN_VEL_GYR_BIAS),

    // @Param: MAGGB
    // @DisplayName: CINS magnetometer - gyro bias gain
    // @Description: CINS magnetometer (compass) gyro bias gain
    // @Range: 0.001 0.0001
    // @User: Advanced
    AP_GROUPINFO("MAGGB", 11, AP_CINS, gains.mag_gyr_bias, CINS_GAIN_MAG_GYR_BIAS),

    // @Param: SATGB
    // @DisplayName: CINS gyro bias saturation
    // @Description: CINS gyro bias saturation
    // @Range: 0.05 0.1
    // @User: Advanced
    AP_GROUPINFO("SATGB", 12, AP_CINS, gains.sat_gyr_bias, CINS_SAT_GYR_BIAS),


    // @Param: GPPAB
    // @DisplayName: CINS GPS position - accel. bias gain
    // @Description: CINS GPS position - accel. bias gain
    // @Range: 0.0000001 0.000001
    // @User: Advanced
    AP_GROUPINFO("GPPAB", 13, AP_CINS, gains.gps_pos_acc_bias, CINS_GAIN_POS_ACC_BIAS),
    
    // @Param: GPVAB
    // @DisplayName: CINS GPS velocity - accel. bias gain
    // @Description: CINS GPS velocity - accel. bias gain
    // @Range: 0.0000001 0.000001
    // @User: Advanced
    AP_GROUPINFO("GPVAB", 14, AP_CINS, gains.gps_vel_acc_bias, CINS_GAIN_VEL_ACC_BIAS),

    // @Param: MAGAB
    // @DisplayName: CINS magnetometer - accel. bias gain
    // @Description: CINS magnetometer (compass) accel. bias gain
    // @Range: 0.001 0.0001
    // @User: Advanced
    AP_GROUPINFO("MAGAB", 15, AP_CINS, gains.mag_acc_bias, CINS_GAIN_MAG_ACC_BIAS),

    // @Param: SATAB
    // @DisplayName: CINS accel. bias saturation
    // @Description: CINS accel. bias saturation
    // @Range: 0.05 0.1
    // @User: Advanced
    AP_GROUPINFO("SATAB", 16, AP_CINS, gains.sat_acc_bias, CINS_SAT_ACC_BIAS),
    
    AP_GROUPEND
};

Vector3F computeRotationCorrection(const Vector3F& v1, const Vector3F& v2, const ftype& gain, const ftype& dt);

// constructor
AP_CINS::AP_CINS(void)
{
    ASSERT_STORAGE_SIZE(SIM23, 152);
    ASSERT_STORAGE_SIZE(GL2, 32);

    AP_Param::setup_object_defaults(this, var_info);
}

/*
  initialise the filter
 */
void AP_CINS::init(void)
{
    //Initialise XHat and ZHat as stationary at the origin
    state.XHat = Gal3F::identity();
    state.ZHat = SIM23::identity();

    state.gyr_bias.zero();
    state.gyr_bias_gain_mat.rot.zero();
    state.gyr_bias_gain_mat.vel.zero();
    state.gyr_bias_gain_mat.pos.zero();

    state.acc_bias.zero();
    state.acc_bias_gain_mat.rot.zero();
    state.acc_bias_gain_mat.vel.zero();
    state.acc_bias_gain_mat.pos.zero();

    delayer.YR = Gal3F::identity();
    delayer.YR_stepper = Gal3F::identity();
}
/*
  update function, called at loop rate
 */
void AP_CINS::update(void)
{
    auto &dal = AP::dal();

    if (!done_yaw_init) {
        done_yaw_init = init_yaw();
    }

    const auto &ins = dal.ins();

    // Get delta angle and convert to gyro rad/s
    const uint8_t gyro_index = ins.get_first_usable_gyro();
    Vector3f delta_angle;
    float dangle_dt;
    if (!ins.get_delta_angle(gyro_index, delta_angle, dangle_dt) || dangle_dt <= 0) {
        // can't update, no delta angle
        return;
    }
    // turn delta angle into a gyro in radians/sec
    const Vector3F gyro = (delta_angle / dangle_dt).toftype();

    // get delta velocity and convert to accel m/s/s
    Vector3f delta_velocity;
    float dvel_dt;
    if (!ins.get_delta_velocity(gyro_index, delta_velocity, dvel_dt) || dvel_dt <= 0) {
        // can't update, no delta velocity
        return;
    }
    // turn delta velocity into a accel vector
    const Vector3F accel = (delta_velocity / dvel_dt).toftype();

    if (done_yaw_init && state.have_origin){
        update_imu(gyro, accel, dangle_dt);
    }

    // see if we have new GPS data
    const auto &gps = dal.gps();
    if (gps.status() >= AP_DAL_GPS::GPS_OK_FIX_3D) {
        const uint32_t last_gps_fix_ms = gps.last_message_time_ms(0);
        if (last_gps_update_ms != last_gps_fix_ms) {
            // don't allow for large gain if we lose and regain GPS
            const float gps_dt = MIN((last_gps_fix_ms - last_gps_update_ms)*0.001, 1);
            last_gps_update_ms = last_gps_fix_ms;
            const auto &loc = gps.location();
            if (!state.have_origin) {
                state.have_origin = true;
                state.origin = loc;
            }
            const auto & vel = gps.velocity();
            const Vector3d pos = state.origin.get_distance_NED_double(loc);

            update_gps(pos.toftype(), vel.toftype(), gps_dt);
        }
    }

    update_attitude_from_compass();

    // Update GNSS delay buffers
    buffers.internal_time += dangle_dt;
    buffers.stamped_pos.push_front({buffers.internal_time, state.XHat.pos()});
    while (!buffers.stamped_pos.empty() && buffers.internal_time - buffers.stamped_pos.back().first > gains.gps_lag){
        buffers.stamped_pos.pop_back();
    }
    buffers.stamped_vel.push_front({buffers.internal_time, state.XHat.vel()});
    while (!buffers.stamped_pos.empty() && buffers.internal_time - buffers.stamped_vel.back().first > gains.gps_lag){
        buffers.stamped_vel.pop_back();
    }



    // Write logging messages

    // @LoggerMessage: CINS
    // @Description: CINS state
    // @Field: TimeUS: Time since system startup
    // @Field: I: instance
    // @Field: Roll: euler roll
    // @Field: Pitch: euler pitch
    // @Field: Yaw: euler yaw
    // @Field: VN: velocity north
    // @Field: VE: velocity east
    // @Field: VD: velocity down
    // @Field: PN: position north
    // @Field: PE: position east
    // @Field: PD: position down
    // @Field: Lat: latitude
    // @Field: Lon: longitude
    // @Field: Alt: altitude AMSL
    ftype roll_rad, pitch_rad, yaw_rad;
    state.XHat.rot().to_euler(&roll_rad, &pitch_rad, &yaw_rad);
    const Location loc = get_location();

    AP::logger().WriteStreaming("CINS", "TimeUS,I,Roll,Pitch,Yaw,VN,VE,VD,PN,PE,PD,Lat,Lon,Alt",
                                "s#dddnnnmmmDUm",
                                "F-000000000GG0",
                                "QBfffffffffLLf",
                                dal.micros64(),
                                DAL_CORE(0),
                                degrees(roll_rad),
                                degrees(pitch_rad),
                                wrap_360(degrees(yaw_rad)),
                                state.XHat.vel().x,
                                state.XHat.vel().y,
                                state.XHat.vel().z,
                                state.XHat.pos().x,
                                state.XHat.pos().y,
                                state.XHat.pos().z,
                                loc.lat,
                                loc.lng,
                                loc.alt*0.01);

    // @LoggerMessage: CIN2
    // @Description: Extra CINS states
    // @Field: TimeUS: Time since system startup
    // @Field: I: instance
    // @Field: GX: Gyro Bias X
    // @Field: GY: Gyro Bias Y
    // @Field: GZ: Gyro Bias Z
    // @Field: AX: Accel Bias X
    // @Field: AY: Accel Bias Y
    // @Field: AZ: Accel Bias Z
    // @Field: AVN: auxiliary velocity north
    // @Field: AVE: auxiliary velocity east
    // @Field: AVD: auxiliary velocity down
    // @Field: APN: auxiliary position north
    // @Field: APE: auxiliary position east
    // @Field: APD: auxiliary position down
    AP::logger().WriteStreaming("CIN2", "TimeUS,I,GX,GY,GZ,AX,AY,AZ,AVN,AVE,AVD,APN,APE,APD",
                                "s#kkkooonnnmmm",
                                "F-000000000000",
                                "QBffffffffffff",
                                dal.micros64(),
                                DAL_CORE(0),
                                degrees(state.gyr_bias.x),
                                degrees(state.gyr_bias.y),
                                degrees(state.gyr_bias.z),
                                state.acc_bias.x,
                                state.acc_bias.y,
                                state.acc_bias.z,
                                state.ZHat.W1().x,
                                state.ZHat.W1().y,
                                state.ZHat.W1().z,
                                state.ZHat.W2().x,
                                state.ZHat.W2().y,
                                state.ZHat.W2().z);
}


/*
  update on new GPS sample
 */
void AP_CINS::update_gps(const Vector3F &pos, const Vector3F &vel, const ftype gps_dt)
{
    // Forward the GPS to the current time
    // const Vector3F undelayed_pos = buffers.stamped_pos.empty() && gains.gps_lag > 0. ? pos : pos + state.XHat.pos() - buffers.stamped_pos.back().second;
    // const Vector3F undelayed_vel = buffers.stamped_vel.empty() && gains.gps_lag > 0. ? vel : vel + state.XHat.vel() - buffers.stamped_vel.back().second;

    // Update delay buffers
    delayer.YR = delayer.YR_stepper.inverse() * delayer.YR;
    delayer.YR_stepper = Gal3F::identity();
    delayer.gps_vel = vel;
    delayer.gps_pos = pos;

    // TODO: move GPS update to IMU update using delay matrices.

    //compute correction terms 
    // const Vector3F zero_vector;
    // update_vector_measurement_cts(undelayed_vel, zero_vector, Vector2F(1,0), gains.gpsvel_att, gains.gps_vel, gains.gps_vel_gyr_bias, gains.gps_vel_acc_bias, gps_dt);
    // update_vector_measurement_cts(undelayed_pos, zero_vector, Vector2F(0,1), gains.gpspos_att, gains.gps_pos, gains.gps_pos_gyr_bias, gains.gps_pos_acc_bias, gps_dt);

    // use AHRS3 for debugging
    ftype roll_rad, pitch_rad, yaw_rad;
    const auto rot = state.XHat.rot() * AP::ahrs().get_rotation_vehicle_body_to_autopilot_body().toftype();
    rot.to_euler(&roll_rad, &pitch_rad, &yaw_rad);
    const Location loc = get_location();
    const mavlink_ahrs3_t pkt {
    roll : float(roll_rad),
    pitch : float(pitch_rad),
    yaw : float(yaw_rad),
    altitude : float(-state.XHat.pos().z),
    lat: loc.lat,
    lng: loc.lng,
    v1 : 0,
    v2 : 0,
    v3 : 0,
    v4 : 0
    };
    gcs().send_to_active_channels(MAVLINK_MSG_ID_AHRS3, (const char *)&pkt);
}


/*
  update from IMU data
 */
void AP_CINS::update_imu(const Vector3F &gyro_rads, const Vector3F &accel_mss, const ftype dt)
{
    //Integrate Dynamics using the Matrix exponential 
    //Create Zero vector 
    const Vector3F zero_vector;
    const Vector3F gravity_vector{0,0,GRAVITY_MSS};

    // Update the bias gain matrices
    // In mathematical terms, \dot{M} = B = - Ad_{Z^{-1} \hat{X}} [I_3, 0_3; 0_3, I_3; 0_3, 0_3].
    // Here, we split B into its parts and add them to the parts of the bias gain matrix.
    if (done_yaw_init) {
        const SIM23 XInv_Z { SIM23(state.XHat.inverse()) * state.ZHat };
        state.gyr_bias_gain_mat.rot += -state.XHat.rot() * dt;
        state.gyr_bias_gain_mat.vel += Matrix3F::skew_symmetric(XInv_Z.W1()) * XInv_Z.R().transposed() * dt;
        state.gyr_bias_gain_mat.pos += Matrix3F::skew_symmetric(XInv_Z.W2()) * XInv_Z.R().transposed() * dt;

        // state.acc_bias_gain_mat.rot is unchanged
        state.acc_bias_gain_mat.vel += -state.XHat.rot() * state.ZHat.A().a11() * dt;
        state.acc_bias_gain_mat.pos += -state.XHat.rot() * state.ZHat.A().a12() * dt;
    }

    const Gal3F leftMat = Gal3F::exponential(zero_vector, zero_vector, gravity_vector*dt, -dt);
    const Gal3F rightMat = Gal3F::exponential((gyro_rads-state.gyr_bias)*dt, zero_vector, (accel_mss-state.acc_bias)*dt, dt);
    //Update XHat (Observer Dynamics)
    state.XHat = leftMat * state.XHat * rightMat;

    //Update ZHat (Auxilary Dynamics)
    SIM23 Gamma_IMU_inv = SIM23::identity();
    const GL2 S_Gamma = 0.5 * state.ZHat.A().transposed() * GL2(gains.Q11, 0., 0., gains.Q22) * state.ZHat.A();
    Gamma_IMU_inv.A() = GL2::identity() - dt * S_Gamma;

    // Update the bias gain with Gamma
    compute_bias_update_imu(Gamma_IMU_inv.inverse());

    state.ZHat = SIM23(leftMat) * state.ZHat * Gamma_IMU_inv;

    // Update the delay matrices and buffer
    delayer.YR = delayer.YR * rightMat;
    delayer.internal_time += dt;
    delayer.stamped_inputs.push_front({delayer.internal_time, rightMat});
    while (!delayer.stamped_inputs.empty() && delayer.stamped_inputs.back().first < delayer.internal_time - gains.gps_lag){
        delayer.YR_stepper = delayer.YR_stepper * delayer.stamped_inputs.back().second;
        delayer.stamped_inputs.pop_back();
    }

    // Update using delayed GPS
    const Vector3F& left_delay_vel = -gravity_vector*gains.gps_lag;
    const Vector3F& left_delay_pos = -gravity_vector*0.5*gains.gps_lag*gains.gps_lag;

    const Vector2F& ref_vel = Vector2F(1., 0.);
    const Vector3F& mu0_vel = delayer.YR.vel();
    const Vector3F mu_vel = delayer.gps_vel - left_delay_vel * ref_vel.x - left_delay_pos * ref_vel.y;
    update_vector_measurement_cts(mu_vel, mu0_vel, ref_vel, gains.gpsvel_att, gains.gps_vel, gains.gps_vel_gyr_bias, gains.gps_vel_acc_bias, dt);

    const Vector3F& mu0_pos = delayer.YR.pos();
    const Vector2F& ref_pos = Vector2F(-gains.gps_lag, 1.);
    const Vector3F mu_pos = delayer.gps_pos - left_delay_vel * ref_pos.x - left_delay_pos * ref_pos.y;
    update_vector_measurement_cts(mu_pos, mu0_pos, ref_pos, gains.gpspos_att, gains.gps_pos, gains.gps_pos_gyr_bias, gains.gps_pos_acc_bias, dt);

}

void AP_CINS::update_vector_measurement_cts(const Vector3F &measurement, const Vector3F& reference, const Vector2F &ref_base, const ftype& gain_R, const ftype& gain_V, const ftype& gain_gyr_bias, const ftype& gain_acc_bias, const ftype dt) {
    // Compute and apply an update for an arbitrary vector-type measurement.
    // The measurement is of the form $\mu = R \mu_0 + V C$, where
    // \mu is the measured value
    // \mu_0 is a reference vector (e.g. zero for GPS and 'north' for magnetometer)
    // C is a the reference vector 'base' (e.g. zero for magnetometer and (1,0) or (0,1) for GPS vel. or pos.)
    // The update equations are then drawn from Lemma 5.3 in
    // van Goor, Pieter, et al. "Constructive synchronous observer design for inertial navigation with delayed GNSS measurements." European Journal of Control (2024): 101047.

    // Set up some computation variables
    const Vector3F& muHat = state.XHat.rot() * reference + state.XHat.vel()*ref_base.x + state.XHat.pos()*ref_base.y;
    const SIM23 ZInv { state.ZHat.inverse() };
    const Vector2F& gains_AInvC = ZInv.A() * ref_base;
    const GL2& CCT = GL2(ref_base.x*ref_base.x, ref_base.x*ref_base.y, ref_base.x*ref_base.y, ref_base.y*ref_base.y);
    const Vector3F& mu_Z = state.ZHat.W1()*gains_AInvC.x + state.ZHat.W2()*gains_AInvC.y;

    // Compute correction terms
    Matrix3F I3; I3.identity();
    const GL2& S_Gamma = -0.5 * (gain_V) * ZInv.A() * CCT * ZInv.A().transposed();
    const Vector3F& W_Gamma_1 = - (measurement - mu_Z) * gains_AInvC.x * (gain_R+gain_V);
    const Vector3F& W_Gamma_2 = - (measurement - mu_Z) * gains_AInvC.y * (gain_R+gain_V);
    const SIM23 GammaInv = SIM23(I3, -W_Gamma_1*dt, -W_Gamma_2*dt, GL2::identity() - dt*S_Gamma);

    Vector3F Omega_Delta = computeRotationCorrection((muHat - mu_Z), -(measurement - mu_Z), 4.*gain_R, dt);
    Vector3F W_Delta1 = (measurement - muHat) * gains_AInvC.x * (gain_R + gain_V);
    Vector3F W_Delta2 = (measurement - muHat) * gains_AInvC.y * (gain_R + gain_V);

    // Compute the bias correction

    // Construct the components of the 9x9 Adjoint matrix $\Ad_Z$
    const Matrix3F& AdZ_11 = state.ZHat.R();
    const Matrix3F& AdZ_21 = - Matrix3F::skew_symmetric(ZInv.W1());
    const Matrix3F& AdZ_22 = state.ZHat.R() * ZInv.A().a11();
    const Matrix3F& AdZ_23 = state.ZHat.R() * ZInv.A().a21();
    const Matrix3F& AdZ_31 = - Matrix3F::skew_symmetric(ZInv.W2());
    const Matrix3F& AdZ_32 = state.ZHat.R() * ZInv.A().a12();
    const Matrix3F& AdZ_33 = state.ZHat.R() * ZInv.A().a22();

    // Linearised measurement matrix $\mu - \hat{\mu} = Dh DR_{\hat{X}} \Ad_Z \varepsilon$.
    const Matrix3F& pre_C11 = - Matrix3F::skew_symmetric(muHat);
    const Matrix3F& pre_C12 = I3*ref_base.x;
    const Matrix3F& pre_C13 = I3*ref_base.y;

    const Matrix3F& C11 = pre_C11 * AdZ_11 + pre_C12 * AdZ_21 + pre_C13 * AdZ_31;
    const Matrix3F& C12 = pre_C12 * AdZ_22 + pre_C13 * AdZ_32;
    const Matrix3F& C13 = pre_C12 * AdZ_23 + pre_C13 * AdZ_33;

    // Gain matrix $\Delta = K (\mu - \hat{\mu})$.
    const Matrix3F& K11 = Matrix3F::skew_symmetric(muHat - mu_Z) * 4. * gain_R * dt;
    const Matrix3F& K21 = I3 * gains_AInvC.x * (gain_R + gain_V) * dt;
    const Matrix3F& K31 = I3 * gains_AInvC.y * (gain_R + gain_V) * dt;

    // Compute the bias correction using the C matrices.
    // The correction to bias is given by the formula delta_b = k_b (MC)^\top (y-\hat{y})
    const Matrix3F M1Gyr = state.gyr_bias_gain_mat.rot;
    const Matrix3F M2Gyr = state.gyr_bias_gain_mat.vel;
    const Matrix3F M3Gyr = state.gyr_bias_gain_mat.pos;

    const Matrix3F M1Acc = state.acc_bias_gain_mat.rot;
    const Matrix3F M2Acc = state.acc_bias_gain_mat.vel;
    const Matrix3F M3Acc = state.acc_bias_gain_mat.pos;

    state.gyr_bias_correction = (C11*M1Gyr + C12*M2Gyr + C13*M3Gyr).mul_transpose(measurement - muHat) * gain_gyr_bias;
    state.acc_bias_correction = (C11*M1Acc + C12*M2Acc + C13*M3Acc).mul_transpose(measurement - muHat) * gain_acc_bias;
    saturate_bias(state.gyr_bias_correction, state.gyr_bias, gains.sat_gyr_bias, dt);
    saturate_bias(state.acc_bias_correction, state.acc_bias, gains.sat_acc_bias, dt);

    // Compute the bias gain matrix updates
    const Matrix3F& A11 = I3;
    const Matrix3F& A21 = - Matrix3F::skew_symmetric(GammaInv.W1());
    const Matrix3F& A22 = I3*GammaInv.A().a11();
    const Matrix3F& A23 = I3*GammaInv.A().a21();
    const Matrix3F& A31 = - Matrix3F::skew_symmetric(GammaInv.W2());
    const Matrix3F& A32 = I3*GammaInv.A().a12();
    const Matrix3F& A33 = I3*GammaInv.A().a22();

    state.gyr_bias_gain_mat.rot = (A11 - K11 * C11) * M1Gyr + ( K11 * C12) * M2Gyr + (K11 * C13) * M3Gyr;
    state.gyr_bias_gain_mat.vel = (A21 - K21 * C11) * M1Gyr + (A22 - K21 * C12) * M2Gyr + (A23 - K21 * C13) * M3Gyr;
    state.gyr_bias_gain_mat.pos = (A31 - K31 * C11) * M1Gyr + (A32 - K31 * C12) * M2Gyr + (A33 - K31 * C13) * M3Gyr;

    state.acc_bias_gain_mat.rot = (A11 - K11 * C11) * M1Acc + ( K11 * C12) * M2Acc + (K11 * C13) * M3Acc;
    state.acc_bias_gain_mat.vel = (A21 - K21 * C11) * M1Acc + (A22 - K21 * C12) * M2Acc + (A23 - K21 * C13) * M3Acc;
    state.acc_bias_gain_mat.pos = (A31 - K31 * C11) * M1Acc + (A32 - K31 * C12) * M2Acc + (A33 - K31 * C13) * M3Acc;

    // Apply the state and bias updates
    state.gyr_bias += state.gyr_bias_correction * dt;
    state.acc_bias += state.acc_bias_correction * dt;
    Omega_Delta += state.gyr_bias_gain_mat.rot * state.gyr_bias_correction + state.acc_bias_gain_mat.rot * state.acc_bias_correction;
    W_Delta1 += state.gyr_bias_gain_mat.pos * state.gyr_bias_correction + state.acc_bias_gain_mat.pos * state.acc_bias_correction;
    W_Delta2 += state.gyr_bias_gain_mat.vel * state.gyr_bias_correction + state.acc_bias_gain_mat.vel * state.acc_bias_correction;

    // Construct the correction term Delta
    SIM23 Delta_SIM23 = SIM23(Matrix3F::from_angular_velocity(Omega_Delta*dt), W_Delta1*dt, W_Delta2*dt, GL2::identity());
    Delta_SIM23 = state.ZHat * Delta_SIM23 * ZInv;
    const Gal3F Delta = Gal3F(Delta_SIM23.R(), Delta_SIM23.W2(), Delta_SIM23.W1(), 0.);

    // Update the states using the correction terms
    state.XHat = Delta * state.XHat;
    state.ZHat = state.ZHat * GammaInv;
}

/*
  initialise yaw from compass, if available
 */
bool AP_CINS::init_yaw(void)
{
    ftype mag_yaw, dt;
    if (!get_compass_yaw(mag_yaw, dt)) {
        return false;
    }
    ftype roll_rad, pitch_rad, yaw_rad;
    state.XHat.rot().to_euler(&roll_rad, &pitch_rad, &yaw_rad);
    state.XHat.rot().from_euler(roll_rad, pitch_rad, mag_yaw);
    
    return true;
}

/*
  get yaw from compass
 */
bool AP_CINS::get_compass_yaw(ftype &yaw_rad, ftype &dt)
{
    auto &dal = AP::dal();
    const auto &compass = dal.compass();
    if (compass.get_num_enabled() == 0) {
        return false;
    }
    const uint8_t mag_idx = compass.get_first_usable();
    if (!compass.healthy(mag_idx)) {
        return false;
    }
    if (!state.have_origin) {
        return false;
    }
    const auto &field = compass.get_field(mag_idx);
    const uint32_t last_us = compass.last_update_usec(mag_idx);
    if (last_us == last_mag_us) {
        // no new data
        return false;
    }
    dt = (last_us - last_mag_us) * 1.0e-6;
    last_mag_us = last_us;

    const float declination_rad = dal.compass().get_declination();
    if (is_zero(declination_rad)) {
        // wait for declination
        return false;
    }

    // const float cos_pitch_sq = 1.0f-(state.XHat.rot().c.x*state.XHat.rot().c.x);
    // const float headY = field.y * state.XHat.rot().c.z - field.z * state.XHat.rot().c.y;

    // // Tilt compensated magnetic field X component:
    // const float headX = field.x * cos_pitch_sq - state.XHat.rot().c.x * (field.y * state.XHat.rot().c.y + field.z * state.XHat.rot().c.z);

    // return magnetic yaw

    yaw_rad = wrap_PI(-atan2f(field.y,field.x) + declination_rad);

    return true;
}


bool AP_CINS::get_compass_vector(Vector3F &mag_vec, Vector3F &mag_ref, ftype &dt)
{
    auto &dal = AP::dal();
    const auto &compass = dal.compass();
    if (compass.get_num_enabled() == 0) {
        return false;
    }
    const uint8_t mag_idx = compass.get_first_usable();
    if (!compass.healthy(mag_idx)) {
        return false;
    }
    if (!state.have_origin) {
        return false;
    }
    mag_vec = compass.get_field(mag_idx).toftype();
    const uint32_t last_us = compass.last_update_usec(mag_idx);
    if (last_us == last_mag_us) {
        // no new data
        return false;
    }
    dt = (last_us - last_mag_us) * 1.0e-6;
    last_mag_us = last_us;

    const Location loc = get_location();
    mag_ref = AP_Declination::get_earth_field_ga(loc).toftype();
    if (mag_ref.is_zero()) {
        // wait for declination
        return false;
    }

    return true;
}

void AP_CINS::update_attitude_from_compass() {
    ftype dt;
    Vector3F mag_vec, mag_ref;
    if (!get_compass_vector(mag_vec, mag_ref, dt)) {
        return;
    }
    mag_vec *= 1.e-3; // Convert mag measurement from milliGauss to Gauss


    Vector2F zero_2;
    update_vector_measurement_cts(mag_ref, mag_vec, zero_2, gains.mag_att, 0.0, gains.mag_gyr_bias, gains.mag_acc_bias, dt);
}

void AP_CINS::compute_bias_update_imu(const SIM23& Gamma) {
    // Compute the bias update for IMU inputs

    const SIM23 GammaInv = Gamma.inverse();
    const Matrix3F& Ad_Gamma_11 { Gamma.R() };
    const Matrix3F Ad_Gamma_12; // Zero matrix
    const Matrix3F Ad_Gamma_13; // Zero matrix
    const Matrix3F Ad_Gamma_21 = - Matrix3F::skew_symmetric(GammaInv.W1()) * Gamma.R();
    const Matrix3F Ad_Gamma_22 = Gamma.R() * GammaInv.A().a11();
    const Matrix3F Ad_Gamma_23 = Gamma.R() * GammaInv.A().a21();
    const Matrix3F Ad_Gamma_31 = - Matrix3F::skew_symmetric(GammaInv.W2()) * Gamma.R();
    const Matrix3F Ad_Gamma_32 = Gamma.R() * GammaInv.A().a12();
    const Matrix3F Ad_Gamma_33 = Gamma.R() * GammaInv.A().a22();

    // Implement M(t+) = A M(t)
    const Matrix3F &M1Gyr = state.gyr_bias_gain_mat.rot;
    const Matrix3F &M2Gyr = state.gyr_bias_gain_mat.vel;
    const Matrix3F &M3Gyr = state.gyr_bias_gain_mat.pos;
    state.gyr_bias_gain_mat.rot = Ad_Gamma_11 * M1Gyr + Ad_Gamma_12 * M2Gyr + Ad_Gamma_13 * M3Gyr;
    state.gyr_bias_gain_mat.vel = Ad_Gamma_21 * M1Gyr + Ad_Gamma_22 * M2Gyr + Ad_Gamma_23 * M3Gyr;
    state.gyr_bias_gain_mat.pos = Ad_Gamma_31 * M1Gyr + Ad_Gamma_32 * M2Gyr + Ad_Gamma_33 * M3Gyr;

    const Matrix3F &M1Acc = state.acc_bias_gain_mat.rot;
    const Matrix3F &M2Acc = state.acc_bias_gain_mat.vel;
    const Matrix3F &M3Acc = state.acc_bias_gain_mat.pos;
    state.acc_bias_gain_mat.rot = Ad_Gamma_11 * M1Acc + Ad_Gamma_12 * M2Acc + Ad_Gamma_13 * M3Acc;
    state.acc_bias_gain_mat.vel = Ad_Gamma_21 * M1Acc + Ad_Gamma_22 * M2Acc + Ad_Gamma_23 * M3Acc;
    state.acc_bias_gain_mat.pos = Ad_Gamma_31 * M1Acc + Ad_Gamma_32 * M2Acc + Ad_Gamma_33 * M3Acc;
}


void AP_CINS::saturate_bias(Vector3F& bias_correction, const Vector3F& current_bias, const ftype& saturation, const ftype& dt) const {
    // Ensure that no part of the bias exceeds the saturation limit
    if (abs(bias_correction.x) > 1E-8)
        bias_correction.x *= MIN(1., (saturation - abs(current_bias.x)) / (dt * abs(bias_correction.x)));
    if (abs(bias_correction.y) > 1E-8)
        bias_correction.y *= MIN(1., (saturation - abs(current_bias.y)) / (dt * abs(bias_correction.y)));
    if (abs(bias_correction.z) > 1E-8)
        bias_correction.z *= MIN(1., (saturation - abs(current_bias.z)) / (dt * abs(bias_correction.z)));
}


Vector3F computeRotationCorrection(const Vector3F& v1, const Vector3F& v2, const ftype& gain, const ftype& dt) {
    // The correction is given by Omega = - skew(v1) * v2 * gain, where v2 = R v0 for some const. v0.
    // It is applied as dR/dt = skew(Omega) R.
    // The key point is that Omega changes R which changes Omega.
    // We use a first order expansion to capture this effect here.

    const Vector3F omega1 = - Matrix3F::skew_symmetric(v1) * v2 * gain;
    const Vector3F omega2 = - Matrix3F::skew_symmetric(v1) * Matrix3F::skew_symmetric(omega1) * v2 * gain;
    const Vector3F omega3 = - Matrix3F::skew_symmetric(v1) * Matrix3F::skew_symmetric(omega2) * v2 * gain
        - Matrix3F::skew_symmetric(v1) * Matrix3F::skew_symmetric(omega1) * Matrix3F::skew_symmetric(omega1) * v2 * gain;
    return omega1 + omega2*dt + omega3*0.5*dt*dt;
    // return omega1;

}
