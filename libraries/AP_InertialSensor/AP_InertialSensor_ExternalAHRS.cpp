#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_ExternalAHRS.h"
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

#if HAL_EXTERNAL_AHRS_ENABLED

const extern AP_HAL::HAL& hal;

AP_InertialSensor_ExternalAHRS::AP_InertialSensor_ExternalAHRS(AP_InertialSensor &imu, uint8_t _serial_port) :
    AP_InertialSensor_Backend(imu),
    serial_port(_serial_port)
{
}

void AP_InertialSensor_ExternalAHRS::handle_external(const AP_ExternalAHRS::ins_data_message_t &pkt)
{
    if (!started) {
        return;
    }

    if ((pkt.valid_fields & AP_ExternalAHRS::ins_data_message_field::ACCEL) != 0) {
        Vector3f accel = pkt.accel;
        _rotate_and_correct_accel(accel_instance, accel);
        _notify_new_accel_raw_sample(accel_instance, accel, AP_HAL::micros64());
    }

    if ((pkt.valid_fields & AP_ExternalAHRS::ins_data_message_field::TEMPERATURE) != 0) {
        _publish_temperature(accel_instance, pkt.temperature);
    }


    if ((pkt.valid_fields & AP_ExternalAHRS::ins_data_message_field::GYRO) != 0) {
        Vector3f gyro = pkt.gyro;
        gcs().send_text(MAV_SEVERITY_INFO, "%f/%f/%f", gyro[0], gyro[1], gyro[2]);
        _notify_new_gyro_sensor_rate_sample(gyro_instance, gyro);
        gcs().send_text(MAV_SEVERITY_INFO, "%fx%fx%f", gyro[0], gyro[1], gyro[2]);
        _rotate_and_correct_gyro(gyro_instance, gyro);
        gcs().send_text(MAV_SEVERITY_INFO, "%fy%fy%f", gyro[0], gyro[1], gyro[2]);
        _notify_new_gyro_raw_sample(gyro_instance, gyro, AP_HAL::micros64());
    }
}

bool AP_InertialSensor_ExternalAHRS::update(void)
{
    if (started) {
        update_accel(accel_instance);
        update_gyro(gyro_instance);
    }
    return started;
}

void AP_InertialSensor_ExternalAHRS::start()
{
    const float rate = AP::externalAHRS().get_IMU_rate();
    if (_imu.register_gyro(gyro_instance, rate,
                           AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL, serial_port, 1, DEVTYPE_SERIAL)) &&
        _imu.register_accel(accel_instance, rate,
                            AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL, serial_port, 2, DEVTYPE_SERIAL))) {
        started = true;
    }
}

void AP_InertialSensor_ExternalAHRS::accumulate()
{
    AP::externalAHRS().update();
}

#endif // HAL_EXTERNAL_AHRS_ENABLED

