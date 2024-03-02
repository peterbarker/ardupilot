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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include "AP_Notify_config.h"

#include "NotifyDevice.h"

// Device parameters values
#define RGB_LED_OFF     0
#define RGB_LED_LOW     1
#define RGB_LED_MEDIUM  2
#define RGB_LED_HIGH    3
#define BUZZER_ON       1
#define BUZZER_OFF      0

#define NOTIFY_TEXT_BUFFER_SIZE 51

//Type of on-board display
#define DISPLAY_OFF     0
#define DISPLAY_SSD1306 1
#define DISPLAY_SH1106  2
#define DISPLAY_SITL 10

class AP_Notify
{
    friend class RGBLed;            // RGBLed needs access to notify parameters
    friend class Display;           // Display needs access to notify parameters
public:
    AP_Notify();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Notify);

    // get singleton instance
    static AP_Notify *get_singleton(void) {
        return _singleton;
    }

    enum Notify_LED_Type {
        Notify_LED_None                     = 0,        // not enabled
        Notify_LED_Board                    = (1 << 0), // Built in board LED's
#if AP_NOTIFY_TOSHIBALED_ENABLED
        Notify_LED_ToshibaLED_I2C_Internal  = (1 << 1), // Internal ToshibaLED_I2C
        Notify_LED_ToshibaLED_I2C_External  = (1 << 2), // External ToshibaLED_I2C
#endif
#if AP_NOTIFY_PCA9685_ENABLED
        Notify_LED_PCA9685LED_I2C_External  = (1 << 3), // External PCA9685_I2C
#endif
#if AP_NOTIFY_OREOLED_ENABLED
        Notify_LED_OreoLED                  = (1 << 4), // Oreo
#endif
#if AP_NOTIFY_DRONECAN_LED_ENABLED
        Notify_LED_DroneCAN                   = (1 << 5), // UAVCAN RGB LED
#endif
#if AP_NOTIFY_NCP5623_ENABLED
        Notify_LED_NCP5623_I2C_External     = (1 << 6), // External NCP5623
        Notify_LED_NCP5623_I2C_Internal     = (1 << 7), // Internal NCP5623
#endif
#if AP_NOTIFY_NEOPIXEL_ENABLED
        Notify_LED_NeoPixel                 = (1 << 8), // NeoPixel 5050 AdaFruit 1655 SK6812  Worldsemi WS2812B
#endif
#if AP_NOTIFY_PROFILED_ENABLED
        Notify_LED_ProfiLED                 = (1 << 9), // ProfiLED
#endif
#if AP_NOTIFY_SCRIPTING_LED_ENABLED
        Notify_LED_Scripting                = (1 << 10),// Colour accessor for scripting
#endif
#if AP_NOTIFY_DSHOT_LED_ENABLED
        Notify_LED_DShot                    = (1 << 11),// Use dshot commands to set ESC LEDs
#endif
#if AP_NOTIFY_PROFILED_SPI_ENABLED
        Notify_LED_ProfiLED_SPI             = (1 << 12), // ProfiLED (SPI)
#endif
#if AP_NOTIFY_LP5562_ENABLED
        Notify_LED_LP5562_I2C_External      = (1 << 13), // LP5562
        Notify_LED_LP5562_I2C_Internal      = (1 << 14), // LP5562
#endif
#if AP_NOTIFY_IS31FL3195_ENABLED
        Notify_LED_IS31FL3195_I2C_External  = (1 << 15), // IS31FL3195
        Notify_LED_IS31FL3195_I2C_Internal  = (1 << 16), // IS31FL3195
#endif
#if AP_NOTIFY_DISCRETE_RGB_ENABLED
        Notify_LED_DiscreteRGB              = (1 << 17), // DiscreteRGB
#endif
#if AP_NOTIFY_NEOPIXEL_ENABLED
        Notify_LED_NeoPixelRGB              = (1 << 18), // NeoPixel AdaFruit 4544 Worldsemi WS2811
#endif
        Notify_LED_MAX
    };

    enum Notify_Buzz_Type {
        Notify_Buzz_None                    = 0,
        Notify_Buzz_Builtin                 = (1 << 0), // Built in default Alarm Out
        Notify_Buzz_DShot                   = (1 << 1), // DShot Alarm
        Notify_Buzz_UAVCAN                  = (1 << 2), // UAVCAN Alarm
    };


    enum class Flag {
        INITIALISING,
        RADIO_FAILSAFE,
        GCS_FAILSAFE,
        BATTERY_FAILSAFE,
        EKF_FAILSAFE,
        PRE_ARMS_OK,
        COMPASS_CAL_RUNNING,
        ARMED,
        TEMP_CAL_RUNNING,
        EKF_BAD,
        FIRMWARE_UPDATE,
        PARACHUTE_RELEASED,
        POWERING_OFF,
        GYRO_CALIBRATED,
        FLYING,
        ESC_CALIBRATION,
        GPS_FUSION,
        HAVE_POS_ABS,
        GPS_GLITCHING,
        VEHICLE_LOST,
        LEAK_DETECTED,
        PRE_ARM_GPS_CHECK,
        SAVE_TRIM,
        VIDEO_RECORDING,
        WAITING_FOR_THROW,
        AUTOPILOT_MODE,
    };

    static bool flag_is_set(Flag flag) { return flags & (1U << (uint8_t)flag); }
    static void set_flag(Flag flag, bool value);

    static void set_flight_mode(uint8_t flight_mode) { _flight_mode = flight_mode; }
    static uint8_t flight_mode() { return _flight_mode; }

    static void set_gps_status(uint8_t status) { _gps_status = status; }
    static uint8_t gps_status() { return _gps_status; }

    static void set_gps_num_sats(uint8_t num_sats) { _gps_num_sats = num_sats; }
    static uint8_t gps_num_sats() { return _gps_num_sats; }

    static void set_tune_next(uint8_t _tune) { tune = _tune; }
    static uint8_t get_tune_next() { return tune; }

    enum class Event {
        ARMING_FAILED,
        USER_MODE_CHANGE,
        USER_MODE_CHANGE_FAILED,
        FAILSAFE_MODE_CHANGE,
        AUTOTUNE_COMPLETE,
        AUTOTUNE_FAILED,
        AUTOTUNE_NEXT_AXIS,
        MISSION_COMPLETE,
        WAYPOINT_COMPLETE,
        INITIATED_COMPASS_CAL,
        COMPASS_CAL_SAVED,
        COMPASS_CAL_FAILED,
        COMPASS_CAL_CANCELED,
        TUNE_STARTED,
        TUNE_SAVE,
        TUNE_ERROR,
        INITIATED_TEMP_CAL,
        TEMP_CAL_SAVED,
        TEMP_CAL_FAILED,


        FLAG_CHANGED_ARMED_ON,
        FLAG_CHANGED_ARMED_OFF,
        FLAG_CHANGED_BATTERY_FAILSAFE_ON,
        FLAG_CHANGED_BATTERY_FAILSAFE_OFF,
        FLAG_CHANGED_PARACHUTE_RELEASED_ON,
        FLAG_CHANGED_PARACHUTE_RELEASED_OFF,
        FLAG_CHANGED_PRE_ARMS_OK_ON,
        FLAG_CHANGED_PRE_ARMS_OK_OFF,
        FLAG_CHANGED_RADIO_FAILSAFE_ON,
        FLAG_CHANGED_RADIO_FAILSAFE_OFF,
        FLAG_CHANGED_GCS_FAILSAFE_ON,
        FLAG_CHANGED_GCS_FAILSAFE_OFF,
        FLAG_CHANGED_EKF_FAILSAFE_ON,
        FLAG_CHANGED_EKF_FAILSAFE_OFF,
        FLAG_CHANGED_VEHICLE_LOST_ON,
        FLAG_CHANGED_VEHICLE_LOST_OFF,
        FLAG_CHANGED_COMPASS_CAL_RUNNING_ON,
        FLAG_CHANGED_COMPASS_CAL_RUNNING_OFF,
        FLAG_CHANGED_WAITING_FOR_THROW_ON,
        FLAG_CHANGED_WAITING_FOR_THROW_OFF,
        FLAG_CHANGED_LEAK_DETECTED_ON,
        FLAG_CHANGED_LEAK_DETECTED_OFF,
        FLAG_CHANGED_POWERING_OFF_ON,
        FLAG_CHANGED_POWERING_OFF_OFF,
        FLAG_CHANGED_TEMP_CAL_RUNNING_ON,
        FLAG_CHANGED_TEMP_CAL_RUNNING_OFF,
        FLAG_CHANGED_EKF_BAD_ON,
        FLAG_CHANGED_EKF_BAD_OFF,
    };

    static void event(Event event) { events |= (1U<<(uint8_t)event); }
    static bool event_triggered(Event event) { return events & (1U<<(uint8_t)event); }

private:

    static uint8_t _gps_status;       // see the GPS_0 = no gps, 1 = no lock, 2 = 2d lock, 3 = 3d lock, 4 = dgps lock, 5 = rtk lock
    static uint8_t _gps_num_sats;     // number of sats
    static uint8_t _flight_mode;      // flight mode

    static uint32_t flags;

    static uint8_t tune;

    // The notify flags and values are static to allow direct class access
    // without declaring the object.
    static uint64_t events;

public:

    // initialisation
    void init(void);

    /// update - allow updates of leds that cannot be updated during a timed interrupt
    void update(void);

#if AP_NOTIFY_MAVLINK_LED_CONTROL_SUPPORT_ENABLED
    // handle a LED_CONTROL message
    static void handle_led_control(const mavlink_message_t &msg);
#endif

    // handle RGB from Scripting or AP_Periph
    static void handle_rgb(uint8_t r, uint8_t g, uint8_t b, uint8_t rate_hz = 0);

    // handle RGB from Scripting
    static void handle_rgb_id(uint8_t r, uint8_t g, uint8_t b, uint8_t id);

#if AP_NOTIFY_MAVLINK_PLAY_TUNE_SUPPORT_ENABLED
    // handle a PLAY_TUNE message
    static void handle_play_tune(const mavlink_message_t &msg);
#endif

    // play a tune string
    static void play_tune(const char *tune);

    bool buzzer_enabled() const { return _buzzer_type != 0; }

    uint8_t get_buzzer_types() const { return _buzzer_type; }

    // set flight mode string
    void set_flight_mode_str(const char *str);
    const char* get_flight_mode_str() const { return _flight_mode_str; }

    // send text to display
    void send_text(const char *str);
    const char* get_text() const { return _send_text; }
    uint32_t get_text_updated_millis() const {return _send_text_updated_millis; }
 
#if AP_SCRIPTING_ENABLED
    // send text to the display using scripting
    void send_text_scripting(const char *str, uint8_t r);
    void release_text_scripting(uint8_t r);
#endif

    static const struct AP_Param::GroupInfo var_info[];
    int8_t get_buzz_pin() const  { return _buzzer_pin; }
    uint8_t get_buzz_level() const  { return _buzzer_level; }
    uint8_t get_buzz_volume() const  { return _buzzer_volume; }
    uint8_t get_led_len() const { return _led_len; }
    uint32_t get_led_type() const { return _led_type; }
    int8_t get_rgb_led_brightness_percent() const;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    HAL_Semaphore sf_window_mutex;
#endif

private:

    static AP_Notify *_singleton;

    void add_backend_helper(NotifyDevice *backend);

    // add all backends
    void add_backends(void);

    // parameters
    AP_Int8 _rgb_led_brightness;
    AP_Int8 _rgb_led_override;
    AP_Int8 _buzzer_type;
    AP_Int8 _display_type;
    AP_Int8 _oreo_theme;
    AP_Int8 _buzzer_pin;
    AP_Int32 _led_type;
    AP_Int8 _buzzer_level;
    AP_Int8 _buzzer_volume;
    AP_Int8 _led_len;

    char _send_text[NOTIFY_TEXT_BUFFER_SIZE];
    uint32_t _send_text_updated_millis; // last time text changed
    char _flight_mode_str[5];

    static NotifyDevice* _devices[];
    static uint8_t _num_devices;
};

namespace AP {
    AP_Notify &notify();
};
