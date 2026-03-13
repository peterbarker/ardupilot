/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * AP_OSD partially based on betaflight and inav osd.c implemention.
 * clarity.mcm font is taken from inav configurator.
 * Many thanks to their authors.
 */

/*
  parameter object for one setting in AP_OSD
 */

#include "AP_OSD.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <AP_ADSB/AP_ADSB_config.h>
#include <AP_AIS/AP_AIS_config.h>
#include <AP_Beacon/AP_Beacon_config.h>
#include <AP_CRSF/AP_CRSF_config.h>
#include <AP_DDS/AP_DDS_config.h>
#include <AP_Devo_Telem/AP_Devo_Telem.h>
#include <AP_EFI/AP_EFI_config.h>
#include <AP_FETtecOneWire/AP_FETtecOneWire.h>
#include <AP_Frsky_Telem/AP_Frsky_config.h>
#include <AC_Sprayer/AC_Sprayer_config.h>
#include <AP_Generator/AP_Generator_config.h>
#include <AP_Gripper/AP_Gripper_config.h>
#include <AP_Hott_Telem/AP_Hott_Telem.h>
#include <AP_IBus_Telem/AP_IBus_Telem.h>
#include <AP_ICEngine/AP_ICEngine_config.h>
#include <AP_InertialSensor/AP_InertialSensor_config.h>
#include <AP_LandingGear/AP_LandingGear_config.h>
#include <AP_LTM_Telem/AP_LTM_Telem.h>
#include <AP_Mount/AP_Mount_config.h>
#include <AP_MSP/AP_MSP_config.h>
#include <AP_Networking/AP_Networking_Config.h>
#include <AP_NMEA_Output/AP_NMEA_Output_config.h>
#include <AP_Notify/AP_Notify_config.h>
#include <AP_Proximity/AP_Proximity_config.h>
#include <AP_RobotisServo/AP_RobotisServo.h>
#include <AP_SBusOut/AP_SBusOut_config.h>
#include <AP_Scripting/AP_Scripting_config.h>
#include <AP_Soaring/AP_Soaring_config.h>
#include <AP_Volz_Protocol/AP_Volz_Protocol.h>
#include <AP_WindVane/AP_WindVane_config.h>

#if OSD_PARAM_ENABLED

const AP_Param::GroupInfo AP_OSD_ParamSetting::var_info[] = {
    // @Param: _EN
    // @DisplayName: Enable
    // @Description: Enable setting
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("_EN", 1, AP_OSD_ParamSetting, enabled, default_enabled),

    // @Param: _X
    // @DisplayName: X position
    // @Description: Horizontal position on screen
    // @Range: 0 29
    // @User: Standard
    AP_GROUPINFO("_X", 2, AP_OSD_ParamSetting, xpos, 2),

    // @Param: _Y
    // @DisplayName: Y position
    // @Description: Vertical position on screen
    // @Range: 0 15
    // @User: Standard
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("_Y", 3, AP_OSD_ParamSetting, ypos, default_ypos),

    // Parameter access keys. These default to -1 too allow user overrides
    // to work properly

    // @Param: _KEY
    // @DisplayName: Parameter key
    // @Description: Key of the parameter to be displayed and modified
    // @User: Standard
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("_KEY", 4, AP_OSD_ParamSetting, _param_key, default_param_key),

    // @Param: _IDX
    // @DisplayName: Parameter index
    // @Description: Index of the parameter to be displayed and modified
    // @User: Standard
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("_IDX", 5, AP_OSD_ParamSetting, _param_idx, default_param_idx),

    // @Param: _GRP
    // @DisplayName: Parameter group
    // @Description: Group of the parameter to be displayed and modified
    // @User: Standard
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("_GRP", 6, AP_OSD_ParamSetting, _param_group, default_param_group),

    // @Param: _MIN
    // @DisplayName: Parameter minimum
    // @Description: Minimum value of the parameter to be displayed and modified
    // @User: Standard
    AP_GROUPINFO("_MIN", 7, AP_OSD_ParamSetting, _param_min, 0.0f),

    // @Param: _MAX
    // @DisplayName: Parameter maximum
    // @Description: Maximum of the parameter to be displayed and modified
    // @User: Standard
    AP_GROUPINFO("_MAX", 8, AP_OSD_ParamSetting, _param_max, 1.0f),

    // @Param: _INCR
    // @DisplayName: Parameter increment
    // @Description: Increment of the parameter to be displayed and modified
    // @User: Standard
    AP_GROUPINFO("_INCR", 9, AP_OSD_ParamSetting, _param_incr, 0.001f),

    // @Param: _TYPE
    // @DisplayName: Parameter type
    // @Description: Type of the parameter to be displayed and modified
    // @User: Standard
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("_TYPE", 10, AP_OSD_ParamSetting, _type, default_type),

    AP_GROUPEND
};

#if HAL_GCS_ENABLED
// ensure that our OSD_PARAM type enumeration is 1:1 with the mavlink
// numbers.  This allows us to do a simple cast from one to the other
// when sending mavlink messages, rather than having some sort of
// mapping function from our internal enumeration into the mavlink
// enumeration.  Doing things this way has two advantages - in the
// future we could add that mapping function and change our
// enumeration, and the other is that it allows us to build the GPS
// library without having the mavlink headers built (for example, in
// AP_Periph we shouldn't need mavlink headers).
static_assert((uint32_t)AP_OSD_ParamSetting::Type::NONE == (uint32_t)OSD_PARAM_NONE, "OSD_PARAM_NONE incorrect");
static_assert((uint32_t)AP_OSD_ParamSetting::Type::SERIAL_PROTOCOL == (uint32_t)OSD_PARAM_SERIAL_PROTOCOL, "OSD_PARAM_SERIAL_PROTOCOL incorrect");
static_assert((uint32_t)AP_OSD_ParamSetting::Type::SERVO_FUNCTION == (uint32_t)OSD_PARAM_SERVO_FUNCTION, "OSD_PARAM_SERVO_FUNCTION incorrect");
static_assert((uint32_t)AP_OSD_ParamSetting::Type::AUX_FUNCTION == (uint32_t)OSD_PARAM_AUX_FUNCTION, "OSD_PARAM_AUX_FUNCTION incorrect");
static_assert((uint32_t)AP_OSD_ParamSetting::Type::FLIGHT_MODE == (uint32_t)OSD_PARAM_FLIGHT_MODE, "OSD_PARAM_FLIGHT_MODE incorrect");
static_assert((uint32_t)AP_OSD_ParamSetting::Type::FAILSAFE_ACTION == (uint32_t)OSD_PARAM_FAILSAFE_ACTION, "OSD_PARAM_FAILSAFE_ACTION incorrect");
static_assert((uint32_t)AP_OSD_ParamSetting::Type::FAILSAFE_ACTION_1 == (uint32_t)OSD_PARAM_FAILSAFE_ACTION_1, "OSD_PARAM_FAILSAFE_ACTION_1 incorrect");
static_assert((uint32_t)AP_OSD_ParamSetting::Type::FAILSAFE_ACTION_2 == (uint32_t)OSD_PARAM_FAILSAFE_ACTION_2, "OSD_PARAM_FAILSAFE_ACTION_2 incorrect");
static_assert((uint32_t)AP_OSD_ParamSetting::Type::NUM_TYPES == (uint32_t)AP_OSD_ParamSetting::Type::NUM_TYPES, "AP_OSD_ParamSetting::Type::NUM_TYPES incorrect");
#endif  // HAL_GCS_ENABLED

#define PARAM_COMPOSITE_INDEX(key, idx, group) (uint32_t((uint32_t(key) << 23) | (uint32_t(idx) << 18) | uint32_t(group)))

#define OSD_PARAM_DEBUG 0
#if OSD_PARAM_DEBUG
#define debug(fmt, args ...) do { hal.console->printf("OSD: " fmt, args); } while (0)
#else
#define debug(fmt, args ...)
#endif

// at the cost of a little flash, we can create much better ranges and values for certain important settings
// common labels - all strings must be upper case

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_COPTER_OR_HELI

struct SerialProtocolMeta : AP_OSD_ParamSetting::ParamMetadata {
    SerialProtocolMeta() : AP_OSD_ParamSetting::ParamMetadata(-1, AP_SerialManager::SerialProtocol_NumProtocols - 1, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch (v) {
#if HAL_GCS_ENABLED
        case AP_SerialManager::SerialProtocol_MAVLink:              return "MAV";
        case AP_SerialManager::SerialProtocol_MAVLink2:             return "MAV2";
#endif
#if AP_FRSKY_D_TELEM_ENABLED
        case AP_SerialManager::SerialProtocol_FrSky_D:              return "FSKY_D";
#endif
#if AP_FRSKY_SPORT_TELEM_ENABLED
        case AP_SerialManager::SerialProtocol_FrSky_SPort:          return "FSKY_S";
#endif
#if AP_GPS_ENABLED
        case AP_SerialManager::SerialProtocol_GPS:                  return "GPS";
#endif
#if HAL_MOUNT_ENABLED
        case AP_SerialManager::SerialProtocol_AlexMos:              return "ALEX";
        case AP_SerialManager::SerialProtocol_Gimbal:               return "STORM";
#endif
#if AP_RANGEFINDER_ENABLED
        case AP_SerialManager::SerialProtocol_Rangefinder:          return "RNG";
#endif
#if AP_FRSKY_SPORT_TELEM_ENABLED
        case AP_SerialManager::SerialProtocol_FrSky_SPort_Passthrough: return "FSKY_TX";
#endif
#if HAL_PROXIMITY_ENABLED
        case AP_SerialManager::SerialProtocol_Lidar360:             return "LID360";
#endif
#if AP_BEACON_ENABLED
        case AP_SerialManager::SerialProtocol_Beacon:               return "BEACN";
#endif
#if AP_VOLZ_ENABLED
        case AP_SerialManager::SerialProtocol_Volz:                 return "VOLZ";
#endif
#if AP_SBUSOUTPUT_ENABLED
        case AP_SerialManager::SerialProtocol_Sbus1:                return "SBUS";
#endif
#if HAL_WITH_ESC_TELEM
        case AP_SerialManager::SerialProtocol_ESCTelemetry:         return "ESC_TLM";
#endif
#if AP_DEVO_TELEM_ENABLED
        case AP_SerialManager::SerialProtocol_Devo_Telem:           return "DEV_TLM";
#endif
#if AP_OPTICALFLOW_ENABLED
        case AP_SerialManager::SerialProtocol_OpticalFlow:          return "OPTFLW";
#endif
#if AP_ROBOTISSERVO_ENABLED
        case AP_SerialManager::SerialProtocol_Robotis:              return "RBTSRV";
#endif
#if HAL_NMEA_OUTPUT_ENABLED
        case AP_SerialManager::SerialProtocol_NMEAOutput:           return "NMEA";
#endif
#if AP_WINDVANE_ENABLED
        case AP_SerialManager::SerialProtocol_WindVane:             return "WNDVNE";
#endif
#if HAL_CANMANAGER_ENABLED
        case AP_SerialManager::SerialProtocol_SLCAN:                return "SLCAN";
#endif
        case AP_SerialManager::SerialProtocol_RCIN:                 return "RCIN";
#if HAL_EFI_ENABLED
        case AP_SerialManager::SerialProtocol_EFI:                  return "MGSQRT";
#endif
#if AP_LTM_TELEM_ENABLED
        case AP_SerialManager::SerialProtocol_LTM_Telem:            return "LTM";
#endif
#if AP_CAMERA_RUNCAM_ENABLED
        case AP_SerialManager::SerialProtocol_RunCam:               return "RUNCAM";
#endif
#if HAL_HOTT_TELEM_ENABLED
        case AP_SerialManager::SerialProtocol_Hott:                 return "HOT_TLM";
#endif
#if AP_SCRIPTING_ENABLED
        case AP_SerialManager::SerialProtocol_Scripting:            return "SCRIPT";
#endif
#if AP_CRSF_ENABLED
        case AP_SerialManager::SerialProtocol_CRSF:                 return "CRSF";
#endif
#if HAL_GENERATOR_ENABLED
        case AP_SerialManager::SerialProtocol_Generator:            return "GEN";
#endif
#if AP_WINCH_ENABLED
        case AP_SerialManager::SerialProtocol_Winch:                return "WNCH";
#endif
#if HAL_MSP_ENABLED
        case AP_SerialManager::SerialProtocol_MSP:                  return "MSP";
        case AP_SerialManager::SerialProtocol_DJI_FPV:              return "DJI";
#endif
#if AP_AIRSPEED_ENABLED
        case AP_SerialManager::SerialProtocol_AirSpeed:             return "AIRSPD";
#endif
#if HAL_ADSB_ENABLED
        case AP_SerialManager::SerialProtocol_ADSB:                 return "ADSB";
#endif
#if AP_AHRS_ENABLED
        case AP_SerialManager::SerialProtocol_AHRS:                 return "AHRS";
#endif
#if AP_SMARTAUDIO_ENABLED
        case AP_SerialManager::SerialProtocol_SmartAudio:           return "AUDIO";
#endif
#if AP_FETTEC_ONEWIRE_ENABLED
        case AP_SerialManager::SerialProtocol_FETtecOneWire:        return "FETTEC";
#endif
#if HAL_TORQEEDO_ENABLED
        case AP_SerialManager::SerialProtocol_Torqeedo:             return "TORQ";
#endif
#if AP_AIS_ENABLED
        case AP_SerialManager::SerialProtocol_AIS:                  return "AIS";
#endif
        case AP_SerialManager::SerialProtocol_CoDevESC:             return "CD_ESC";
#if HAL_WITH_MSP_DISPLAYPORT
        case AP_SerialManager::SerialProtocol_MSP_DisplayPort:      return "MSP_DP";
#endif
#if HAL_GCS_ENABLED
        case AP_SerialManager::SerialProtocol_MAVLinkHL:            return "MAV_HL";
#endif
#if AP_TRAMP_ENABLED
        case AP_SerialManager::SerialProtocol_Tramp:                return "TRAMP";
#endif
#if AP_DDS_ENABLED
        case AP_SerialManager::SerialProtocol_DDS_XRCE:             return "DDS";
#endif
#if AP_SERIALMANAGER_IMUOUT_ENABLED
        case AP_SerialManager::SerialProtocol_IMUOUT:               return "IMUOUT";
#endif
#if AP_NETWORKING_BACKEND_PPP
        case AP_SerialManager::SerialProtocol_PPP:                  return "PPP";
#endif
#if AP_IBUS_TELEM_ENABLED
        case AP_SerialManager::SerialProtocol_IBUS_Telem:           return "IBUS_TLM";
#endif
#if HAL_WITH_IO_MCU
        case AP_SerialManager::SerialProtocol_IOMCU:                return "IOMCU";
#endif
        default: return nullptr;
        }
    }
};
static const SerialProtocolMeta meta_serial_protocol;

struct ServoFunctionMeta : AP_OSD_ParamSetting::ParamMetadata {
    ServoFunctionMeta() : AP_OSD_ParamSetting::ParamMetadata(0, SRV_Channel::k_nr_aux_servo_functions - 1, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch (v) {
        case SRV_Channel::k_none:                return "NONE";
        case SRV_Channel::k_manual:              return "RCPASS";
        case SRV_Channel::k_flap:                return "FLAP";
        case SRV_Channel::k_flap_auto:           return "FLAP_AUTO";
        case SRV_Channel::k_aileron:             return "AIL";
#if HAL_MOUNT_ENABLED
        case SRV_Channel::k_mount_pan:           return "MNT_PAN";
        case SRV_Channel::k_mount_tilt:          return "MNT_TLT";
        case SRV_Channel::k_mount_roll:          return "MNT_RLL";
        case SRV_Channel::k_mount_open:          return "MNT_OPEN";
#endif
#if AP_CAMERA_ENABLED
        case SRV_Channel::k_cam_trigger:         return "CAM_TRG";
#endif
#if HAL_MOUNT_ENABLED
        case SRV_Channel::k_mount2_pan:          return "MNT2_PAN";
        case SRV_Channel::k_mount2_tilt:         return "MNT2_TLT";
        case SRV_Channel::k_mount2_roll:         return "MNT2_RLL";
        case SRV_Channel::k_mount2_open:         return "MNT2_OPEN";
#endif
        case SRV_Channel::k_dspoilerLeft1:       return "DIF_SPL_L1";
        case SRV_Channel::k_dspoilerRight1:      return "DIF_SPL_R1";
        case SRV_Channel::k_elevator:            return "ELE";
        case SRV_Channel::k_rudder:              return "RUD";
#if HAL_SPRAYER_ENABLED
        case SRV_Channel::k_sprayer_pump:        return "SPR_PMP";
        case SRV_Channel::k_sprayer_spinner:     return "SPR_SPIN";
#endif
        case SRV_Channel::k_flaperon_left:       return "FLPRON_L";
        case SRV_Channel::k_flaperon_right:      return "FLPRON_R";
        case SRV_Channel::k_steering:            return "GRND_STEER";
#if HAL_PARACHUTE_ENABLED
        case SRV_Channel::k_parachute_release:   return "PARACHT";
#endif
#if AP_GRIPPER_ENABLED
        case SRV_Channel::k_gripper:             return "GRIP";
#endif
#if AP_LANDINGGEAR_ENABLED
        case SRV_Channel::k_landing_gear_control: return "GEAR";
#endif
#if AP_ICENGINE_ENABLED
        case SRV_Channel::k_engine_run_enable:   return "ENG_RUN_EN";
#endif
#if APM_BUILD_COPTER_OR_HELI
        case SRV_Channel::k_heli_rsc:            return "HELI_RSC";
        case SRV_Channel::k_heli_tail_rsc:       return "HELI_TAIL_RSC";
#endif
        case SRV_Channel::k_motor1:              return "MOT_1";
        case SRV_Channel::k_motor2:              return "MOT_2";
        case SRV_Channel::k_motor3:              return "MOT_3";
        case SRV_Channel::k_motor4:              return "MOT_4";
        case SRV_Channel::k_motor5:              return "MOT_5";
        case SRV_Channel::k_motor6:              return "MOT_6";
        case SRV_Channel::k_motor7:              return "MOT_7";
        case SRV_Channel::k_motor8:              return "MOT_8";
        case SRV_Channel::k_motor_tilt:          return "MOT_TLT";
        case SRV_Channel::k_rcin1:               return "RCIN_1";
        case SRV_Channel::k_rcin2:               return "RCIN_2";
        case SRV_Channel::k_rcin3:               return "RCIN_3";
        case SRV_Channel::k_rcin4:               return "RCIN_4";
        case SRV_Channel::k_rcin5:               return "RCIN_5";
        case SRV_Channel::k_rcin6:               return "RCIN_6";
        case SRV_Channel::k_rcin7:               return "RCIN_7";
        case SRV_Channel::k_rcin8:               return "RCIN_8";
        case SRV_Channel::k_rcin9:               return "RCIN_9";
        case SRV_Channel::k_rcin10:              return "RCIN_10";
        case SRV_Channel::k_rcin11:              return "RCIN_11";
        case SRV_Channel::k_rcin12:              return "RCIN_12";
        case SRV_Channel::k_rcin13:              return "RCIN_13";
        case SRV_Channel::k_rcin14:              return "RCIN_14";
        case SRV_Channel::k_rcin15:              return "RCIN_15";
        case SRV_Channel::k_rcin16:              return "RCIN_16";
#if AP_ICENGINE_ENABLED
        case SRV_Channel::k_ignition:            return "IGN";
        case SRV_Channel::k_starter:             return "START";
#endif
        case SRV_Channel::k_throttle:            return "THR";
        case SRV_Channel::k_tracker_yaw:         return "TRCK_YAW";
        case SRV_Channel::k_tracker_pitch:       return "TRCK_PIT";
        case SRV_Channel::k_throttleLeft:        return "THR_L";
        case SRV_Channel::k_throttleRight:       return "THR_R";
        case SRV_Channel::k_tiltMotorLeft:       return "TLTMOT_L";
        case SRV_Channel::k_tiltMotorRight:      return "TLTMOT_R";
        case SRV_Channel::k_elevon_left:         return "ELEVN_L";
        case SRV_Channel::k_elevon_right:        return "ELEVN_R";
        case SRV_Channel::k_vtail_left:          return "VTAIL_L";
        case SRV_Channel::k_vtail_right:         return "VTAIL_R";
#if APM_BUILD_COPTER_OR_HELI
        case SRV_Channel::k_boost_throttle:      return "BOOST_THR";
#endif
        case SRV_Channel::k_motor9:              return "MOT_9";
        case SRV_Channel::k_motor10:             return "MOT_10";
        case SRV_Channel::k_motor11:             return "MOT_11";
        case SRV_Channel::k_motor12:             return "MOT_12";
        case SRV_Channel::k_dspoilerLeft2:       return "DIF_SPL_L2";
        case SRV_Channel::k_dspoilerRight2:      return "DIF_SPL_R2";
        case SRV_Channel::k_mainsail_sheet:      return "MAIN_SAIL";
#if AP_CAMERA_ENABLED
        case SRV_Channel::k_cam_iso:             return "CAM_ISO";
        case SRV_Channel::k_cam_aperture:        return "CAM_APTR";
        case SRV_Channel::k_cam_focus:           return "CAM_FOC";
        case SRV_Channel::k_cam_shutter_speed:   return "CAM_SH_SPD";
#endif
#if AP_SCRIPTING_ENABLED
        case SRV_Channel::k_scripting1:          return "SCRPT_1";
        case SRV_Channel::k_scripting2:          return "SCRPT_2";
        case SRV_Channel::k_scripting3:          return "SCRPT_3";
        case SRV_Channel::k_scripting4:          return "SCRPT_4";
        case SRV_Channel::k_scripting5:          return "SCRPT_5";
        case SRV_Channel::k_scripting6:          return "SCRPT_6";
        case SRV_Channel::k_scripting7:          return "SCRPT_7";
        case SRV_Channel::k_scripting8:          return "SCRPT_8";
        case SRV_Channel::k_scripting9:          return "SCRPT_9";
        case SRV_Channel::k_scripting10:         return "SCRPT_10";
        case SRV_Channel::k_scripting11:         return "SCRPT_11";
        case SRV_Channel::k_scripting12:         return "SCRPT_12";
        case SRV_Channel::k_scripting13:         return "SCRPT_13";
        case SRV_Channel::k_scripting14:         return "SCRPT_14";
        case SRV_Channel::k_scripting15:         return "SCRPT_15";
        case SRV_Channel::k_scripting16:         return "SCRPT_16";
#endif
#if AP_NOTIFY_NEOPIXEL_ENABLED
        case SRV_Channel::k_LED_neopixel1:       return "NEOPX_1";
        case SRV_Channel::k_LED_neopixel2:       return "NEOPX_2";
        case SRV_Channel::k_LED_neopixel3:       return "NEOPX_3";
        case SRV_Channel::k_LED_neopixel4:       return "NEOPX_4";
#endif
        case SRV_Channel::k_roll_out:            return "RAT_RLL";
        case SRV_Channel::k_pitch_out:           return "RAT_PIT";
        case SRV_Channel::k_thrust_out:          return "RAT_THRST";
        case SRV_Channel::k_yaw_out:             return "RAT_YAW";
        case SRV_Channel::k_wingsail_elevator:   return "WSAIL_EL";
#if AP_NOTIFY_PROFILED_ENABLED
        case SRV_Channel::k_ProfiLED_1:          return "PRLED_1";
        case SRV_Channel::k_ProfiLED_2:          return "PRLED_2";
        case SRV_Channel::k_ProfiLED_3:          return "PRLED_3";
        case SRV_Channel::k_ProfiLED_Clock:      return "PRLED_CLK";
#endif
#if AP_WINCH_ENABLED
        case SRV_Channel::k_winch_clutch:        return "WNCH_CL";
#endif
        default:  return nullptr;
        }
    }
};
static const ServoFunctionMeta meta_servo_function;

#endif  // APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_COPTER_OR_HELI

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)

struct PlaneAuxOptionsMeta : AP_OSD_ParamSetting::ParamMetadata {
    PlaneAuxOptionsMeta() : AP_OSD_ParamSetting::ParamMetadata(0, 105, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch (RC_Channel::AUX_FUNC(v)) {
        case RC_Channel::AUX_FUNC::DO_NOTHING:          return "NONE";
        case RC_Channel::AUX_FUNC::RTL:                 return "RTL";
#if AP_CAMERA_ENABLED
        case RC_Channel::AUX_FUNC::CAMERA_TRIGGER:      return "CAM_TRG";
#endif
        case RC_Channel::AUX_FUNC::AUTO:                return "AUTO";
#if AP_MISSION_ENABLED
        case RC_Channel::AUX_FUNC::MISSION_RESET:       return "MIS_RST";
#endif
#if AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED
        case RC_Channel::AUX_FUNC::RELAY:               return "RLY";
#endif
        case RC_Channel::AUX_FUNC::LANDING_GEAR:        return "LAND_GR";
        case RC_Channel::AUX_FUNC::LOST_VEHICLE_SOUND:  return "LOST_SND";
        case RC_Channel::AUX_FUNC::MOTOR_ESTOP:         return "M_ESTOP";
#if AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED
        case RC_Channel::AUX_FUNC::RELAY3:              return "RLY3";
        case RC_Channel::AUX_FUNC::RELAY4:              return "RLY4";
#endif
        case RC_Channel::AUX_FUNC::AVOID_ADSB:          return "OA_ADSB";
        case RC_Channel::AUX_FUNC::ARMDISARM_UNUSED:    return "ARM/DS";
        case RC_Channel::AUX_FUNC::INVERTED:            return "INVERT";
        case RC_Channel::AUX_FUNC::RC_OVERRIDE_ENABLE:  return "RC_OVRD";
        case RC_Channel::AUX_FUNC::MANUAL:              return "MANUAL";
        case RC_Channel::AUX_FUNC::GUIDED:              return "GUIDE";
        case RC_Channel::AUX_FUNC::LOITER:              return "LOIT";
#if AP_MISSION_ENABLED
        case RC_Channel::AUX_FUNC::CLEAR_WP:            return "CLR_WP";
#endif
        case RC_Channel::AUX_FUNC::COMPASS_LEARN:       return "COMP_LRN";
        case RC_Channel::AUX_FUNC::REVERSE_THROTTLE:    return "REV_THR";
#if AP_GPS_ENABLED
        case RC_Channel::AUX_FUNC::GPS_DISABLE:         return "GPS_DIS";
#endif
#if AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED
        case RC_Channel::AUX_FUNC::RELAY5:              return "RLY5";
        case RC_Channel::AUX_FUNC::RELAY6:              return "RLY6";
#endif
        case RC_Channel::AUX_FUNC::CIRCLE:              return "CIRCLE";
        case RC_Channel::AUX_FUNC::TAKEOFF:             return "TAKEOFF";
#if AP_CAMERA_RUNCAM_ENABLED
        case RC_Channel::AUX_FUNC::RUNCAM_CONTROL:      return "RCAM_CTL";
        case RC_Channel::AUX_FUNC::RUNCAM_OSD_CONTROL:  return "RCAM_OSD";
#endif
        case RC_Channel::AUX_FUNC::DISARM:              return "DSARM";
        case RC_Channel::AUX_FUNC::Q_ASSIST:            return "QASS3POS";
        case RC_Channel::AUX_FUNC::AIRMODE:             return "AIR";
#if HAL_GENERATOR_ENABLED
        case RC_Channel::AUX_FUNC::GENERATOR:           return "GEN";
#endif
        case RC_Channel::AUX_FUNC::TER_DISABLE:         return "TER_AUTO";
        case RC_Channel::AUX_FUNC::CROW_SELECT:         return "CROW_SEL";
        case RC_Channel::AUX_FUNC::SOARING:             return "SOAR";
#if AP_INERTIALSENSOR_KILL_IMU_ENABLED
        case RC_Channel::AUX_FUNC::KILL_IMU1:           return "KILLIMU1";
        case RC_Channel::AUX_FUNC::KILL_IMU2:           return "KILLIMU2";
#endif
#if AP_CAMERA_ENABLED
        case RC_Channel::AUX_FUNC::CAM_MODE_TOGGLE:     return "CAM_TOG";
#endif
#if AP_GPS_ENABLED
        case RC_Channel::AUX_FUNC::GPS_DISABLE_YAW:     return "GPSYAW_DIS";
#endif
        default: return nullptr;
        }
    }
};
static const PlaneAuxOptionsMeta meta_aux_options;

struct PlaneFlightModesMeta : AP_OSD_ParamSetting::ParamMetadata {
    PlaneFlightModesMeta() : AP_OSD_ParamSetting::ParamMetadata(0, 25, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch (v) {
        case 0:  return "MAN";
        case 1:  return "CIRC";
        case 2:  return "STAB";
        case 3:  return "TRAIN";
        case 4:  return "ACRO";
        case 5:  return "FBWA";
        case 6:  return "FBWB";
        case 7:  return "CRUISE";
        case 8:  return "ATUNE";
        case 10: return "AUTO";
        case 11: return "RTL";
        case 12: return "LOIT";
        case 13: return "TKOF";
        case 14: return "ADSB";
        case 15: return "GUID";
#if 1  // HAL_QUADPLANE_ENABLED
        case 17: return "QSTAB";
        case 18: return "QHOV";
        case 19: return "QLOIT";
        case 20: return "QLAND";
        case 21: return "QRTL";
        case 22: return "QTUNE";
        case 23: return "QACRO";
#endif
        case 24: return "THRML";
#if 1  // HAL_QUADPLANE_ENABLED — no library-level config, always enabled for Plane builds
        case 25: return "L2QLND";
#endif
        default: return nullptr;
        }
    }
};
static const PlaneFlightModesMeta meta_flt_modes;

struct PlaneFailsafeActionMeta : AP_OSD_ParamSetting::ParamMetadata {
    PlaneFailsafeActionMeta() : AP_OSD_ParamSetting::ParamMetadata(0, 5, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch (v) {
        case 0: return "NONE";
        case 1: return "RTL";
        case 2: return "LAND";
        case 3: return "TERM";
        case 4: return "QLAND";
        case 5: return "PARA";
        default: return nullptr;
        }
    }
};
static const PlaneFailsafeActionMeta meta_fs_act;

struct PlaneShortFailsafeMeta : AP_OSD_ParamSetting::ParamMetadata {
    PlaneShortFailsafeMeta() : AP_OSD_ParamSetting::ParamMetadata(0, 3, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch (v) {
        case 0: return "CRC_NOCHNGE";
        case 1: return "CIRC";
        case 2: return "FBWA";
        case 3: return "DSABLE";
        default: return nullptr;
        }
    }
};
static const PlaneShortFailsafeMeta meta_fs_shrt;

struct PlaneLongFailsafeMeta : AP_OSD_ParamSetting::ParamMetadata {
    PlaneLongFailsafeMeta() : AP_OSD_ParamSetting::ParamMetadata(0, 3, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch (v) {
        case 0: return "CNTNUE";
        case 1: return "RTL";
        case 2: return "GLIDE";
        case 3: return "PARACHT";
        default: return nullptr;
        }
    }
};
static const PlaneLongFailsafeMeta meta_fs_lng;

// plane parameters
const AP_OSD_ParamSetting::ParamMetadata* const AP_OSD_ParamSetting::_param_metadata[] = {
    &meta_serial_protocol,  // SERIAL_PROTOCOL      (1)
    &meta_servo_function,   // SERVO_FUNCTION        (2)
    &meta_aux_options,      // AUX_FUNCTION          (3)
    &meta_flt_modes,        // FLIGHT_MODE           (4)
    &meta_fs_act,           // FAILSAFE_ACTION       (5)
    &meta_fs_shrt,          // FAILSAFE_ACTION_1     (6)
    &meta_fs_lng,           // FAILSAFE_ACTION_2     (7)
};

#elif APM_BUILD_COPTER_OR_HELI

struct CopterAuxOptionsMeta : AP_OSD_ParamSetting::ParamMetadata {
    CopterAuxOptionsMeta() : AP_OSD_ParamSetting::ParamMetadata(0, 105, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch (RC_Channel::AUX_FUNC(v)) {
        case RC_Channel::AUX_FUNC::DO_NOTHING:          return "NONE";
        case RC_Channel::AUX_FUNC::FLIP:                return "FLIP";
        case RC_Channel::AUX_FUNC::SIMPLE_MODE:         return "SIMP";
        case RC_Channel::AUX_FUNC::RTL:                 return "RTL";
        case RC_Channel::AUX_FUNC::SAVE_TRIM:           return "SAV_TRM";
#if AP_MISSION_ENABLED
        case RC_Channel::AUX_FUNC::SAVE_WP:             return "SAV_WP";
#endif
#if AP_CAMERA_ENABLED
        case RC_Channel::AUX_FUNC::CAMERA_TRIGGER:      return "CAM_TRG";
#endif
#if AP_RANGEFINDER_ENABLED
        case RC_Channel::AUX_FUNC::RANGEFINDER:         return "RNG";
#endif
#if AP_FENCE_ENABLED
        case RC_Channel::AUX_FUNC::FENCE:               return "FENCE";
#endif
        case RC_Channel::AUX_FUNC::SUPERSIMPLE_MODE:    return "SSIMP";
        case RC_Channel::AUX_FUNC::ACRO_TRAINER:        return "ACRO_TRN";
#if HAL_SPRAYER_ENABLED
        case RC_Channel::AUX_FUNC::SPRAYER:             return "SPRAY";
#endif
        case RC_Channel::AUX_FUNC::AUTO:                return "AUTO";
        case RC_Channel::AUX_FUNC::AUTOTUNE_MODE:       return "AUTOTN";
        case RC_Channel::AUX_FUNC::LAND:                return "LAND";
#if AP_GRIPPER_ENABLED
        case RC_Channel::AUX_FUNC::GRIPPER:             return "GRIP";
#endif
#if HAL_PARACHUTE_ENABLED
        case RC_Channel::AUX_FUNC::PARACHUTE_ENABLE:    return "CHUTE_EN";
        case RC_Channel::AUX_FUNC::PARACHUTE_RELEASE:   return "CHUTE_RL";
        case RC_Channel::AUX_FUNC::PARACHUTE_3POS:      return "CHUTE_3P";
#endif
#if AP_MISSION_ENABLED
        case RC_Channel::AUX_FUNC::MISSION_RESET:       return "MIS_RST";
#endif
        case RC_Channel::AUX_FUNC::ATTCON_FEEDFWD:      return "ATT_FF";
        case RC_Channel::AUX_FUNC::ATTCON_ACCEL_LIM:    return "ATT_ACC";
#if HAL_MOUNT_ENABLED
        case RC_Channel::AUX_FUNC::RETRACT_MOUNT1:      return "RET_MNT";
#endif
#if AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED
        case RC_Channel::AUX_FUNC::RELAY:               return "RLY";
#endif
        case RC_Channel::AUX_FUNC::LANDING_GEAR:        return "LAND_GR";
        case RC_Channel::AUX_FUNC::LOST_VEHICLE_SOUND:  return "LOST_SND";
        case RC_Channel::AUX_FUNC::MOTOR_ESTOP:         return "M_ESTOP";
        case RC_Channel::AUX_FUNC::MOTOR_INTERLOCK:     return "M_ILOCK";
        case RC_Channel::AUX_FUNC::BRAKE:               return "BRAKE";
#if AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED
        case RC_Channel::AUX_FUNC::RELAY2:              return "RLY2";
        case RC_Channel::AUX_FUNC::RELAY3:              return "RLY3";
        case RC_Channel::AUX_FUNC::RELAY4:              return "RLY4";
#endif
        case RC_Channel::AUX_FUNC::THROW:               return "THROW";
        case RC_Channel::AUX_FUNC::AVOID_ADSB:          return "OA_ADSB";
        case RC_Channel::AUX_FUNC::PRECISION_LOITER:    return "PR_LOIT";
        case RC_Channel::AUX_FUNC::AVOID_PROXIMITY:     return "OA_PROX";
        case RC_Channel::AUX_FUNC::ARMDISARM_UNUSED:    return "ARM/DS";
        case RC_Channel::AUX_FUNC::SMART_RTL:           return "SMRT_RTL";
        case RC_Channel::AUX_FUNC::INVERTED:            return "INVERT";
        case RC_Channel::AUX_FUNC::RC_OVERRIDE_ENABLE:  return "RC_OVRD";
        case RC_Channel::AUX_FUNC::USER_FUNC1:          return "USR1";
        case RC_Channel::AUX_FUNC::USER_FUNC2:          return "USR2";
        case RC_Channel::AUX_FUNC::USER_FUNC3:          return "USR3";
        case RC_Channel::AUX_FUNC::ACRO:                return "ACRO";
        case RC_Channel::AUX_FUNC::GUIDED:              return "GUIDE";
        case RC_Channel::AUX_FUNC::LOITER:              return "LOIT";
        case RC_Channel::AUX_FUNC::FOLLOW:              return "FOLLOW";
#if AP_MISSION_ENABLED
        case RC_Channel::AUX_FUNC::CLEAR_WP:            return "CLR_WP";
#endif
        case RC_Channel::AUX_FUNC::ZIGZAG:              return "ZZAG";
        case RC_Channel::AUX_FUNC::ZIGZAG_SaveWP:       return "ZZ_SVWP";
        case RC_Channel::AUX_FUNC::COMPASS_LEARN:       return "COMP_LRN";
#if AP_GPS_ENABLED
        case RC_Channel::AUX_FUNC::GPS_DISABLE:         return "GPS_DIS";
#endif
#if AP_SERVORELAYEVENTS_ENABLED && AP_RELAY_ENABLED
        case RC_Channel::AUX_FUNC::RELAY5:              return "RLY5";
        case RC_Channel::AUX_FUNC::RELAY6:              return "RLY6";
#endif
        case RC_Channel::AUX_FUNC::STABILIZE:           return "STAB";
        case RC_Channel::AUX_FUNC::POSHOLD:             return "PHOLD";
        case RC_Channel::AUX_FUNC::ALTHOLD:             return "AHOLD";
        case RC_Channel::AUX_FUNC::FLOWHOLD:            return "FHOLD";
        case RC_Channel::AUX_FUNC::CIRCLE:              return "CIRCLE";
        case RC_Channel::AUX_FUNC::DRIFT:               return "DRIFT";
        case RC_Channel::AUX_FUNC::STANDBY:             return "STANDBY";
#if AP_CAMERA_RUNCAM_ENABLED
        case RC_Channel::AUX_FUNC::RUNCAM_CONTROL:      return "RCAM_CTL";
        case RC_Channel::AUX_FUNC::RUNCAM_OSD_CONTROL:  return "RCAM_OSD";
#endif
#if HAL_VISUALODOM_ENABLED
        case RC_Channel::AUX_FUNC::VISODOM_ALIGN:       return "VISO_CAL";
#endif
        case RC_Channel::AUX_FUNC::DISARM:              return "DISARM";
        case RC_Channel::AUX_FUNC::ZIGZAG_Auto:         return "ZZ_Auto";
        case RC_Channel::AUX_FUNC::AIRMODE:             return "AIR";
#if AP_INERTIALSENSOR_KILL_IMU_ENABLED
        case RC_Channel::AUX_FUNC::KILL_IMU1:           return "KILLIMU1";
        case RC_Channel::AUX_FUNC::KILL_IMU2:           return "KILLIMU2";
#endif
#if AP_CAMERA_ENABLED
        case RC_Channel::AUX_FUNC::CAM_MODE_TOGGLE:     return "CAM_MOD_TOG";
#endif
#if AP_GPS_ENABLED
        case RC_Channel::AUX_FUNC::GPS_DISABLE_YAW:     return "GPSYAW_DIS";
#endif
        default: return nullptr;
        }
    }
};
static const CopterAuxOptionsMeta meta_aux_options;

struct CopterFlightModesMeta : AP_OSD_ParamSetting::ParamMetadata {
    CopterFlightModesMeta() : AP_OSD_ParamSetting::ParamMetadata(0, 28, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch (v) {
        case 0:  return "STAB";
        case 1:  return "ACRO";
        case 2:  return "ALTHOLD";
        case 3:  return "AUTO";
        case 4:  return "GUIDED";
        case 5:  return "LOIT";
        case 6:  return "RTL";
        case 7:  return "CIRC";
        case 9:  return "LAND";
        case 11: return "DRFT";
        case 13: return "SPORT";
        case 14: return "FLIP";
        case 15: return "ATUN";
        case 16: return "POSHLD";
        case 17: return "BRAKE";
        case 18: return "THROW";
        case 19: return "AVD_ADSB";
        case 20: return "GUID_NOGPS";
        case 21: return "SMRTRTL";
        case 22: return "FLOHOLD";
        case 23: return "FOLLOW";
        case 24: return "ZIGZAG";
        case 25: return "SYSID";
        case 26: return "HELI_ARO";
        case 27: return "AUTORTL";
        case 28: return "TRTLE";
        default: return nullptr;
        }
    }
};
static const CopterFlightModesMeta meta_flt_modes;

struct CopterFailsafeOptionsMeta : AP_OSD_ParamSetting::ParamMetadata {
    CopterFailsafeOptionsMeta() : AP_OSD_ParamSetting::ParamMetadata(0, 3, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch (v) {
        case 0:  return "NONE";
        case 1:  return "CONT_RCFS";
        case 2:  return "CONT_GCSFS";
        case 3:  return "CONT_RC/GCSFS";
        case 4:  return "CONT_GUID_RC";
        case 8:  return "CONT_LAND";
        case 15: return "CONT_CTRL_GCS";
        case 18: return "CONTNUE";
        default: return nullptr;
        }
    }
};
static const CopterFailsafeOptionsMeta meta_fs_options;

struct CopterFailsafeActionMeta : AP_OSD_ParamSetting::ParamMetadata {
    CopterFailsafeActionMeta() : AP_OSD_ParamSetting::ParamMetadata(0, 5, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch (v) {
        case 0: return "NONE";
        case 1: return "LAND";
        case 2: return "RTL";
        case 3: return "SRTL_RTL";
        case 4: return "SRTL_LAND";
        case 5: return "TERM";
        default: return nullptr;
        }
    }
};
static const CopterFailsafeActionMeta meta_fs_act;

struct CopterThrFailsafeActionMeta : AP_OSD_ParamSetting::ParamMetadata {
    CopterThrFailsafeActionMeta() : AP_OSD_ParamSetting::ParamMetadata(0, 5, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch (v) {
        case 0: return "NONE";
        case 1: return "RTL";
        case 2: return "CONT";
        case 3: return "LAND";
        case 4: return "SRTL_RTL";
        case 5: return "SRTL_LAND";
        default: return nullptr;
        }
    }
};
static const CopterThrFailsafeActionMeta meta_thr_fs_act;

// copter parameters
const AP_OSD_ParamSetting::ParamMetadata* const AP_OSD_ParamSetting::_param_metadata[] = {
    &meta_serial_protocol,  // SERIAL_PROTOCOL      (1)
    &meta_servo_function,   // SERVO_FUNCTION        (2)
    &meta_aux_options,      // AUX_FUNCTION          (3)
    &meta_flt_modes,        // FLIGHT_MODE           (4)
    &meta_fs_options,       // FAILSAFE_ACTION       (5)
    &meta_fs_act,           // FAILSAFE_ACTION_1     (6)
    &meta_thr_fs_act,       // FAILSAFE_ACTION_2     (7)
};

#else
const AP_OSD_ParamSetting::ParamMetadata* const AP_OSD_ParamSetting::_param_metadata[] = {};
#endif

extern const AP_HAL::HAL& hal;

// default constructor that just sets some sensible defaults that exist on all platforms
AP_OSD_ParamSetting::AP_OSD_ParamSetting(uint8_t param_number) :
    _param_number(param_number),
    default_ypos(param_number + 1),
    default_param_group(-1),
    default_param_idx(-1),
    default_param_key(-1)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// construct a setting from a compact static initializer structure
AP_OSD_ParamSetting::AP_OSD_ParamSetting(const Initializer& initializer) :
    _param_number(initializer.index),
    default_enabled(true),
    default_ypos(initializer.index + 1),
    default_param_group(initializer.token.group_element),
    default_param_idx(initializer.token.idx),
    default_param_key(initializer.token.key),
    default_type(float(initializer.type))
{
    AP_Param::setup_object_defaults(this, var_info);
}

// update the contained parameter
void AP_OSD_ParamSetting::update()
{
    // if the user has not made any changes then skip the update
    if (PARAM_TOKEN_INDEX(_current_token) == PARAM_COMPOSITE_INDEX(_param_key, _param_idx, _param_group) && _param_key >= 0) {
        return;
    }
    // if a parameter was configured then use that
    _current_token = AP_Param::ParamToken {};
    // surely there is a more efficient way than brute-force search
    for (_param = AP_Param::first(&_current_token, &_param_type);
        _param && (AP_Param::get_persistent_key(_current_token.key) != uint16_t(_param_key.get())
            || _current_token.idx != uint8_t(_param_idx.get())
            || _current_token.group_element != uint32_t(_param_group.get()));
        _param = AP_Param::next_scalar(&_current_token, &_param_type)) {
    }

    if (_param == nullptr) {
        enabled.set(false);
    } else {
        guess_ranges();
    }
}

// update parameter settings from the named parameter
bool AP_OSD_ParamSetting::set_by_name(const char* name, uint8_t config_type, float pmin, float pmax, float pincr)
{
    AP_Param::ParamToken token = AP_Param::ParamToken {};
    ap_var_type type;
    AP_Param* param = AP_Param::find_by_name(name, &type, &token);

    if (param == nullptr) {
        // leave unchanged
        return false;
    } else {
        _current_token = token;
        _param_type = type;
        _param = param;
        enabled.set_and_save_ifchanged(true);
    }

    _type.set_and_save_ifchanged(config_type);

    if (config_type == uint8_t(Type::NONE) && !is_zero(pincr)) {
        // ranges
        _param_min.set_and_save_ifchanged(pmin);
        _param_max.set_and_save_ifchanged(pmax);
        _param_incr.set_and_save_ifchanged(pincr);
    } else {
        guess_ranges(true);
    }

    _param_key.set_and_save_ifchanged(AP_Param::get_persistent_key(_current_token.key));
    _param_idx.set_and_save_ifchanged(_current_token.idx);
    _param_group.set_and_save_ifchanged(_current_token.group_element);
    return true;
}

// guess the ranges and increment for the selected parameter
// only called when a change has been made
void AP_OSD_ParamSetting::guess_ranges(bool force)
{
    if (_param->is_read_only()) {
        return;
    }

    // check for statically configured setting metadata
    if (set_from_metadata()) {
        return;
    }

    // nothing statically configured so guess some appropriate values
    float min = -1, max = 127, incr = 1;

    if (_param != nullptr) {
        switch (_param_type) {
        case AP_PARAM_INT8:
            break;
        case AP_PARAM_INT16: {
            AP_Int16* p = (AP_Int16*)_param;
            min = -1;
            uint8_t digits = 0;
            for (int16_t int16p = p->get(); int16p > 0; int16p /= 10) {
                digits++;
            }
            incr = MAX(1, powf(10, digits - 2));
            max = powf(10, digits + 1);
            debug("Guessing range for value %d as %f -> %f, %f\n", p->get(), min, max, incr);
            break;
        }
        case AP_PARAM_INT32: {
            AP_Int32* p = (AP_Int32*)_param;
            min = -1;
            uint8_t digits = 0;
            for (int32_t int32p = p->get(); int32p > 0; int32p /= 10) {
                digits++;
            }
            incr = MAX(1, powf(10, digits - 2));
            max = powf(10, digits + 1);
            debug("Guessing range for value %d as %f -> %f, %f\n", int(p->get()), min, max, incr);
            break;
        }
        case AP_PARAM_FLOAT: {
            AP_Float* p = (AP_Float*)_param;

            uint8_t digits = 0;
            for (float floatp = p->get(); floatp > 1.0f; floatp /= 10) {
                digits++;
            }
            float floatp = p->get();
            if (digits < 1) {
                if (!is_zero(floatp)) {
                    incr = floatp * 0.01f; // move in 1% increments
                } else {
                    incr = 0.01f; // move in absolute 1% increments
                }
                max = 1.0;
                min = 0.0f;
            } else {
                if (!is_zero(floatp)) {
                    incr = floatp * 0.01f; // move in 1% increments
                } else {
                    incr = MAX(1, powf(10, digits - 2));
                }
                max = powf(10, digits + 1);
                min = 0.0f;
            }
            debug("Guessing range for value %f as %f -> %f, %f\n", p->get(), min, max, incr);
            break;
        }
        case AP_PARAM_VECTOR3F:
        case AP_PARAM_NONE:
        case AP_PARAM_GROUP:
            break;
        }

        if (force || !_param_min.configured()) {
            _param_min.set(min);
        }
        if (force || !_param_max.configured()) {
            _param_max.set(max);
        }
        if (force || !_param_incr.configured()) {
            _param_incr.set(incr);
        }
    }
}

// copy the name converting FOO_BAR_BAZ to FooBarBaz
void AP_OSD_ParamSetting::copy_name_camel_case(char* name, size_t len) const
{
    char buf[17];
    _param->copy_name_token(_current_token, buf, 17);
    buf[16] = 0;
    name[0] = buf[0];
    for (uint8_t i = 1, n = 1; i < len; i++, n++) {
        if (buf[i] == '_') {
            name[n] = buf[i+1];
            i++;
        } else {
            name[n] = tolower(buf[i]);
        }
    }
}

bool AP_OSD_ParamSetting::set_from_metadata()
{
    if (_type > 0 && uint8_t(_type) <= ARRAY_SIZE(_param_metadata)) {
        const ParamMetadata* m = _param_metadata[_type - 1];
        if (m == nullptr) {
            return false;
        }
        _param_incr.set(m->increment);
        _param_min.set(m->min_value);
        _param_max.set(m->max_value);
        return true;
    }
    return false;
}

// modify the selected parameter values
void AP_OSD_ParamSetting::save_as_new()
{
    _param_group.save();
    _param_key.save();
    _param_idx.save();
    // the user has configured the range and increment, but the parameter
    // is no longer valid so reset these to guessed values
    guess_ranges(true);
    if (_param_min.configured()) {
        _param_min.save();
    }
    if (_param_max.configured()) {
        _param_max.save();
    }
    if (_param_incr.configured()) {
        _param_incr.save();
    }
}

#endif // OSD_PARAM_ENABLED

