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
 */

/*
  Copter-specific OSD parameter metadata registration
 */

#include <AP_OSD/AP_OSD_config.h>

#if OSD_PARAM_ENABLED

#include <AP_OSD/AP_OSD.h>

#include <SRV_Channel/SRV_Channel.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Notify/AP_Notify_config.h>
#include <AP_LandingGear/AP_LandingGear_config.h>
#include "Copter.h"

struct CopterServoFunctionMeta : AP_OSD_ParamSetting::ParamMetadata {
    CopterServoFunctionMeta() : AP_OSD_ParamSetting::ParamMetadata(0, SRV_Channel::k_nr_aux_servo_functions - 1, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch (v) {
        case SRV_Channel::k_none:                return "NONE";
        case SRV_Channel::k_manual:              return "RCPASS";
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
#if HAL_SPRAYER_ENABLED
        case SRV_Channel::k_sprayer_pump:        return "SPR_PMP";
        case SRV_Channel::k_sprayer_spinner:     return "SPR_SPIN";
#endif
#if HAL_PARACHUTE_ENABLED
        case SRV_Channel::k_parachute_release:   return "PARACHT";
#endif
#if AP_GRIPPER_ENABLED
        case SRV_Channel::k_gripper:             return "GRIP";
#endif
#if AP_LANDINGGEAR_ENABLED
        case SRV_Channel::k_landing_gear_control: return "GEAR";
#endif
        case SRV_Channel::k_heli_rsc:            return "HELI_RSC";
        case SRV_Channel::k_heli_tail_rsc:       return "HELI_TAIL_RSC";
        case SRV_Channel::k_motor1:              return "MOT_1";
        case SRV_Channel::k_motor2:              return "MOT_2";
        case SRV_Channel::k_motor3:              return "MOT_3";
        case SRV_Channel::k_motor4:              return "MOT_4";
        case SRV_Channel::k_motor5:              return "MOT_5";
        case SRV_Channel::k_motor6:              return "MOT_6";
        case SRV_Channel::k_motor7:              return "MOT_7";
        case SRV_Channel::k_motor8:              return "MOT_8";
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
        case SRV_Channel::k_throttle:            return "THR";
        case SRV_Channel::k_tiltMotorLeft:       return "TLTMOT_L";
        case SRV_Channel::k_tiltMotorRight:      return "TLTMOT_R";
        case SRV_Channel::k_boost_throttle:      return "BOOST_THR";
        case SRV_Channel::k_motor9:              return "MOT_9";
        case SRV_Channel::k_motor10:             return "MOT_10";
        case SRV_Channel::k_motor11:             return "MOT_11";
        case SRV_Channel::k_motor12:             return "MOT_12";
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
static const CopterServoFunctionMeta meta_servo_function;

struct CopterFlightModesMeta : AP_OSD_ParamSetting::ParamMetadata {
    CopterFlightModesMeta() : AP_OSD_ParamSetting::ParamMetadata(0, 28, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch (Mode::Number(v)) {
        case Mode::Number::STABILIZE:                  return "STAB";
#if MODE_ACRO_ENABLED
        case Mode::Number::ACRO:                       return "ACRO";
#endif
        case Mode::Number::ALT_HOLD:                   return "ALTHOLD";
#if MODE_AUTO_ENABLED
        case Mode::Number::AUTO:                       return "AUTO";
        case Mode::Number::AUTO_RTL:                   return "AUTORTL";
#endif
#if MODE_GUIDED_ENABLED
        case Mode::Number::GUIDED:                     return "GUIDED";
#endif
#if MODE_LOITER_ENABLED
        case Mode::Number::LOITER:                     return "LOIT";
#endif
#if MODE_RTL_ENABLED
        case Mode::Number::RTL:                        return "RTL";
#endif
#if MODE_CIRCLE_ENABLED
        case Mode::Number::CIRCLE:                     return "CIRC";
#endif
        case Mode::Number::LAND:                       return "LAND";
#if MODE_DRIFT_ENABLED
        case Mode::Number::DRIFT:                      return "DRFT";
#endif
#if MODE_SPORT_ENABLED
        case Mode::Number::SPORT:                      return "SPORT";
#endif
#if MODE_FLIP_ENABLED
        case Mode::Number::FLIP:                       return "FLIP";
#endif
#if AUTOTUNE_ENABLED
        case Mode::Number::AUTOTUNE:                   return "ATUN";
#endif
#if MODE_POSHOLD_ENABLED
        case Mode::Number::POSHOLD:                    return "POSHLD";
#endif
#if MODE_BRAKE_ENABLED
        case Mode::Number::BRAKE:                      return "BRAKE";
#endif
#if MODE_THROW_ENABLED
        case Mode::Number::THROW:                      return "THROW";
#endif
#if AP_ADSB_AVOIDANCE_ENABLED
        case Mode::Number::AVOID_ADSB:                 return "AVD_ADSB";
#endif
#if MODE_GUIDED_NOGPS_ENABLED
        case Mode::Number::GUIDED_NOGPS:               return "GUID_NOGPS";
#endif
#if MODE_SMARTRTL_ENABLED
        case Mode::Number::SMART_RTL:                  return "SMRTRTL";
#endif
#if MODE_FLOWHOLD_ENABLED
        case Mode::Number::FLOWHOLD:                   return "FLOHOLD";
#endif
#if MODE_FOLLOW_ENABLED
        case Mode::Number::FOLLOW:                     return "FOLLOW";
#endif
#if MODE_ZIGZAG_ENABLED
        case Mode::Number::ZIGZAG:                     return "ZIGZAG";
#endif
#if MODE_SYSTEMID_ENABLED
        case Mode::Number::SYSTEMID:                   return "SYSID";
#endif
#if MODE_AUTOROTATE_ENABLED
        case Mode::Number::AUTOROTATE:                 return "HELI_ARO";
#endif
#if MODE_TURTLE_ENABLED
        case Mode::Number::TURTLE:                     return "TRTLE";
#endif
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
        switch ((Copter::FailsafeAction)v) {
        case Copter::FailsafeAction::NONE:               return "NONE";
        case Copter::FailsafeAction::LAND:               return "LAND";
        case Copter::FailsafeAction::RTL:                return "RTL";
        case Copter::FailsafeAction::SMARTRTL:           return "SRTL_RTL";
        case Copter::FailsafeAction::SMARTRTL_LAND:      return "SRTL_LAND";
        case Copter::FailsafeAction::TERMINATE:          return "TERM";
        case Copter::FailsafeAction::AUTO_DO_LAND_START: return "DO_LAND_ST";
        case Copter::FailsafeAction::BRAKE_LAND:         return "BRK_LAND";
        }
        return nullptr;
    }
};
static const CopterFailsafeActionMeta meta_fs_act;

struct CopterThrFailsafeActionMeta : AP_OSD_ParamSetting::ParamMetadata {
    CopterThrFailsafeActionMeta() : AP_OSD_ParamSetting::ParamMetadata(0, 5, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch (v) {
        case FS_THR_DISABLED:                        return "NONE";
        case FS_THR_ENABLED_ALWAYS_RTL:              return "RTL";
        case FS_THR_ENABLED_CONTINUE_MISSION:        return "CONT";
        case FS_THR_ENABLED_ALWAYS_LAND:             return "LAND";
        case FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_RTL:  return "SRTL_RTL";
        case FS_THR_ENABLED_ALWAYS_SMARTRTL_OR_LAND: return "SRTL_LAND";
        }
        return nullptr;
    }
};
static const CopterThrFailsafeActionMeta meta_thr_fs_act;

void osd_register_copter_metadata()
{
    AP_OSD_ParamSetting::set_metadata(AP_OSD_ParamSetting::Type::SERVO_FUNCTION,    &meta_servo_function);
    AP_OSD_ParamSetting::set_metadata(AP_OSD_ParamSetting::Type::FLIGHT_MODE,       &meta_flt_modes);
    AP_OSD_ParamSetting::set_metadata(AP_OSD_ParamSetting::Type::FAILSAFE_ACTION,   &meta_fs_options);
    AP_OSD_ParamSetting::set_metadata(AP_OSD_ParamSetting::Type::FAILSAFE_ACTION_1, &meta_fs_act);
    AP_OSD_ParamSetting::set_metadata(AP_OSD_ParamSetting::Type::FAILSAFE_ACTION_2, &meta_thr_fs_act);
}

#endif  // OSD_PARAM_ENABLED
