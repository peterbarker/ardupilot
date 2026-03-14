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
  Plane-specific OSD parameter metadata registration
 */

#include <AP_OSD/AP_OSD_config.h>

#if OSD_PARAM_ENABLED

#include <AP_OSD/AP_OSD.h>

#include <SRV_Channel/SRV_Channel.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Notify/AP_Notify_config.h>
#include <AP_LandingGear/AP_LandingGear_config.h>
#include <AP_ICEngine/AP_ICEngine_config.h>

#include "Plane.h"

struct PlaneServoFunctionMeta : AP_OSD_ParamSetting::ParamMetadata {
    PlaneServoFunctionMeta() : AP_OSD_ParamSetting::ParamMetadata(0, SRV_Channel::k_nr_aux_servo_functions - 1, 1) {}
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
        case SRV_Channel::k_throttleLeft:        return "THR_L";
        case SRV_Channel::k_throttleRight:       return "THR_R";
        case SRV_Channel::k_tiltMotorLeft:       return "TLTMOT_L";
        case SRV_Channel::k_tiltMotorRight:      return "TLTMOT_R";
        case SRV_Channel::k_elevon_left:         return "ELEVN_L";
        case SRV_Channel::k_elevon_right:        return "ELEVN_R";
        case SRV_Channel::k_vtail_left:          return "VTAIL_L";
        case SRV_Channel::k_vtail_right:         return "VTAIL_R";
        case SRV_Channel::k_motor9:              return "MOT_9";
        case SRV_Channel::k_motor10:             return "MOT_10";
        case SRV_Channel::k_motor11:             return "MOT_11";
        case SRV_Channel::k_motor12:             return "MOT_12";
        case SRV_Channel::k_dspoilerLeft2:       return "DIF_SPL_L2";
        case SRV_Channel::k_dspoilerRight2:      return "DIF_SPL_R2";
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
static const PlaneServoFunctionMeta meta_servo_function;

struct PlaneFlightModesMeta : AP_OSD_ParamSetting::ParamMetadata {
    PlaneFlightModesMeta() : AP_OSD_ParamSetting::ParamMetadata(0, 25, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch (Mode::Number(v)) {
        case Mode::Number::MANUAL:        return "MAN";
        case Mode::Number::CIRCLE:        return "CIRC";
        case Mode::Number::STABILIZE:     return "STAB";
        case Mode::Number::TRAINING:      return "TRAIN";
        case Mode::Number::ACRO:          return "ACRO";
        case Mode::Number::FLY_BY_WIRE_A: return "FBWA";
        case Mode::Number::FLY_BY_WIRE_B: return "FBWB";
        case Mode::Number::CRUISE:        return "CRUISE";
        case Mode::Number::AUTOTUNE:      return "ATUNE";
        case Mode::Number::AUTO:          return "AUTO";
        case Mode::Number::RTL:           return "RTL";
        case Mode::Number::LOITER:        return "LOIT";
        case Mode::Number::TAKEOFF:       return "TKOF";
        case Mode::Number::AVOID_ADSB:    return "ADSB";
        case Mode::Number::GUIDED:        return "GUID";
#if HAL_QUADPLANE_ENABLED
        case Mode::Number::QSTABILIZE:    return "QSTAB";
        case Mode::Number::QHOVER:        return "QHOV";
        case Mode::Number::QLOITER:       return "QLOIT";
        case Mode::Number::QLAND:         return "QLAND";
        case Mode::Number::QRTL:          return "QRTL";
#if QAUTOTUNE_ENABLED
        case Mode::Number::QAUTOTUNE:     return "QTUNE";
#endif
        case Mode::Number::QACRO:         return "QACRO";
#endif
        case Mode::Number::THERMAL:       return "THRML";
#if HAL_QUADPLANE_ENABLED
        case Mode::Number::LOITER_ALT_QLAND: return "L2QLND";
#endif
        default: return nullptr;
        }
    }
};
static const PlaneFlightModesMeta meta_flt_modes;

struct PlaneFailsafeActionMeta : AP_OSD_ParamSetting::ParamMetadata {
    PlaneFailsafeActionMeta() : AP_OSD_ParamSetting::ParamMetadata(0, 5, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch ((Plane::Failsafe_Action)v) {
        case Plane::Failsafe_Action_None:           return "NONE";
        case Plane::Failsafe_Action_RTL:            return "RTL";
        case Plane::Failsafe_Action_Land:           return "LAND";
        case Plane::Failsafe_Action_Terminate:      return "TERM";
#if HAL_QUADPLANE_ENABLED
        case Plane::Failsafe_Action_QLand:          return "QLAND";
        case Plane::Failsafe_Action_Loiter_alt_QLand: return "LOIT_QLAND";
#endif
        case Plane::Failsafe_Action_Parachute:      return "PARA";
        case Plane::Failsafe_Action_AUTOLAND_OR_RTL: return "ALND_RTL";
        }
        return nullptr;
    }
};
static const PlaneFailsafeActionMeta meta_fs_act;

struct PlaneShortFailsafeMeta : AP_OSD_ParamSetting::ParamMetadata {
    PlaneShortFailsafeMeta() : AP_OSD_ParamSetting::ParamMetadata(0, 3, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch ((failsafe_action_short)v) {
        case FS_ACTION_SHORT_BESTGUESS: return "CRC_NOCHNGE";
        case FS_ACTION_SHORT_CIRCLE:    return "CIRC";
        case FS_ACTION_SHORT_FBWA:      return "FBWA";
        case FS_ACTION_SHORT_DISABLED:  return "DSABLE";
        case FS_ACTION_SHORT_FBWB:      return "FBWB";
        }
        return nullptr;
    }
};
static const PlaneShortFailsafeMeta meta_fs_shrt;

struct PlaneLongFailsafeMeta : AP_OSD_ParamSetting::ParamMetadata {
    PlaneLongFailsafeMeta() : AP_OSD_ParamSetting::ParamMetadata(0, 3, 1) {}
    const char *name_for_value(uint8_t v) const override {
        switch ((failsafe_action_long)v) {
        case FS_ACTION_LONG_CONTINUE:   return "CNTNUE";
        case FS_ACTION_LONG_RTL:        return "RTL";
        case FS_ACTION_LONG_GLIDE:      return "GLIDE";
        case FS_ACTION_LONG_PARACHUTE:  return "PARACHT";
        case FS_ACTION_LONG_AUTO:       return "AUTO";
        case FS_ACTION_LONG_AUTOLAND:   return "AUTOLND";
        }
        return nullptr;
    }
};
static const PlaneLongFailsafeMeta meta_fs_lng;

void osd_register_plane_metadata()
{
    AP_OSD_ParamSetting::set_metadata(AP_OSD_ParamSetting::Type::SERVO_FUNCTION,    &meta_servo_function);
    AP_OSD_ParamSetting::set_metadata(AP_OSD_ParamSetting::Type::FLIGHT_MODE,       &meta_flt_modes);
    AP_OSD_ParamSetting::set_metadata(AP_OSD_ParamSetting::Type::FAILSAFE_ACTION,   &meta_fs_act);
    AP_OSD_ParamSetting::set_metadata(AP_OSD_ParamSetting::Type::FAILSAFE_ACTION_1, &meta_fs_shrt);
    AP_OSD_ParamSetting::set_metadata(AP_OSD_ParamSetting::Type::FAILSAFE_ACTION_2, &meta_fs_lng);
}

#endif  // OSD_PARAM_ENABLED
