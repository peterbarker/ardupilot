# Target ExternalNav

Locates this vehicle from a target whose global position is known, and sends
that position to the EKF as ExternalNav data.

The target (or a system acting for it) sends TARGET_ABSOLUTE with its global
position, e.g. from GNSS on a landing pad. A sensor on this vehicle, typically
a companion computer tracking a fiducial marker, sends TARGET_RELATIVE with
the target's position relative to the vehicle. Each TARGET_RELATIVE is paired
with the latest TARGET_ABSOLUTE that has the same `id`, and the vehicle's
position is the target's position minus the relative position.

The vehicle's position is only as good as the target's: if the target's GNSS
is 1m out, so is the vehicle. An RTK-positioned target gives centimetre-level
positions.

## Parameters

- TGTN_ENABLE : set to 1 to enable this script
- TGTN_ABS_AGE : a TARGET_RELATIVE is only used if the TARGET_ABSOLUTE with the
  same id arrived within this many seconds of it. If TARGET_ABSOLUTE gives the
  target's velocity, its position is moved forward to the time of the
  TARGET_RELATIVE

## Setup

1. copy this script to the autopilot's "scripts" directory
2. within the "scripts" directory create a "modules" directory
3. copy the MAVLink/mavlink_msgs_xxx files to the "scripts" directory
4. set SCR_ENABLE = 1 and VISO_TYPE = 1 (MAVLink)
5. set EK3_SRC1_POSXY = 6 (ExternalNav), or use a second source set
   (EK3_SRC2_POSXY = 6) and switch to it, e.g. with an RC switch
6. set VISO_POS_X/Y/Z to the position of the sensor on the vehicle
7. the EKF needs an origin before the script can place the target in its
   local frame: from a GPS fix earlier in the flight, or SET_GPS_GLOBAL_ORIGIN

## Notes

- TARGET_RELATIVE may be given in MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_OFFSET_NED
  or MAV_FRAME_BODY_FRD, or in the sensor frame with q_sensor set. Body-frame
  observations are rotated with the EKF's attitude when they arrive, so any EKF
  yaw error multiplied by the range to the target becomes a position error.
- No attitude is supplied to the EKF; do not use ExternalNav as the yaw source
  (EK3_SRCn_YAW = 6) with this script.
- Unknown (NaN) uncertainties are treated as zero, so VISO_POS_M_NSE is the
  floor on the position error given to the EKF.
- target ids must be unique among the targets this vehicle can hear.
