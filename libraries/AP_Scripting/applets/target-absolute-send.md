# Target Absolute Send

Sends the TARGET_ABSOLUTE mavlink message to allow other vehicles to follow this one.
TARGET_ABSOLUTE supersedes FOLLOW_TARGET; followers running firmware that only understands FOLLOW_TARGET should use
follow-target-send.lua instead.

The message carries this vehicle's position (altitude AMSL), NED velocity, attitude and body-frame rates from the AHRS.
Acceleration is not sent, and the uncertainty fields are NaN (unknown).

## Parameters

- TGTA_ENABLE : Set to 1 to enable this script
- TGTA_MAV_CHAN : MAVLink channel to which TARGET_ABSOLUTE should be sent

## How To Use

1. copy this script to the autopilot's "scripts" directory
2. within the "scripts" directory create a "modules" directory
3. copy the MAVLink/mavlink_msgs_xxx files to the "scripts" directory
4. the TARGET_ABSOLUTE message will be published at 10hz
