-- This script is an example setting up a custom motor matrix mix.  It
-- sets up a 64-motor frame, 8 clockwise-X coaxial octacopters stacked
-- one on top of the other.

-- mkdir -p scripts
-- ln -s ../libraries/AP_Scripting/examples/MotorMatrix_hexacontatetra_octaquad_x.lua scripts/
-- ./Tools/autotest/sim_vehicle.py -v Copter --gdb --debug --map --console
-- param set SCR_ENABLE

local NUM_BANKS = 16

-- helper function duplication of the one found in AP_MotorsMatrix
local function add_motor(motor_num, angle_degrees, yaw_factor, testing_order)
   gcs:send_text(0, string.format("  Adding motor %u (ang=%f)", motor_num, angle_degrees))
   MotorsMatrix:add_motor_raw(
      motor_num,
      math.cos(math.rad(angle_degrees + 90)),
      math.cos(math.rad(angle_degrees)),
      yaw_factor,
      testing_order
   )
end

local fr_yaw_factor = 1  -- front-right yaw factor, switches per level
local motor_num = 0
for i = 1, NUM_BANKS do
   local base = (i-1)*4
   gcs:send_text(0, string.format("Setting up motor bank %u at %u", i, base))
   add_motor(motor_num,     45,  fr_yaw_factor, base+1)
   add_motor(motor_num+1, -135,  fr_yaw_factor, base+3)
   add_motor(motor_num+2,  -45, -fr_yaw_factor, base+4)
   add_motor(motor_num+3,  135, -fr_yaw_factor, base+2)
   fr_yaw_factor = -fr_yaw_factor
   motor_num = motor_num + 4
end

assert(MotorsMatrix:init(NUM_BANKS*4), "Failed to init MotorsMatrix")

motors:set_frame_string(string.format("Motors%u", NUM_BANKS*4))
