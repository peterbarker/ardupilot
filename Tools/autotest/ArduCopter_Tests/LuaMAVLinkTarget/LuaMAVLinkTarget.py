'''
Copter autotest: LuaMAVLinkTarget

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class LuaMAVLinkTarget(TestGroup):
    @testcase(bucket="CopterTests2c")
    def LuaMAVLinkTarget(self):
        """Lua sends legacy, broadcast and full-width targets without mutating payloads."""
        self.set_parameter('SCR_ENABLE', 1)
        self.install_mavlink_module_context('MAVLink')
        self.install_script_content_context('mavlink-target.lua', """
local msgs = require('MAVLink/mavlink_msgs')
mavlink:init(4, 1)
mavlink:register_rx_msgid(76)
mavlink:block_command(31000)
local targets = {false, false, 0, 7, 255, 256, 70000, 0x7fffffff,
                 0x80000000, 0xffffffff, uint32_t(0xffffffff), 256}
local function update()
    local raw, chan = mavlink:receive_chan()
    if raw then
        local msg = msgs.decode(raw, {[76]='COMMAND_LONG'})
        if msg and msg.command == 31000 then
            local case = math.floor(msg.param1)
            local payload = string.pack('<HBBi4BB', 31000, 0, 0, case, 99, 190)
            if case == 12 then payload = payload:sub(1, 8) end
            local saved = payload
            -- Messages without a target field cannot use this override.
            assert(not pcall(mavlink.send_chan, mavlink, chan, 0, string.rep('x', 9), 256))
            local sent
            if case == 1 then
                sent = mavlink:send_chan(chan, 77, payload)
            elseif case == 2 then
                sent = mavlink:send_chan(chan, 77, payload, nil)
            else
                sent = mavlink:send_chan(chan, 77, payload, targets[case])
            end
            assert(sent and payload == saved)
        end
    end
    return update, 20
end
return update()
""")
        self.reboot_sitl()
        self.wait_ready_to_arm()
        targets = (99, 99, 0, 7, 255, 256, 70000, 0x7FFFFFFF,
                   0x80000000, 0xFFFFFFFF, 0xFFFFFFFF, 256)
        for case, target in enumerate(targets, 1):
            self.start_subtest("Lua target case %u: %u" % (case, target))
            self.mav.mav.command_long_send(self.sysid_thismav(), 1, 31000, 0, case, 0, 0, 0, 0, 0, 0)
            reply = self.assert_received_message_field_values('COMMAND_ACK', {
                'command': 31000,
                'result': mavutil.mavlink.MAV_RESULT_ACCEPTED,
                'result_param2': case,
                'target_system': target,
                'target_component': 0 if case == 12 else 190,
            })
            wide = target > 255
            if bool(reply.get_header().incompat_flags & mavutil.mavlink.MAVLINK_IFLAG_TARGET32) != wide:
                raise NotAchievedException("Incorrect Lua target header")
            if reply.get_payload().ljust(10, b'\0')[8] != (255 if wide else target):
                raise NotAchievedException("Incorrect Lua payload target")
