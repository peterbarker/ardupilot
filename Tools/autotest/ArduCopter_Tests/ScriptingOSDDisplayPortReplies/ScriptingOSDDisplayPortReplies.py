'''
Copter autotest: ScriptingOSDDisplayPortReplies

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptingOSDDisplayPortReplies(TestGroup):
    @testcase(bucket="CopterTests2b")
    def ScriptingOSDDisplayPortReplies(self):
        '''a VTX on MSP DisplayPort is still answered while a script draws the OSD'''
        # A VTX such as a DJI O4 polls MSP_STATUS for the arm bit, and only the OSD
        # thread can read the DisplayPort uart, so it must answer while a script holds
        # the OSD. SITL lets any thread read a uart, so this fails when nothing answers
        # but cannot see a build that answers from the script's own thread.
        self.context_collect('STATUSTEXT')
        self.install_script_content_context("osd_draw.lua", '''
local function update()
    osd:draw_screen()
    osd:write(1, 1, "SCRIPT")
    osd:flush()
    return update, 100
end
gcs:send_text(6, "osd draw script running")
return update()
''')
        self.set_parameters({
            "SCR_ENABLE": 1,
            "SERIAL5_PROTOCOL": 42,  # MSP DisplayPort
            "OSD_TYPE": 5,           # MSP DisplayPort
        })
        port = self.spare_network_port()
        self.customise_SITL_commandline(["--serial5=tcp:%u" % port])
        self.wait_statustext("osd draw script running", check_context=True)
        self.wait_ready_to_arm()
        # faster than real time this client cannot drain the redraws, and a reply that
        # does not fit in the transmit buffer is dropped
        self.context_set_speedup(1)
        msp = self.msp_connect(port)
        arm_bits = []
        script_draws = []

        def collect(cmd, data):
            if cmd == 101 and len(data) >= 10:  # MSP_STATUS flight mode flags, bit 0 armed
                arm_bits.append(int.from_bytes(bytes(data[6:10]), 'little') & 1)
            if cmd == 182 and len(data) > 4 and data[0] == 3 and bytes(data[4:]).startswith(b"SCRIPT"):
                script_draws.append(1)  # MSP_DISPLAYPORT write string, from the script

        msp.callback = collect

        def arm_bit():
            # the last reply of the window, as the first may answer a poll sent before
            del arm_bits[:]
            del script_draws[:]
            tstart = self.get_sim_time()
            next_poll = tstart
            while self.get_sim_time_cached() - tstart < 3:
                if self.get_sim_time_cached() >= next_poll:
                    msp.send_command(101)
                    next_poll += 0.1
                msp.update()
                self.drain_mav()
            # the script must still hold the OSD, or the normal OSD would be answering
            if len(script_draws) < 10:
                raise NotAchievedException("the script drew %u times in 3 s" % len(script_draws))
            if len(arm_bits) < 10:
                raise NotAchievedException("%u MSP_STATUS replies in 3 s while a script draws the OSD" % len(arm_bits))
            return arm_bits[-1]

        if arm_bit() != 0:
            raise NotAchievedException("arm bit set while disarmed")
        self.change_mode('STABILIZE')
        self.arm_vehicle()
        if arm_bit() != 1:
            raise NotAchievedException("arm bit not set while armed")
        self.disarm_vehicle()
        if arm_bit() != 0:
            raise NotAchievedException("arm bit still set after disarming")
        msp.callback = None
