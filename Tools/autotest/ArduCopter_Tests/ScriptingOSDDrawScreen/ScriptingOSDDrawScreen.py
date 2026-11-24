'''
Copter autotest: ScriptingOSDDrawScreen

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptingOSDDrawScreen(TestGroup):
    @testcase(bucket="CopterTests2b")
    def ScriptingOSDDrawScreen(self):
        '''a script's draw_screen(), writes and flush() reach the OSD as one frame'''
        self.set_parameters({
            "SCR_ENABLE": 1,
            "SERIAL5_PROTOCOL": 42,  # MSP DisplayPort
            "OSD_TYPE": 5,           # MSP DisplayPort
        })
        self.install_script_content_context("osd_draw_screen.lua", """
local function update()
    osd:draw_screen()
    osd:write(1, 1, "SCRIPTED")
    osd:flush()
    return update, 100
end
return update()
""")
        port = self.spare_network_port()
        self.customise_SITL_commandline([
            "--serial5=tcp:%u" % port
        ])
        msp = self.msp_connect(port)

        MSP_DISPLAYPORT = 182
        CLEAR_SCREEN = 2
        WRITE_STRING = 3
        DRAW_SCREEN = 4
        frames = []
        writes = []
        cleared = False

        def collect(cmd, data):
            nonlocal cleared
            if cmd != MSP_DISPLAYPORT or len(data) == 0:
                return
            if data[0] == CLEAR_SCREEN:
                writes.clear()
                cleared = True
            elif not cleared:
                # joined part way through a frame
                return
            elif data[0] == WRITE_STRING:
                writes.append(bytes(data[4:]))
            elif data[0] == DRAW_SCREEN:
                frames.append(list(writes))
        msp.callback = collect
        tstart = self.get_sim_time()
        try:
            while self.get_sim_time_cached() - tstart < 5:
                msp.update()
                self.drain_mav(quiet=True)
        finally:
            msp.callback = None

        if len(frames) < 20:
            raise NotAchievedException("only %u OSD frames drawn" % len(frames))
        without_script = [f for f in frames if not any(b"SCRIPTED" in w for w in f)]
        if len(without_script) > 0:
            raise NotAchievedException("%u of %u OSD frames drawn without the script's text" %
                                       (len(without_script), len(frames)))
        script_only = [f for f in frames if len(f) < 2]
        if len(script_only) > 0:
            raise NotAchievedException("%u of %u OSD frames carry the script's text without the normal screen" %
                                       (len(script_only), len(frames)))
        self.progress("%u OSD frames, every one with the normal screen and the script's text" % len(frames))
