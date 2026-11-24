'''
Plane autotest: ScriptingTerrainCorrected

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase

from arduplane import SITL_START_HEADING
from arduplane import SITL_START_LOCATION


class ScriptingTerrainCorrected(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def ScriptingTerrainCorrected(self):
        '''check terrain:height_amsl() applies the arming reference offset when asked to'''
        # start well above the SRTM height so arming creates a large reference offset
        start = SITL_START_LOCATION
        home_alt = start.get_alt_m(AltFrame.ABSOLUTE) + 20
        self.customise_SITL_commandline([
            "--home", f"{start.lat:.7f},{start.lng:.7f},{home_alt:.2f},{SITL_START_HEADING:.1f}",
        ])
        self.install_terrain_handlers_context()
        self.install_script_content_context("terrain-corrected.lua", f"""
local loc = Location()
loc:lat({int(start.lat * 1e7)})
loc:lng({int(start.lng * 1e7)})

function update()
  local raw = terrain:height_amsl(loc, false)
  local corrected = terrain:height_amsl(loc, true)
  if raw and corrected then
    gcs:send_named_float("TER_RAW", raw)
    gcs:send_named_float("TER_COR", corrected)
  end
  return update, 500
end

return update()
""")
        self.set_parameters({
            "SCR_ENABLE": 1,
            "TERRAIN_OFS_MAX": 30,
        })
        self.reboot_sitl()
        self.wait_ready_to_arm()

        def script_offset():
            raw = self.assert_receive_named_value_float("TER_RAW", timeout=60).value
            corrected = self.assert_receive_named_value_float("TER_COR").value
            return raw, corrected - raw

        raw, offset = script_offset()
        if abs(offset) > 0.01:
            raise NotAchievedException(f"Offset before arming (got={offset:f})")

        self.arm_vehicle()
        # TERRAIN_REPORT carries the corrected height (C++ default)
        want = self.get_terrain_height_at(start) - raw
        if want < 15:
            raise NotAchievedException(f"Reference offset too small to test (got={want:f})")
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise NotAchievedException(f"Script offset want={want:f} got={offset:f}")
            raw, offset = script_offset()
            if abs(offset - want) < 0.01:
                break
        self.disarm_vehicle()
