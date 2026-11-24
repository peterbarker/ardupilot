'''
Plane autotest: PlaneDAASmoothingSafety

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAASmoothingSafety(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAASmoothingSafety(self):
        """Production smoothing must reject blocked commands and retain safe anti-flip damping."""
        self.install_planedaa_scripts()
        module = self.installed_script_module_path("daacore.lua")
        with open(module) as f:
            content = f.read()
        anchor = "    self.configure          = configure\n"
        if content.count(anchor) != 1:
            raise NotAchievedException("DAA smoothing test seam changed")
        # Expose a test-only entry inside the installed instance. The production
        # smoothing, hysteresis and slew code run unchanged in the real Lua VM;
        # only the clearance oracle is controlled to exercise narrow boundaries.
        injection = '''    function self.test_smoothing()
        local saved_probe = probe_bearing
        local threat = {distance_m = 100, type = 1}
        local motion = {ttc = 1000, closing_speed = 1, cpa_miss = 200, pass_behind = 0}
        local function setup(last, side, hold, slew, paths)
            current_loc = Location()
            ground_course_deg = 0
            bendy_angle, bendy_ratio = 30, 1.5
            side_hold_s, slew_dps, slew_urg_s = hold, slew, 1
            committed_side_sign, side_flip_pending = side, false
            last_avoid_bearing_deg = last
            now_ms, last_cmd_bearing_ms = uint32_t(10000), uint32_t(9900)
            probe_bearing = function(bearing)
                local key = math.floor(wrap_360(bearing) * 100 + 0.5) / 100
                local path = assert(paths[key], "unexpected probe bearing " .. key)
                return path[1], bearing, path[2] and threat or nil
            end
        end
        local function check(raw, distance, expected, expected_distance, label)
            local bearing, clearance = refine_avoidance_bearing(0, raw, distance, motion, threat, Location())
            assert(math.abs(wrap_180(bearing - expected)) < 0.01, label .. ": wrong heading " .. bearing)
            assert(math.abs(clearance - expected_distance) < 0.01, label .. ": wrong clearance " .. clearance)
            last_avoid_bearing_deg = bearing
            now_ms = now_ms + uint32_t(100)
        end
        setup(60, 1, 5, 0, {[60] = {-20, true}, [300] = {100, false}})
        check(300, 100, 300, 100, "blocked held side")
        for i = 1, 20 do
            local raw = (i % 2 == 0) and 60 or 300
            check(raw, raw == 60 and -20 or 100, 300, 100, "safe replacement must not flip back")
        end
        setup(60, 1, 5, 0, {[60] = {20, false}, [300] = {100, false}})
        for i = 1, 20 do
            local raw = (i % 2 == 0) and 60 or 300
            check(raw, raw == 60 and 20 or 100, 60, 20, "safe side commitment")
        end
        setup(0, 0, 0, 20, {[0] = {-20, true}, [2] = {-10, true}, [90] = {100, false}})
        check(90, 100, 90, 100, "blocked slew intermediate")
        setup(0, 0, 0, 20, {[0] = {10, true}, [2] = {20, false}, [90] = {100, false}})
        check(90, 100, 2, 20, "safe slew remains limited")
        setup(0, 0, 0, 20, {[0] = {-20, true}, [2] = {10, true}, [90] = {100, false}})
        check(90, 100, 90, 100, "margin-blocked slew")
        setup(0, 0, 0, 20, {[0] = {-20, true}, [2] = {-10, true}, [90] = {10, true}})
        check(90, 10, 90, 10, "positive escape from violation")
        setup(0, 0, 0, 20, {[0] = {-20, true}, [90] = {-10, true}})
        check(90, -10, 0, -20, "no clear path must not claim clearance")
        probe_bearing = saved_probe
    end
'''
        with open(module, "w") as f:
            f.write(content.replace(anchor, injection + anchor))
        self.install_script_content("planedaa.lua", '''local function run()
    local core = require("daacore").new({
        obstacles = {OBSTACLE_TYPE = {}}, MAV_SEVERITY = {ERROR = 3},
    })
    local ok, err = pcall(core.test_smoothing)
    if not ok then
        gcs:send_text(0, tostring(err))
        return
    end
    gcs:send_text(6, "DAA smoothing tests passed")
end
return run, 1000
''')
        self.set_parameters({"SCR_ENABLE": 1, "SCR_VM_I_COUNT": 1000000, "AVD_ENABLE": 1})
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_text("DAA smoothing tests passed", check_context=True, timeout=60)
