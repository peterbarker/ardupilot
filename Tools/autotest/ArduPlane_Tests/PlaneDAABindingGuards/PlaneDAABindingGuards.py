'''
Plane autotest: PlaneDAABindingGuards

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAABindingGuards(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAABindingGuards(self):
        """Lua vehicle APIs must work with and without the optional DAA bindings."""
        self.install_script_content_context("daa-binding-guards.lua", """
local SCRIPT_NAME = "DAA Binding Guard Test"
local SCRIPT_NAME_SHORT = "DAABnd"
local SCRIPT_VERSION = "1.0.0"

local function update()
    if not ahrs:home_is_set() then
        return update, 100
    end
    -- This shared vehicle API must remain usable even without scripted avoidance.
    assert(vehicle:set_crosstrack_start(ahrs:get_home()), "shared crosstrack API failed")
    local enabled = param:get("AVD_GND_ALT") ~= nil
    assert((OAScripting ~= nil) == enabled, "DAA singleton/parameter guards disagree")
    if enabled then
        assert(type(OAScripting.find_threats) == "function", "DAA query binding missing")
        assert(vehicle:get_crosstrack_enabled() == true, "crosstrack start did not enable")
        assert(vehicle:set_crosstrack_enabled(false), "crosstrack disable failed")
        assert(vehicle:get_crosstrack_enabled() == false, "crosstrack stayed enabled")
        assert(vehicle:set_crosstrack_enabled(true), "crosstrack restore failed")
        assert(vehicle:get_crosstrack_enabled() == true, "crosstrack stayed disabled")
    else
        assert(vehicle.set_crosstrack_enabled == nil, "DAA setter leaked into Lua-only build")
        assert(vehicle.get_crosstrack_enabled == nil, "DAA getter leaked into Lua-only build")
    end
    gcs:send_text(6, SCRIPT_NAME_SHORT .. " " .. SCRIPT_VERSION .. " passed: " ..
                 (enabled and "DAA enabled" or "DAA disabled"))
end

assert(SCRIPT_NAME ~= "")
return update()
""")
        self.set_parameters({"SCR_ENABLE": 1, "SCR_VM_I_COUNT": 100000})
        self.context_collect("STATUSTEXT")
        self.reboot_sitl()
        self.wait_text("DAABnd 1.0.0 passed: DAA", check_context=True, timeout=60)
