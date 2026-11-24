'''
Plane autotest: FenceAutoEnableDisableSwitch

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class FenceAutoEnableDisableSwitch(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def FenceAutoEnableDisableSwitch(self):
        '''Tests autoenablement of regular fences and manual disablement'''
        self.set_parameters({
            "FENCE_TYPE": 9,     # Set fence type to min alt, max alt
            "FENCE_ACTION": 1,   # Set action to RTL
            "FENCE_ALT_MIN": 50,
            "FENCE_ALT_MAX": 100,
            "FENCE_AUTOENABLE": 2,
            "FENCE_OPTIONS" : 1,
            "FENCE_ENABLE" : 1,
            "FENCE_RADIUS" : 300,
            "FENCE_RET_ALT" : 0,
            "FENCE_RET_RALLY" : 0,
            "FENCE_TOTAL" : 0,
            "RTL_ALTITUDE" : 75,
            "TKOFF_ALT" : 75,
            "RC7_OPTION" : 11,   # AC_Fence uses Aux switch functionality
        })
        self.reboot_sitl()
        self.context_collect("STATUSTEXT")

        fence_bit = mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE
        # Grab Home Position
        self.assert_receive_message('HOME_POSITION')
        self.set_rc(7, 1000) # Turn fence off with aux function, does not impact later auto-enable

        self.wait_ready_to_arm()

        self.progress("Check fence disabled at boot")
        m = self.assert_receive_message('SYS_STATUS')
        if (m.onboard_control_sensors_enabled & fence_bit):
            raise NotAchievedException("Fence is enabled at boot")

        cruise_alt = 75
        self.takeoff(cruise_alt, mode='TAKEOFF')

        self.progress("Fly above ceiling and check there is a breach")
        self.change_mode('FBWA')
        self.set_rc_from_map({3: 2000, 2: 1000})

        self.wait_statustext("Max Alt fence breached", timeout=10, check_context=True)
        self.wait_mode('RTL')

        m = self.assert_receive_message('SYS_STATUS')
        if (m.onboard_control_sensors_health & fence_bit):
            raise NotAchievedException("Fence ceiling not breached")

        self.set_rc_from_map({3: 1500, 2: 1500})

        self.progress("Wait for RTL alt reached")
        self.wait_altitude(cruise_alt-5, cruise_alt+5, relative=True, timeout=30)

        self.progress("Return to cruise alt")
        self.set_rc(3, 1500)
        self.change_altitude(cruise_alt, relative=True)

        self.progress("Check fence breach cleared")
        m = self.assert_receive_message('SYS_STATUS')
        if (not (m.onboard_control_sensors_health & fence_bit)):
            raise NotAchievedException("Fence breach not cleared")

        self.progress("Fly above ceiling and check there is a breach and switch to AUTOLAND mode")
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "FENCE_ACTION": 8,   # Set action to AUTOLAND if possible
        })

        self.set_rc_from_map({3: 2000, 2: 1000})

        self.wait_statustext("Max Alt fence breached", timeout=10, check_context=True)
        self.wait_mode("AUTOLAND")
        self.assert_fence_sys_status(True, True, False)

        self.set_rc_from_map({3: 1500, 2: 1500})

        self.progress("Wait for cruise alt reached")
        self.wait_altitude(
            cruise_alt-5,
            cruise_alt+5,
            relative=True,
            timeout=30,
        )

        self.progress("Check fence breach cleared")
        self.assert_fence_sys_status(True, True, True)

        self.context_pop()
        self.change_mode('FBWA')
        self.progress("Fly below floor and check for breach")
        self.set_rc(2, 2000)
        self.wait_statustext("Min Alt fence breached", timeout=10, check_context=True)
        self.wait_mode("RTL")
        m = self.assert_receive_message('SYS_STATUS')
        if (m.onboard_control_sensors_health & fence_bit):
            raise NotAchievedException("Fence floor not breached")

        self.change_mode("FBWA")

        self.progress("Fly above floor and check fence is enabled")
        self.set_rc(3, 2000)
        self.change_altitude(75, relative=True)
        m = self.assert_receive_message('SYS_STATUS')
        if (not (m.onboard_control_sensors_enabled & fence_bit)):
            raise NotAchievedException("Fence Floor not enabled")

        self.progress("Toggle fence enable/disable")
        self.set_rc(7, 2000)
        self.delay_sim_time(2, reason="fence switch to register")
        self.set_rc(7, 1000)

        self.progress("Check fence is disabled")
        self.wait_sensor_state(fence_bit, present=True, enabled=False, healthy=True)

        self.progress("Fly below floor and check for no breach")
        self.change_altitude(40, relative=True)
        m = self.assert_receive_message('SYS_STATUS')
        if (not (m.onboard_control_sensors_health & fence_bit)):
            raise NotAchievedException("Fence floor breached")

        self.progress("Return to cruise alt")
        self.set_rc(3, 1500)
        self.change_altitude(cruise_alt, relative=True)
        self.fly_home_land_and_disarm(timeout=250)
