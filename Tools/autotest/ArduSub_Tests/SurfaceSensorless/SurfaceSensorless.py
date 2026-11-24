'''
Sub autotest: SurfaceSensorless

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase

from ardusub import Joystick


class SurfaceSensorless(TestGroup):
    @testcase(bucket="Sub")
    def SurfaceSensorless(self):
        """Test surface mode with sensorless thrust"""
        # this drives the throttle down and waits to arrive at 9.5m, so
        # it has to start above that.  The vehicle is wherever the
        # previous test left it, which can be a long way below:
        #     Failed to attain Altitude want -9.5, reached -52.093
        # with the depth not moving at all - it was already far past the
        # altitude it was descending towards.  Reboot back to the
        # surface first, as several tests in this file already do.
        self.reboot_sitl()
        # set GCS failsafe to SURFACE
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.change_mode("STABILIZE")
        self.set_parameter("MAV_GCS_SYSID", self.mav.source_system)

        self.set_rc(Joystick.Throttle, 1100)
        self.wait_altitude(altitude_min=-10, altitude_max=-9, relative=False, timeout=60)
        self.set_rc(Joystick.Throttle, 1500)

        self.setGCSfailsafe(4)
        self.set_parameter("SIM_BARO_DISABLE", 1)
        self.set_heartbeat_rate(0)
        self.wait_mode("SURFACE")
        self.progress("Surface mode engaged")
        self.wait_altitude(altitude_min=-1, altitude_max=0, relative=False, timeout=60)
        self.progress("Vehicle resurfaced")
        self.set_heartbeat_rate(self.speedup)
        self.wait_statustext("GCS Failsafe Cleared", timeout=60)
        self.progress("Baro-less Surface mode OK")
        self.disarm_vehicle()
