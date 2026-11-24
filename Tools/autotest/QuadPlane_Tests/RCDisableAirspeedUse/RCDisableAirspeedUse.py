'''
QuadPlane autotest: RCDisableAirspeedUse

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class RCDisableAirspeedUse(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def RCDisableAirspeedUse(self):
        '''check disabling airspeed using RC switch'''
        self.set_parameter("RC9_OPTION", 106)
        self.set_rc(9, 1000)
        self.wait_sensor_state(
            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
            True,
            True,
            True)
        self.set_rc(9, 2000)
        self.wait_sensor_state(
            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
            True,
            False,
            True)
        self.set_rc(9, 1000)
        self.wait_sensor_state(
            mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
            True,
            True,
            True)

        self.progress("Disabling airspeed sensor")
        self.context_push()
        self.set_rc(9, 2000)
        self.set_parameters({
            "COMPASS_ENABLE": 0,
            "EK2_ENABLE": 0,
            "AHRS_EKF_TYPE": 3,
            "COMPASS_USE": 0,
            "COMPASS_USE2": 0,
            "COMPASS_USE3": 0,
            "ARMING_SKIPCHK": 1 << 2,  # disables compass
        })

        self.reboot_sitl()

        self.context_collect('STATUSTEXT')
        self.wait_prearm_sys_status_healthy(timeout=120)
        self.change_mode('QLOITER')
        self.arm_vehicle()
        self.set_rc(3, 2000)
        self.wait_altitude(10, 30, relative=True)
        self.change_mode('FBWA')
        self.wait_statustext('Transition done')
        # the vehicle stays in DCM until there's velocity - make sure
        # we did go to EK3 evenutally, 'though:
        self.wait_statustext('EKF3 active', check_context=True)

        self.disarm_vehicle(force=True)
        self.context_pop()
        self.reboot_sitl()
