'''
Copter autotest: GPSViconSwitching

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GPSViconSwitching(TestGroup):
    @testcase(bucket="CopterTests2b")
    def GPSViconSwitching(self):
        """Fly GPS and Vicon switching test"""
        """Setup parameters including switching to EKF3"""
        self.set_parameters({
            "VISO_TYPE": 2,      # enable vicon
            "SERIAL5_PROTOCOL": 2,
            "EK3_ENABLE": 1,
            "EK3_SRC2_POSXY": 6, # External Nav
            "EK3_SRC2_POSZ": 6,  # External Nav
            "EK3_SRC2_VELXY": 6, # External Nav
            "EK3_SRC2_VELZ": 6,  # External Nav
            "EK3_SRC2_YAW": 6,   # External Nav
            "RC7_OPTION": 80,    # RC aux switch 7 set to Viso Align
            "RC8_OPTION": 90,    # RC aux switch 8 set to EKF source selector
            "EK2_ENABLE": 0,
            "AHRS_EKF_TYPE": 3,
        })
        self.customise_SITL_commandline(["--serial5=sim:vicon:"])

        # switch to use GPS
        self.set_rc(8, 1000)

        # ensure we can get a global position:
        self.poll_home_position(timeout=120)

        # record starting position
        old_pos = self.get_global_position_int()
        print("old_pos=%s" % str(old_pos))

        # align vicon yaw with ahrs heading
        self.set_rc(7, 2000)

        # takeoff to 10m in Loiter
        self.progress("Moving to ensure location is tracked")
        self.takeoff(10, mode="LOITER", require_absolute=True, timeout=720)

        # fly forward in Loiter
        self.set_rc(2, 1300)

        # disable vicon
        self.set_parameter("SIM_VICON_FAIL", 1)

        # ensure vehicle remain in Loiter for 15 seconds
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 15:
            if not self.mode_is('LOITER', poll=False):
                raise NotAchievedException("Expected to stay in loiter for >15 seconds")

        # re-enable vicon
        self.set_parameter("SIM_VICON_FAIL", 0)

        # switch to vicon, disable GPS and wait 10sec to ensure vehicle remains in Loiter
        self.set_rc(8, 1500)
        self.set_parameter("GPS1_TYPE", 0)

        # ensure vehicle remain in Loiter for 15 seconds
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 15:
            if not self.mode_is('LOITER', poll=False):
                raise NotAchievedException("Expected to stay in loiter for >15 seconds")

        # RTL and check vehicle arrives within 10m of home
        self.set_rc(2, 1500)
        self.do_RTL()
