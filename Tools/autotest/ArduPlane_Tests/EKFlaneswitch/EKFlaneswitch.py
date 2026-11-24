'''
Plane autotest: EKFlaneswitch

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class EKFlaneswitch(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def EKFlaneswitch(self):
        '''Test EKF3 Affinity and Lane Switching'''

        # new lane switch available only with EK3
        self.set_parameters({
            "EK3_ENABLE": 1,
            "EK2_ENABLE": 0,
            "AHRS_EKF_TYPE": 3,
            "EK3_AFFINITY": 15, # enable affinity for all sensors
            "EK3_IMU_MASK": 3, # use only 2 IMUs
            "GPS2_TYPE": 1,
            "SIM_GPS2_ENABLE": 1,
            "SIM_BARO_COUNT": 2,
            "SIM_BAR2_DISABLE": 0,
            "ARSPD2_TYPE": 2,
            "ARSPD2_USE": 1,
            "ARSPD2_PIN": 2,
        })

        # some parameters need reboot to take effect
        self.reboot_sitl()

        self.lane_switches = []

        # add an EKF lane switch hook
        def statustext_hook(mav, message):
            if message.get_type() != 'STATUSTEXT':
                return
            # example msg: EKF3 lane switch 1
            if not message.text.startswith("EKF3 lane switch "):
                return
            newlane = int(message.text[-1])
            self.lane_switches.append(newlane)
        self.install_message_hook_context(statustext_hook)

        # get flying
        self.takeoff(alt=50)
        self.change_mode('CIRCLE')

        ###################################################################
        self.progress("Checking EKF3 Lane Switching trigger from all sensors")
        ###################################################################
        self.start_subtest("ACCELEROMETER: Change z-axis offset")
        # create an accelerometer error by changing the Z-axis offset
        self.context_collect("STATUSTEXT")
        old_parameter = self.get_parameter("INS_ACCOFFS_Z")
        self.wait_statustext(
            text="EKF3 lane switch",
            timeout=30,
            the_function=self.set_parameter("INS_ACCOFFS_Z", old_parameter + 5),
            check_context=True)
        if self.lane_switches != [1]:
            raise NotAchievedException("Expected lane switch 1, got %s" % str(self.lane_switches[-1]))
        # Cleanup
        self.set_parameter("INS_ACCOFFS_Z", old_parameter)
        self.context_clear_collection("STATUSTEXT")
        self.wait_heading(0, accuracy=10, timeout=60)
        self.wait_heading(180, accuracy=10, timeout=60)
        ###################################################################
        self.start_subtest("BAROMETER: Freeze to last measured value")
        self.context_collect("STATUSTEXT")
        # create a barometer error by inhibiting any pressure change while changing altitude
        old_parameter = self.get_parameter("SIM_BAR2_FREEZE")
        self.set_parameter("SIM_BAR2_FREEZE", 1)
        self.wait_statustext(
            text="EKF3 lane switch",
            timeout=30,
            the_function=lambda: self.set_rc(2, 2000),
            check_context=True)
        if self.lane_switches != [1, 0]:
            raise NotAchievedException("Expected lane switch 0, got %s" % str(self.lane_switches[-1]))
        # Cleanup
        self.set_rc(2, 1500)
        self.set_parameter("SIM_BAR2_FREEZE", old_parameter)
        self.context_clear_collection("STATUSTEXT")
        self.wait_heading(0, accuracy=10, timeout=60)
        self.wait_heading(180, accuracy=10, timeout=60)
        ###################################################################
        self.start_subtest("GPS: Apply GPS Velocity Error in NED")
        self.context_push()
        self.context_collect("STATUSTEXT")

        # create a GPS velocity error by adding a random 2m/s
        # noise on each axis
        def sim_gps_verr():
            self.set_parameters({
                "SIM_GPS1_VERR_X": self.get_parameter("SIM_GPS1_VERR_X") + 2,
                "SIM_GPS1_VERR_Y": self.get_parameter("SIM_GPS1_VERR_Y") + 2,
                "SIM_GPS1_VERR_Z": self.get_parameter("SIM_GPS1_VERR_Z") + 2,
            })
        self.wait_statustext(text="EKF3 lane switch", timeout=30, the_function=sim_gps_verr, check_context=True)
        if self.lane_switches != [1, 0, 1]:
            raise NotAchievedException("Expected lane switch 1, got %s" % str(self.lane_switches[-1]))
        # Cleanup
        self.context_pop()
        self.context_clear_collection("STATUSTEXT")
        self.wait_heading(0, accuracy=10, timeout=60)
        self.wait_heading(180, accuracy=10, timeout=60)
        ###################################################################
        self.start_subtest("MAGNETOMETER: Change X-Axis Offset")
        self.context_collect("STATUSTEXT")
        # create a magnetometer error by changing the X-axis offset
        old_parameter = self.get_parameter("SIM_MAG2_OFS_X")
        self.wait_statustext(
            text="EKF3 lane switch",
            timeout=30,
            the_function=self.set_parameter("SIM_MAG2_OFS_X", old_parameter + 150),
            check_context=True)
        if self.lane_switches != [1, 0, 1, 0]:
            raise NotAchievedException("Expected lane switch 0, got %s" % str(self.lane_switches[-1]))
        # Cleanup
        self.set_parameter("SIM_MAG2_OFS_X", old_parameter)
        self.context_clear_collection("STATUSTEXT")
        self.wait_heading(0, accuracy=10, timeout=60)
        self.wait_heading(180, accuracy=10, timeout=60)
        ###################################################################
        self.start_subtest("AIRSPEED: Fail to constant value")
        self.context_push()
        self.context_collect("STATUSTEXT")

        old_parameter = self.get_parameter("SIM_ARSPD_FAIL")

        def fail_speed():
            self.change_mode("GUIDED")
            loc = self.get_location()
            self.run_cmd_int(
                mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                p5=int(loc.lat * 1e7),
                p6=int(loc.lng * 1e7),
                p7=50,    # alt
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            )
            self.delay_sim_time(5, reason="position to reach")
            # create an airspeed sensor error by freezing to the
            # current airspeed then changing the airspeed demand
            # to a higher value and waiting for the TECS speed
            # loop to diverge
            m = self.assert_receive_message('VFR_HUD')
            self.set_parameter("SIM_ARSPD_FAIL", m.airspeed)
            self.run_cmd(
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                p1=0, # airspeed
                p2=30,
                p3=-1, # throttle / no change
                p4=0, # absolute values
            )
        self.wait_statustext(text="EKF3 lane switch", timeout=30, the_function=fail_speed, check_context=True)
        if self.lane_switches != [1, 0, 1, 0, 1]:
            raise NotAchievedException("Expected lane switch 1, got %s" % str(self.lane_switches[-1]))
        # Cleanup
        self.set_parameter("SIM_ARSPD_FAIL", old_parameter)
        self.change_mode('CIRCLE')
        self.context_pop()
        self.context_clear_collection("STATUSTEXT")
        self.wait_heading(0, accuracy=10, timeout=60)
        self.wait_heading(180, accuracy=10, timeout=60)
        ###################################################################
        self.progress("GYROSCOPE: Change Y-Axis Offset")
        self.context_collect("STATUSTEXT")
        # create a gyroscope error by changing the Y-axis offset
        old_parameter = self.get_parameter("INS_GYR2OFFS_Y")
        self.wait_statustext(
            text="EKF3 lane switch",
            timeout=30,
            the_function=self.set_parameter("INS_GYR2OFFS_Y", old_parameter + 1),
            check_context=True)
        if self.lane_switches != [1, 0, 1, 0, 1, 0]:
            raise NotAchievedException("Expected lane switch 0, got %s" % str(self.lane_switches[-1]))
        # Cleanup
        self.set_parameter("INS_GYR2OFFS_Y", old_parameter)
        self.context_clear_collection("STATUSTEXT")
        ###################################################################

        self.disarm_vehicle(force=True)
