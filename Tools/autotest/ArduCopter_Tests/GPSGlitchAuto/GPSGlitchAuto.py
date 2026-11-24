'''
Copter autotest: GPSGlitchAuto

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GPSGlitchAuto(TestGroup):
    @testcase(bucket="CopterTests2c")
    def GPSGlitchAuto(self, timeout=180):
        '''fly mission and test reaction to gps glitch'''
        # set-up gps glitch array
        glitch_lat = [0.0002996,
                      0.0006958,
                      0.0009431,
                      0.0009991,
                      0.0009444,
                      0.0007716,
                      0.0006221]
        glitch_lon = [0.0000717,
                      0.0000912,
                      0.0002761,
                      0.0002626,
                      0.0002807,
                      0.0002049,
                      0.0001304]
        glitch_num = len(glitch_lat)
        self.progress("GPS Glitches:")
        for i in range(1, glitch_num):
            self.progress("glitch %d %.7f %.7f" %
                          (i, glitch_lat[i], glitch_lon[i]))

        # Fly mission #1
        self.progress("# Load copter_glitch_mission")
        # load the waypoint count
        num_wp = self.load_mission("copter_glitch_mission.txt", strict=False)
        if not num_wp:
            raise NotAchievedException("load copter_glitch_mission failed")

        # turn on simulator display of gps and actual position
        if self.use_map:
            self.show_gps_and_sim_positions(True)

        self.progress("test: Fly a mission from 1 to %u" % num_wp)
        self.set_current_waypoint(1)

        self.change_mode("STABILIZE")
        self.wait_ready_to_arm()
        self.zero_throttle()
        self.arm_vehicle()

        # switch into AUTO mode and raise throttle
        self.change_mode('AUTO')
        self.set_rc(3, 1500)

        # wait until 100m from home
        try:
            self.wait_distance(100, 5, 90)
        except Exception as e:
            if self.use_map:
                self.show_gps_and_sim_positions(False)
            raise e

        # stop and test loss of GPS for a short time - it should resume GPS use without falling back into a non aiding mode
        self.change_mode("LOITER")
        self.set_parameters({
            "SIM_GPS1_ENABLE": 0,
        })
        self.delay_sim_time(2, reason="GPS disable to take effect")
        self.set_parameters({
            "SIM_GPS1_ENABLE": 1,
        })
        # regaining GPS should not result in it falling back to a non-navigation mode
        self.wait_ekf_flags(mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS, 0, timeout=1)
        # It should still be navigating after enougnh time has passed for any pending timeouts to activate.
        self.delay_sim_time(10, reason="pending EKF timeouts to activate")
        self.wait_ekf_flags(mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS, 0, timeout=1)
        self.change_mode("AUTO")

        # record time and position
        tstart = self.get_sim_time()

        # initialise current glitch
        glitch_current = 0
        self.progress("Apply first glitch")
        self.set_parameters({
            "SIM_GPS1_GLTCH_X": glitch_lat[glitch_current],
            "SIM_GPS1_GLTCH_Y": glitch_lon[glitch_current],
        })

        # record position for 30 seconds
        while glitch_current < glitch_num:
            tnow = self.get_sim_time()
            desired_glitch_num = int((tnow - tstart) * 2.2)
            if desired_glitch_num > glitch_current and glitch_current != -1:
                glitch_current = desired_glitch_num
                # apply next glitch
                if glitch_current < glitch_num:
                    self.progress("Applying glitch %u" % glitch_current)
                    self.set_parameters({
                        "SIM_GPS1_GLTCH_X": glitch_lat[glitch_current],
                        "SIM_GPS1_GLTCH_Y": glitch_lon[glitch_current],
                    })

        # turn off glitching
        self.progress("Completed Glitches")
        self.set_parameters({
            "SIM_GPS1_GLTCH_X": 0,
            "SIM_GPS1_GLTCH_Y": 0,
        })

        # continue with the mission
        self.wait_waypoint(0, num_wp-1, timeout=500)

        # wait for arrival back home
        self.wait_distance_to_home(0, 10, timeout=timeout)

        # turn off simulator display of gps and actual position
        if self.use_map:
            self.show_gps_and_sim_positions(False)

        self.progress("GPS Glitch test Auto completed: passed!")
        self.wait_disarmed()
        # re-arming is problematic because the GPS is glitching!
        self.reboot_sitl()
