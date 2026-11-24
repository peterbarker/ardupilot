'''
Plane autotest: mavlink_AIRSPEED

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class mavlink_AIRSPEED(TestGroup):
    @testcase(bucket="PlaneTests1a")
    def mavlink_AIRSPEED(self):
        '''check receiving of two airspeed sensors'''
        self.set_parameters({
            "ARSPD_PIN": 1,
            "ARSPD_RATIO": 2,
            "ARSPD2_RATIO": 2,
            "ARSPD2_TYPE": 100,
            "ARSPD2_AUTOCAL": 1,
        })
        self.reboot_sitl()

        # Add listener to airspeed message
        airspeed_active = [None, None]

        def update_airspeed_active(mav, m):
            if m.get_type() != 'AIRSPEED':
                return
            # 2 here is the correct value of mavutil.mavlink.AIRSPEED_SENSOR_USING
            # However it was wrong in the MAVLink spec for a while
            airspeed_active[m.id] = (m.flags & 2) != 0

        self.install_message_hook_context(update_airspeed_active)

        self.wait_ready_to_arm()

        self.start_subtest('Ensure we get both instances')
        if any(v is None for v in airspeed_active):
            raise NotAchievedException("Did not get both airspeed messages %s" % str(airspeed_active))

        if airspeed_active != [True, False]:
            raise NotAchievedException("Not using expected airspeed sensors %s" % str(airspeed_active))

        self.takeoff(mode="TAKEOFF", alt=100)

        self.start_subtest("Now testing sensor 1 is used in flight")
        if airspeed_active != [True, False]:
            raise NotAchievedException("Not using expected airspeed sensors %s" % str(airspeed_active))

        self.start_subtest("Now testing failure of sensor 1 - fail to many m/s")
        self.set_parameter("SIM_ARSPD_FAIL", 60)

        # airspeed sensor never becomes unhealthy - we just stop using
        # it as EKF3 starts to reject, allow 10 seconds
        self.delay_sim_time(10, reason="EKF to reject failed airspeed sensor")
        if airspeed_active != [False, False]:
            raise NotAchievedException("Not using expected airspeed sensors %s" % str(airspeed_active))

        # Re-enable sensors, after some time the sensor should be re-enabled
        self.set_parameter("SIM_ARSPD_FAIL", 0)
        self.delay_sim_time(60, reason="airspeed sensor to be re-enabled")
        if airspeed_active != [True, False]:
            raise NotAchievedException("Not using expected airspeed sensors %s" % str(airspeed_active))

        self.start_subtest("Now testing combinations of use and primary params")

        # Changing the primary should have no effect as the second sensor is not marked to use
        self.set_parameter("ARSPD_PRIMARY", 1)
        self.delay_sim_time(10, reason="primary airspeed change to take effect")
        if airspeed_active != [True, False]:
            raise NotAchievedException("Not using expected airspeed sensors %s" % str(airspeed_active))

        # Allowing the second sensor to be used should not result in a switch when armed despite the primary parameter
        self.set_parameter("ARSPD2_USE", 1)
        self.delay_sim_time(10, reason="airspeed sensor use to take effect")
        if airspeed_active != [True, False]:
            raise NotAchievedException("Not using expected airspeed sensors %s" % str(airspeed_active))

        # Changing the primary back to the first sensor, and then again to the second
        # Now the second can be used it should switch
        self.set_parameter("ARSPD_PRIMARY", 0)
        self.set_parameter("ARSPD_PRIMARY", 1)
        self.delay_sim_time(10, reason="airspeed primary switch to take effect")
        if airspeed_active != [False, True]:
            raise NotAchievedException("Not using expected airspeed sensors %s" % str(airspeed_active))

        # Now stop using the second
        self.set_parameter("ARSPD2_USE", 0)
        self.delay_sim_time(10, reason="airspeed sensor disable to take effect")
        if airspeed_active != [True, False]:
            raise NotAchievedException("Not using expected airspeed sensors %s" % str(airspeed_active))

        self.fly_home_land_and_disarm()
        self.change_mode("MANUAL")
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_ALL)

        self.start_subtest("Now testing primary arming check")

        # Should not be able to arm with primary sensor not set to use
        self.assert_prearm_failure("Airspeed: not using Primary (2)")

        self.start_subtest("Testing primary changes when disarmed")

        # Now the vehicle is disarmed, enabling use for the primary sensor should result in it being used
        self.set_parameter("ARSPD2_USE", 1)
        self.delay_sim_time(10, reason="airspeed sensor enable to take effect")
        if airspeed_active != [False, True]:
            raise NotAchievedException("Not using expected airspeed sensors %s" % str(airspeed_active))
