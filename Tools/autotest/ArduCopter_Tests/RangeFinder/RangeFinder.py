'''
Copter autotest: RangeFinder

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class RangeFinder(TestGroup):
    @testcase(bucket="CopterTests2d")
    def RangeFinder(self):
        '''Test RangeFinder Basic Functionality'''
        self.assert_not_receiving_message('RANGEFINDER', timeout=5)
        self.assert_not_receiving_message('DISTANCE_SENSOR', timeout=5)

        # may need to force a rotation if some other test has used the
        # rangefinder...
        self.progress("Ensure no RFND messages in log")
        self.set_parameter("LOG_DISARMED", 1)
        if self.current_onboard_log_contains_message("RFND"):
            raise NotAchievedException("Found unexpected RFND message")

        self.set_analog_rangefinder_parameters()
        self.set_parameter("RC9_OPTION", 10) # rangefinder
        self.set_rc(9, 2000)

        self.reboot_sitl()

        self.progress("Making sure we now get RANGEFINDER messages")
        self.context_set_message_rate_hz('RANGEFINDER', self.sitl_streamrate())
        m = self.assert_receive_message('RANGEFINDER', timeout=10)

        self.progress("Making sure we now get DISTANCE_SENSOR messages")
        self.assert_receive_message('DISTANCE_SENSOR', timeout=10)

        self.progress("Checking RangeFinder is marked as enabled in mavlink")
        m = self.assert_receive_message('SYS_STATUS', timeout=10)
        flags = m.onboard_control_sensors_enabled
        if not flags & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION:
            raise NotAchievedException("Laser not enabled in SYS_STATUS")
        self.progress("Disabling laser using switch")
        self.set_rc(9, 1000)
        self.progress("Checking RangeFinder is marked as disabled in mavlink")
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION, True, False, False)

        self.progress("Re-enabling rangefinder")
        self.set_rc(9, 2000)
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_LASER_POSITION, True, True, True)

        self.takeoff(10, mode="LOITER")

        m_p = self.assert_receive_message('GLOBAL_POSITION_INT')

        m_r = self.assert_receive_message('RANGEFINDER')
        if abs(m_r.distance - m_p.relative_alt/1000) > 1:
            raise NotAchievedException(
                "RANGEFINDER/GLOBAL_POSITION_INT mismatch %0.2f vs %0.2f" %
                (m_r.distance, m_p.relative_alt/1000))

        m_ds = self.assert_receive_message('DISTANCE_SENSOR')
        if abs(m_ds.current_distance * 0.01 - m_p.relative_alt/1000) > 1:
            raise NotAchievedException(
                "DISTANCE_SENSOR/GLOBAL_POSITION_INT mismatch %0.2f vs %0.2f" %
                (m_ds.current_distance*0.01, m_p.relative_alt/1000))

        self.land_and_disarm()

        if not self.current_onboard_log_contains_message("RFND"):
            raise NotAchievedException("Did not see expected RFND message")
