'''
Plane autotest: ExternalPositionEstimate

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import OldpymavlinkException

from test_discovery import TestGroup
from test_discovery import testcase


class ExternalPositionEstimate(TestGroup):
    @testcase(bucket="PlaneTests1c")
    def ExternalPositionEstimate(self):
        '''Test mavlink EXTERNAL_POSITION_ESTIMATE command'''
        if not hasattr(mavutil.mavlink, 'MAV_CMD_EXTERNAL_POSITION_ESTIMATE'):
            raise OldpymavlinkException("pymavlink too old; upgrade pymavlink to get MAV_CMD_EXTERNAL_POSITION_ESTIMATE")  # noqa
        self.change_mode("TAKEOFF")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_altitude(48, 52, relative=True)

        loc = self.get_location()
        self.location_offset_ne(loc, 2000, 2000)

        # setting external position fail while we have GPS lock
        self.progress("set new position with GPS")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_EXTERNAL_POSITION_ESTIMATE,
            p1=self.get_sim_time()-1, # transmit time
            p2=0.5, # processing delay
            p3=50, # accuracy
            p5=int(loc.lat * 1e7),
            p6=int(loc.lng * 1e7),
            p7=float("NaN"),    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
            want_result=mavutil.mavlink.MAV_RESULT_FAILED,
        )

        self.progress("disable the GPS")
        self.run_auxfunc(
            65,
            2,
            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED
        )

        self.delay_sim_time(20, "fly for a bit to get into non-aiding state")

        self.progress("getting base position")
        gpi = self.assert_receive_message('GLOBAL_POSITION_INT', timeout=5)
        loc = Location.latlon_only(gpi.lat*1e-7, gpi.lon*1e-7)

        self.progress("set new position with no GPS")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_EXTERNAL_POSITION_ESTIMATE,
            p1=self.get_sim_time()-1, # transmit time
            p2=0.5, # processing delay
            p3=50, # accuracy
            p5=gpi.lat+1,
            p6=gpi.lon+1,
            p7=float("NaN"),    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL,
            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED
        )

        self.delay_sim_time(3, "let new position get processed")

        gpi2 = self.assert_receive_message('GLOBAL_POSITION_INT', timeout=5)
        loc2 = Location.latlon_only(gpi2.lat*1e-7, gpi2.lon*1e-7)
        dist = self.get_distance(loc, loc2)

        self.progress("dist is %.1f" % dist)
        if dist > 200:
            raise NotAchievedException("Position error dist=%.1f" % dist)

        self.progress("re-enable the GPS")
        self.run_auxfunc(
            65,
            0,
            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED
        )

        self.progress("flying home")
        self.fly_home_land_and_disarm()
