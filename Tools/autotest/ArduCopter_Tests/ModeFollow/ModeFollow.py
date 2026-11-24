'''
Copter autotest: ModeFollow

AP_FLAKE8_CLEAN
'''

import copy
import math

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ModeFollow(TestGroup):
    def location_from_message_cache(self) -> Location:
        '''return a lat/lng-only Location from the most recently received
        GPS_RAW_INT; only the horizontal position is of interest here'''
        return Location.latlon_only(
            self.mav.messages['GPS_RAW_INT'].lat*1.0e-7,
            self.mav.messages['GPS_RAW_INT'].lon*1.0e-7,
        )

    @testcase(bucket="CopterTests1e")
    def ModeFollow(self):
        '''Fly follow mode'''
        foll_ofs_x = 30 # metres
        self.set_parameters({
            "FOLL_ENABLE": 1,
            "FOLL_SYSID": self.mav.source_system,
            "FOLL_OFS_X": -foll_ofs_x,
            "FOLL_OFS_TYPE": 1, # relative to other vehicle heading
        })
        self.takeoff(10, mode="LOITER")
        self.context_push()
        self.context_set_speedup(1)
        self.change_mode("FOLLOW")
        new_loc = self.get_location()
        new_loc_offset_n = 20
        new_loc_offset_e = 30
        self.location_offset_ne(new_loc, new_loc_offset_n, new_loc_offset_e)
        self.progress("new_loc: %s" % str(new_loc))
        heading = 0
        if self.mavproxy is not None:
            self.mavproxy.send("map icon %f %f greenplane %f\n" %
                               (new_loc.lat, new_loc.lng, heading))

        expected_loc = copy.copy(new_loc)
        self.location_offset_ne(expected_loc, -foll_ofs_x, 0)
        if self.mavproxy is not None:
            self.mavproxy.send("map icon %f %f hoop\n" %
                               (expected_loc.lat, expected_loc.lng))
        self.progress("expected_loc: %s" % str(expected_loc))

        origin = self.poll_message('GPS_GLOBAL_ORIGIN')

        last_sent = 0
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 60:
                raise NotAchievedException("Did not FOLLOW")
            if now - last_sent > 0.5:
                gpi = self.mav.mav.global_position_int_encode(
                    int(now * 1000), # time_boot_ms
                    int(new_loc.lat * 1e7),
                    int(new_loc.lng * 1e7),
                    int(new_loc.get_alt_m(AltFrame.ABSOLUTE) * 1000), # alt in mm
                    int(new_loc.get_alt_m(AltFrame.ABSOLUTE) * 1000 - origin.altitude), # relative alt - urp.
                    vx=0,
                    vy=0,
                    vz=0,
                    hdg=heading
                )
                gpi.pack(self.mav.mav)
                self.mav.mav.send(gpi)
            self.assert_receive_message('GLOBAL_POSITION_INT')
            pos = self.location_from_message_cache()
            delta = self.get_distance(expected_loc, pos)
            max_delta = 3
            self.progress("position delta=%f (want <%f)" % (delta, max_delta))
            if delta < max_delta:
                break

        self.start_subtest("Trying relative-follow mode")
        self.change_mode('LOITER')
        self.set_parameter('FOLL_ALT_TYPE', 1)  # use relative-home data
        new_loc = self.get_location()
        new_loc_offset_n = -40
        new_loc_offset_e = 60
        new_loc.offset_up_m(1)
        self.location_offset_ne(new_loc, new_loc_offset_n, new_loc_offset_e)
        expected_loc = copy.copy(new_loc)
        self.location_offset_ne(expected_loc, -foll_ofs_x, 0)
        if self.mavproxy is not None:
            self.mavproxy.send("map icon %f %f hoop\n" %
                               (expected_loc.lat, expected_loc.lng))
        self.progress("expected_loc: %s" % str(expected_loc))
        self.progress("new_loc: %s" % str(new_loc))
        self.change_mode('FOLLOW')
        last_sent = 0
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 60:
                raise NotAchievedException("Did not FOLLOW")
            if now - last_sent > 0.5:
                gpi = self.mav.mav.global_position_int_encode(
                    int(now * 1000), # time_boot_ms
                    int(new_loc.lat * 1e7),
                    int(new_loc.lng * 1e7),
                    666, # alt in mm - note incorrect data!
                    int(new_loc.get_alt_m(AltFrame.ABSOLUTE) * 1000 - origin.altitude), # relative alt - urp.
                    vx=0,
                    vy=0,
                    vz=0,
                    hdg=heading
                )
                gpi.pack(self.mav.mav)
                self.mav.mav.send(gpi)
            self.assert_receive_message('GLOBAL_POSITION_INT')
            pos = self.location_from_message_cache()
            delta = self.get_distance(expected_loc, pos)
            max_delta = 3
            self.progress("position delta=%f (want <%f)" % (delta, max_delta))
            if delta < max_delta:
                break

        self.start_subtest("Trying follow-with-velocity mode")
        self.change_mode('LOITER')
        self.set_parameter('FOLL_ALT_TYPE', 1)  # use relative-home data
        new_loc = self.get_location()
        vel_n = 3  # m/s
        vel_e = 2
        vel_d = -1
        tstart = self.get_sim_time()
        first_loop = True
        last_sent = 0
        last_loop_s = tstart
        heading = math.atan2(vel_n, vel_e)
        while True:
            now = self.get_sim_time_cached()
            dt = now - last_loop_s
            last_loop_s = now
            new_loc_offset_n = vel_n * dt
            new_loc_offset_e = vel_e * dt
            new_loc_offset_u = vel_d * dt * -1

            self.location_offset_ne(new_loc, new_loc_offset_n, new_loc_offset_e)
            new_loc.offset_up_m(new_loc_offset_u)
            expected_loc = copy.copy(new_loc)
            self.location_offset_ne(expected_loc, -foll_ofs_x, 0)
            self.progress("expected_loc: %s" % str(expected_loc))
            self.progress("new_loc: %s" % str(new_loc))
            if first_loop:
                self.change_mode('FOLLOW')
                first_loop = False

            if now - tstart > 60:
                raise NotAchievedException("Did not FOLLOW")
            if now - last_sent > 0.5:
                gpi = self.mav.mav.global_position_int_encode(
                    int(now * 1000), # time_boot_ms
                    int(new_loc.lat * 1e7),
                    int(new_loc.lng * 1e7),
                    666, # alt in mm - note incorrect data!
                    int(new_loc.get_alt_m(AltFrame.ABSOLUTE) * 1000 - origin.altitude), # relative alt - urp.
                    vx=int(vel_n*100),
                    vy=int(vel_e*100),
                    vz=int(vel_d*100),
                    hdg=int(math.degrees(heading)*100)  # rad->cdeg
                )
                gpi.pack(self.mav.mav)
                self.mav.mav.send(gpi)
            self.assert_receive_message('GLOBAL_POSITION_INT')
            pos = self.get_location()
            delta = self.get_distance(expected_loc, pos)
            want_delta = foll_ofs_x
            self.progress("position delta=%f (want <%f)" % (delta, max_delta))
            if abs(want_delta - delta) < 3:
                break

        self.context_pop()
        self.do_RTL()
