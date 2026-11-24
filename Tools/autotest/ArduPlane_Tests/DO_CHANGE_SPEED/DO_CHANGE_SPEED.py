'''
Plane autotest: DO_CHANGE_SPEED

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class DO_CHANGE_SPEED(TestGroup):
    def context_push_do_change_speed(self):
        # the following lines ensure we revert these parameter values
        # - DO_CHANGE_AIRSPEED is a permanent vehicle change!
        self.context_push()
        self.set_parameters({
            "AIRSPEED_CRUISE": self.get_parameter("AIRSPEED_CRUISE"),
            "MIN_GROUNDSPEED": self.get_parameter("MIN_GROUNDSPEED"),
            "TRIM_THROTTLE": self.get_parameter("TRIM_THROTTLE"),
        })

    @testcase(bucket="PlaneTests1a")
    def DO_CHANGE_SPEED(self):
        '''Test DO_CHANGE_SPEED command/item'''
        self.set_parameters({
            "RTL_AUTOLAND": 1,
        })

        self.context_push_do_change_speed()
        self.DO_CHANGE_SPEED_mavlink_long()
        self.context_pop()

        self.set_current_waypoint(1)
        self.zero_throttle()

        self.context_push_do_change_speed()
        self.DO_CHANGE_SPEED_mavlink_int()
        self.context_pop()

        self.context_push_do_change_speed()
        self.DO_CHANGE_SPEED_mission()
        self.context_pop()

    def DO_CHANGE_SPEED_mission(self):
        '''test DO_CHANGE_SPEED as a mission item'''
        self.start_subtest("DO_CHANGE_SPEED_mission")
        self.load_mission("mission.txt")
        self.set_current_waypoint(1)

        self.progress("Takeoff")
        self.set_rc(3, 1000)
        self.takeoff(alt=10)
        self.set_rc(3, 1500)

        self.start_subtest("Check initial speed")

        self.change_mode('AUTO')

        checks = [
            (1, self.get_parameter("AIRSPEED_CRUISE")),
            (3, 10),
            (5, 20),
            (7, 15),
        ]

        for (current_waypoint, want_airspeed) in checks:
            self.wait_current_waypoint(current_waypoint, timeout=150)
            self.wait_airspeed(want_airspeed-1, want_airspeed+1, minimum_duration=5, timeout=120)

        self.fly_home_land_and_disarm()

    def DO_CHANGE_SPEED_mavlink_int(self):
        self.DO_CHANGE_SPEED_mavlink(self.run_cmd_int)

    def DO_CHANGE_SPEED_mavlink_long(self):
        self.DO_CHANGE_SPEED_mavlink(self.run_cmd)

    def DO_CHANGE_SPEED_mavlink(self, run_cmd_method):
        '''test DO_CHANGE_SPEED as a mavlink command'''
        self.progress("Takeoff")
        self.takeoff(alt=100, mode="TAKEOFF", timeout=120)
        self.set_rc(3, 1500)
        # ensure we know what the airspeed is:
        self.progress("Entering guided and flying somewhere constant")
        self.change_mode("GUIDED")
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            p5=12345, # lat* 1e7
            p6=12345, # lon* 1e7
            p7=100,   # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        )
        self.delay_sim_time(10, reason="vehicle to establish GUIDED position")
        self.progress("Ensuring initial speed is known and relatively constant")
        initial_speed = 22.0
        timeout = 15
        self.wait_airspeed(initial_speed-1, initial_speed+1, minimum_duration=5, timeout=timeout)

        self.start_subtest("Setting groundspeed")
        for new_target_groundspeed in initial_speed + 5, initial_speed + 2:
            run_cmd_method(
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                p1=1, # groundspeed
                p2=new_target_groundspeed,
                p3=-1, # throttle / no change
                p4=0, # absolute values
            )
            self.wait_groundspeed(new_target_groundspeed-2, new_target_groundspeed+2, timeout=80, minimum_duration=5)
            self.progress("Adding some wind, ensuring groundspeed holds")
            self.set_parameter("SIM_WIND_SPD", 5)
            self.delay_sim_time(5, reason="wind to take effect")
            self.wait_groundspeed(new_target_groundspeed-2, new_target_groundspeed+2, timeout=40, minimum_duration=5)
            self.set_parameter("SIM_WIND_SPD", 0)

        # clear target groundspeed
        run_cmd_method(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            p1=1, # groundspeed
            p2=0,
            p3=-1, # throttle / no change
            p4=0, # absolute values
        )

        self.start_subtest("Setting airspeed")
        for new_target_airspeed in initial_speed - 5, initial_speed + 5:
            run_cmd_method(
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                p1=0, # airspeed
                p2=new_target_airspeed,
                p3=-1, # throttle / no change
                p4=0, # absolute values
            )
            self.wait_airspeed(new_target_airspeed-2, new_target_airspeed+2, minimum_duration=5)

        self.context_push()
        self.progress("Adding some wind, hoping groundspeed increases/decreases")
        self.set_parameters({
            "SIM_WIND_SPD": 7,
            "SIM_WIND_DIR": 270,
        })
        self.delay_sim_time(5, reason="wind to take effect")
        timeout = 10
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not achieve groundspeed delta")
            m = self.assert_receive_message('VFR_HUD')
            delta = abs(m.airspeed - m.groundspeed)
            want_delta = 5
            self.progress("groundspeed and airspeed should be different (have=%f want=%f)" % (delta, want_delta))
            if delta > want_delta:
                break
        self.context_pop()

        # cancel minimum groundspeed:
        run_cmd_method(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            p1=0, # groundspeed
            p2=-2,  # return to default
            p3=0, # throttle / no change
            p4=0, # absolute values
        )
        # cancel airspeed:
        run_cmd_method(
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            p1=1, # airspeed
            p2=-2,  # return to default
            p3=0, # throttle / no change
            p4=0, # absolute values
        )

        self.start_subtest("Setting throttle")
        self.set_parameter('ARSPD_USE', 0)  # setting throttle only effective without airspeed
        for (set_throttle, expected_throttle) in (97, 79), (60, 51), (95, 77):
            run_cmd_method(
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                p1=3, # throttle
                p2=0,
                p3=set_throttle, # throttle / no change
                p4=0, # absolute values
            )
            self.wait_message_field_values('VFR_HUD', {
                "throttle": expected_throttle,
            }, minimum_duration=5, epsilon=5)

        self.fly_home_land_and_disarm(timeout=240)
