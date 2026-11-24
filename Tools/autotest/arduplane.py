'''
Fly ArduPlane in SITL

AP_FLAKE8_CLEAN
'''

import math
import operator
import os
import time

from pymavlink import mavutil
from pymavlink import quaternion

import vehicle_test_suite

from pysim import util
from vehicle_test_suite import AltFrame
from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
SITL_START_LOCATION = Location(-35.362938, 149.165085, 585, AltFrame.ABSOLUTE)
SITL_START_HEADING = 354
WIND = "0,180,0.2"  # speed,direction,variance


class AutoTestPlane(vehicle_test_suite.TestSuite):
    @staticmethod
    def get_not_armable_mode_list():
        return []

    @staticmethod
    def get_not_disarmed_settable_modes_list():
        return ["FOLLOW"]

    @staticmethod
    def get_no_position_not_settable_modes_list():
        return []

    @staticmethod
    def get_position_armable_modes_list():
        return ["GUIDED", "AUTO"]

    @staticmethod
    def get_normal_armable_modes_list():
        return ["MANUAL", "STABILIZE", "ACRO"]

    def log_name(self):
        return "ArduPlane"

    def test_filepath(self):
        return os.path.realpath(__file__)

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def sitl_start_heading(self):
        return SITL_START_HEADING

    test_root = "ArduPlane_Tests"

    def default_frame(self):
        return "plane-elevrev"

    def is_plane(self):
        return True

    def get_stick_arming_channel(self):
        return int(self.get_parameter("RCMAP_YAW"))

    def get_disarm_delay(self):
        return int(self.get_parameter("LAND_DISARMDELAY"))

    def set_autodisarm_delay(self, delay):
        self.set_parameter("LAND_DISARMDELAY", delay)

    def start_flying_mission(self, filename: str) -> None:
        self.progress(f"Flying mission {filename}")
        self.load_mission(filename)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

    def takeoff(self, alt=150, alt_max=None, relative=True, mode=None, timeout=None):
        """Takeoff to altitude."""

        if mode == "TAKEOFF":
            return self.takeoff_in_TAKEOFF(alt=alt, relative=relative, timeout=timeout)

        if mode == "QHOVER":
            return self.takeoff_in_QHOVER(alt=alt, relative=relative, timeout=timeout)

        return self.takeoff_in_FBWA(alt=alt, alt_max=alt_max, relative=relative, timeout=timeout)

    def takeoff_in_QHOVER(self, alt=20, relative=True, timeout=None):
        '''VTOL climb to altitude in QHOVER'''
        self.change_mode("QHOVER")
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.set_rc(3, 1700)  # throttle up to climb
        self.wait_altitude(alt-2, alt+3, relative=relative, timeout=timeout)
        self.set_rc(3, 1500)  # neutral throttle holds altitude in QHOVER

    def takeoff_in_TAKEOFF(self, alt=150, relative=True, mode=None, alt_epsilon=2, timeout=None):
        if relative is not True:
            raise ValueError("Only relative alt supported ATM")
        self.change_mode("TAKEOFF")
        self.context_push()
        self.set_parameter('TKOFF_ALT', alt)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_altitude(alt-alt_epsilon, alt+alt_epsilon, relative=True, timeout=timeout)
        self.context_pop()

    def takeoff_in_FBWA(self, alt=150, alt_max=None, relative=True, mode=None, timeout=30):
        if alt_max is None:
            alt_max = alt + 30

        self.change_mode("FBWA")

        self.wait_ready_to_arm()
        self.arm_vehicle()

        # some rudder to counteract the prop torque
        self.set_rc(4, 1700)

        # some up elevator to keep the tail down
        self.set_rc(2, 1200)

        # get it moving a bit first
        self.set_rc(3, 1300)
        self.wait_groundspeed(6, 100)

        # a bit faster again, straighten rudder
        self.set_rc_from_map({
            3: 1600,
            4: 1500,
        })
        self.wait_groundspeed(12, 100)

        # hit the gas harder now, and give it some more elevator
        self.set_rc_from_map({
            2: 1100,
            3: 2000,
        })

        # gain a bit of altitude
        self.wait_altitude(alt, alt_max, timeout=timeout, relative=relative)

        # level off
        self.set_rc(2, 1500)

        self.progress("TAKEOFF COMPLETE")

    def fly_LOITER(self, num_circles=4, timeout=60):
        """Loiter where we are."""
        self.progress("Testing LOITER for %u turns" % num_circles)
        self.change_mode('LOITER')

        m = self.assert_receive_message('VFR_HUD')
        initial_alt = m.alt
        self.progress("Initial altitude %u\n" % initial_alt)

        while num_circles > 0:
            self.wait_heading(0, accuracy=10, timeout=timeout)
            self.wait_heading(180, accuracy=10, timeout=timeout)
            num_circles -= 1
            self.progress("Loiter %u circles left" % num_circles)

        m = self.assert_receive_message('VFR_HUD')
        final_alt = m.alt
        self.progress("Final altitude %u initial %u\n" %
                      (final_alt, initial_alt))

        self.change_mode('FBWA')

        if abs(final_alt - initial_alt) > 20:
            raise NotAchievedException("Failed to maintain altitude")

        self.progress("Completed Loiter OK")

    def wait_level_flight(self, accuracy=5, timeout=30):
        """Wait for level flight."""
        tstart = self.get_sim_time()
        self.progress("Waiting for level flight")
        self.set_rc_from_map({1: 1500, 2: 1500, 4: 1500})
        while self.get_sim_time_cached() < tstart + timeout:
            m = self.assert_receive_message('ATTITUDE')
            roll = math.degrees(m.roll)
            pitch = math.degrees(m.pitch)
            self.progress("Roll=%.1f Pitch=%.1f" % (roll, pitch))
            if math.fabs(roll) <= accuracy and math.fabs(pitch) <= accuracy:
                self.progress("Attained level flight")
                return
        raise NotAchievedException("Failed to attain level flight")

    def change_altitude(self, altitude, accuracy=30, relative=False):
        """Get to a given altitude."""
        if relative:
            altitude += self.home_position_as_location().get_alt_m(AltFrame.ABSOLUTE)
        self.change_mode('FBWA')
        alt_error = self.mav.messages['VFR_HUD'].alt - altitude
        if alt_error > 0:
            self.set_rc(2, 2000)
        else:
            self.set_rc(2, 1000)
        self.wait_altitude(altitude-accuracy/2, altitude+accuracy/2)
        self.set_rc(2, 1500)
        self.progress("Reached target altitude at %u" %
                      self.mav.messages['VFR_HUD'].alt)
        return self.wait_level_flight()

    def set_attitude_target(self, tolerance=10):
        """Test setting of attitude target in guided mode."""
        self.change_mode("GUIDED")

        steps = [{"name": "roll-over",         "roll": 60, "pitch": 0,  "yaw": 0, "throttle": 0, "type_mask": 0b10000001},
                 {"name": "roll-back",         "roll": 0,  "pitch": 0,  "yaw": 0, "throttle": 0, "type_mask": 0b10000001},
                 {"name": "pitch-up+throttle", "roll": 0,  "pitch": 20, "yaw": 0, "throttle": 1, "type_mask": 0b11000010},
                 {"name": "pitch-back",        "roll": 0,  "pitch": 0,  "yaw": 0, "throttle": 0, "type_mask": 0b10000010}]

        state_wait = "wait"
        state_hold = "hold"
        try:
            for step in steps:
                step_start = self.get_sim_time_cached()
                state = state_wait
                state_start = self.get_sim_time_cached()
                while True:
                    m = self.mav.recv_match(type='ATTITUDE',
                                            blocking=True,
                                            timeout=0.1)
                    now = self.get_sim_time_cached()
                    if now - step_start > 30:
                        raise AutoTestTimeoutException("Maneuvers not completed")
                    if m is None:
                        continue

                    angle_error = 0
                    if (step["type_mask"] & 0b00000001) or (step["type_mask"] == 0b10000000):
                        angle_error += abs(math.degrees(m.roll) - step["roll"])

                    if (step["type_mask"] & 0b00000010) or (step["type_mask"] == 0b10000000):
                        angle_error += abs(math.degrees(m.pitch) - step["pitch"])

                    if (step["type_mask"] & 0b00000100) or (step["type_mask"] == 0b10000000):
                        # Strictly we should angle wrap, by plane doesn't support yaw correctly anyway so its not tested here
                        angle_error += abs(math.degrees(m.yaw) - step["yaw"])

                    # Note were not checking throttle, however the SITL plane needs full throttle to meet the
                    # target pitch attitude, Pitch test will fail without throttle override

                    if state == state_wait:
                        # Reduced tolerance for initial trigger
                        if angle_error < (tolerance * 0.25):
                            state = state_hold
                            state_start = now

                        # Allow 10 seconds to reach attitude
                        if (now - state_start) > 10:
                            raise NotAchievedException(step["name"] + ": Failed to get to set attitude")

                    elif state == state_hold:
                        # Give 2 seconds to stabilize
                        if (now - state_start) > 2 and not (angle_error < tolerance):
                            raise NotAchievedException(step["name"] + ": Failed to hold set attitude")

                        # Hold for 10 seconds
                        if (now - state_start) > 12:
                            # move onto next step
                            self.progress("%s Done" % (step["name"]))
                            break

                    self.progress("%s %s error: %f" % (step["name"], state, angle_error))

                    time_boot_millis = 0 # FIXME
                    target_system = 1 # FIXME
                    target_component = 1 # FIXME
                    type_mask = step["type_mask"] ^ 0xFF # FIXME
                    # attitude in radians:
                    q = quaternion.Quaternion([math.radians(step["roll"]),
                                               math.radians(step["pitch"]),
                                               math.radians(step["yaw"])])
                    self.mav.mav.set_attitude_target_send(time_boot_millis,
                                                          target_system,
                                                          target_component,
                                                          type_mask,
                                                          q,
                                                          0, # roll rate, not used in AP
                                                          0, # pitch rate, not used in AP
                                                          0, # yaw rate, not used in AP
                                                          step["throttle"])
        except Exception as e:
            self.change_mode('FBWA')
            self.set_rc(3, 1700)
            raise e

        # back to FBWA
        self.change_mode('FBWA')
        self.set_rc(3, 1700)
        self.wait_level_flight()

    def fly_mission(self, filename, mission_timeout=60.0, strict=True, quadplane=False):
        """Fly a mission from a file."""
        self.progress("Flying mission %s" % filename)
        num_wp = self.load_mission(filename, strict=strict)-1
        self.fly_mission_waypoints(num_wp, mission_timeout=mission_timeout, quadplane=quadplane)

    def fly_mission_waypoints(self, num_wp, mission_timeout=60.0, quadplane=False,
                              dist_to_final_wp_threshold_m: float | None = None):
        self.set_current_waypoint(0, check_afterwards=False)
        self.context_push()
        self.context_collect('STATUSTEXT')
        self.change_mode('AUTO')
        if dist_to_final_wp_threshold_m is None:
            dist_to_final_wp_threshold_m = 60.0
        self.wait_waypoint(1, num_wp, max_dist_to_final_wp_m=dist_to_final_wp_threshold_m, timeout=mission_timeout)
        self.wait_groundspeed(0, 0.5, timeout=mission_timeout)
        if quadplane:
            self.wait_statustext("Throttle disarmed", timeout=200, check_context=True)
        else:
            self.wait_statustext("Auto disarmed", timeout=60, check_context=True)
        self.context_pop()
        self.progress("Mission OK")

    def fly_deepstall_absolute(self):
        self.start_subtest("DeepStall Relative Absolute")
        deepstall_elevator_pwm = 1661
        self.set_parameters({
            "LAND_TYPE": 1,
            "LAND_DS_ELEV_PWM": deepstall_elevator_pwm,
            "RTL_AUTOLAND": 1,
        })
        self.start_flying_mission("plane-deepstall-mission.txt")
        self.progress("Waiting for deepstall messages")

        # note that the following two don't necessarily happen in this
        # order, but at very high speedups we may miss the elevator
        # PWM if we first look for the text (due to the get_sim_time()
        # in wait_servo_channel_value)

        self.context_collect('STATUSTEXT')

        # assume elevator is on channel 2:
        self.wait_servo_channel_value(2, deepstall_elevator_pwm, timeout=240)

        self.wait_text("Deepstall: Entry: ", check_context=True)

        self.disarm_wait(timeout=120)

        self.progress("Flying home")
        self.set_current_waypoint(0, check_afterwards=False)
        self.takeoff(10)
        self.set_parameter("LAND_TYPE", 0)
        self.fly_home_land_and_disarm()

    def fly_home_land_and_disarm(self, timeout=120):
        filename = "flaps.txt"
        self.progress("Using %s to fly home" % filename)
        n = self.load_generic_mission(filename)
        self.change_mode("AUTO")
        # don't set current waypoint to 8 unless we're distant from it
        # or we arrive instantly and never see it as our current
        # waypoint:
        self.wait_distance_to_waypoint(8, 100, 10000000)
        self.set_current_waypoint(8)
        self.wait_current_waypoint(n-1, timeout=timeout)
        self.wait_disarmed(60)

    def assert_fence_sys_status(self, present, enabled, health):
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE, present, enabled, health)

    def setup_simulated_parachute(self, extra_parameters=None):
        '''set the vehicle and its simulated parachute up, without firing it

        SIM_Parachute deploys as soon as the PWM on SIM_PARA_PIN reads 1250
        or more, and it begins watching that pin the moment the pin number
        is set.  Setting the pin in the same set_parameters() call as the
        release servo's function therefore races the servo output: until
        AP_Parachute has driven the channel to CHUTE_SERVO_OFF (1100 by
        default, below the trigger) it still holds whatever the last test
        left there, and anything at or above 1250 fires the chute during
        setup.  The "BANG!" then arrives before the test starts waiting for
        it, and the real release later in the test is silent because the
        chute has already gone.

        So configure the vehicle first, wait for the release servo to reach
        its off position, and only then let the simulation watch the pin.
        '''
        parameters = {
            "CHUTE_ENABLED": 1,
            "CHUTE_TYPE": 10,
            "SERVO9_FUNCTION": 27,
        }
        if extra_parameters is not None:
            parameters.update(extra_parameters)
        self.set_parameters(parameters)
        self.wait_servo_channel_value(9, 1250, comparator=operator.lt, timeout=10)
        self.set_parameters({
            "SIM_PARA_PIN": 9,
            "SIM_PARA_ENABLE": 1,
        })

    def sample_enable_parameter(self):
        return "Q_ENABLE"

    def rc_defaults(self):
        ret = super(AutoTestPlane, self).rc_defaults()
        ret[3] = 1000
        ret[8] = 1800
        return ret

    def initial_mode_switch_mode(self):
        return "MANUAL"

    def default_mode(self):
        return "MANUAL"

    def PIDTuning(self):
        '''Test PID Tuning'''
        self.change_mode("FBWA") # we don't update PIDs in MANUAL
        super(AutoTestPlane, self).PIDTuning()

    def wait_for_collision_threat_to_clear(self):
        '''wait to get a "clear" collision message", then slurp remaining
        messages'''
        last_collision = self.get_sim_time()
        while True:
            now = self.get_sim_time()
            if now - last_collision > 5:
                return
            self.progress("Waiting for collision message")
            m = self.mav.recv_match(type='COLLISION', blocking=True, timeout=1)
            self.progress("Got (%s)" % str(m))
            if m is None:
                continue
            last_collision = now

    def CPUFailsafe(self):
        '''In lockup Plane should copy RC inputs to RC outputs'''
        self.plane_CPUFailsafe()

    def fly_generic_mission(self, filename, mission_timeout=60.0, strict=True,
                            dist_to_final_wp_threshold_m: float | None = None):
        """Fly a mission from the Generic_Missions directory."""
        self.progress("Flying generic mission %s" % filename)
        num_wp = self.load_generic_mission(filename, strict=strict) - 1
        self.fly_mission_waypoints(num_wp, mission_timeout=mission_timeout,
                                   dist_to_final_wp_threshold_m=dist_to_final_wp_threshold_m)

    def fly_external_AHRS(self, sim, eahrs_type,
                          dist_to_final_wp_threshold_m: float | None = None,
                          max_position_lag_m=0.4):
        """Fly with external AHRS"""
        # The external AHRS drivers parse their serial stream on a
        # real-time thread.  At the default plane speedup (100x) that
        # thread cannot keep the simulated GPS within its health window
        # on a loaded CI runner, producing intermittent "GPS 1: not
        # healthy" arm failures.  Run these tests at a reduced speedup so
        # the thread keeps up.
        self.context_set_speedup(20)
        self.customise_SITL_commandline(["--serial4=sim:%s" % sim])

        self.set_parameters({
            "EAHRS_TYPE": eahrs_type,
            "SERIAL4_PROTOCOL": 36,
            "SERIAL4_BAUD": 230400,
            "GPS1_TYPE": 21,
            "AHRS_EKF_TYPE": 11,
            "INS_GYR_CAL": 1,
        })
        self.reboot_sitl()
        self.delay_sim_time(5, reason="external AHRS to initialise")
        self.progress("Running accelcal")
        self.run_cmd(
            mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
            p5=4,
            timeout=5,
        )

        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.fly_generic_mission("externalahrs.txt", dist_to_final_wp_threshold_m=dist_to_final_wp_threshold_m)
        self.check_EAHRS_position_lag(max_position_lag_m)

    def wait_and_maintain_wind_estimate(
            self,
            want_speed,
            want_dir,
            timeout=10,
            speed_tolerance=0.5,
            dir_tolerance=5,
            **kwargs):
        '''wait for wind estimate to reach speed and direction'''

        def validator(last, _min, _max):
            '''returns false of spd or direction is too-far wrong'''
            (spd, di) = last
            _min_spd, _min_dir = _min
            _max_spd, _max_dir = _max
            if spd < _min_spd or spd > _max_spd:
                return False
            # my apologies to whoever is staring at this and wondering
            # why we're not wrapping angles here...
            if di < _min_dir or di > _max_dir:
                return False
            return True

        def value_getter():
            '''returns a tuple of (wind_speed, wind_dir), where wind_dir is 45 if
            wind is coming from NE'''
            m = self.assert_receive_message("WIND")
            return (m.speed, m.direction)

        class ValueAverager(object):
            def __init__(self):
                self.speed_average = -1
                self.dir_average = -1
                self.count = 0.0

            def add_value(self, value):
                (spd, di) = value
                if self.speed_average == -1:
                    self.speed_average = spd
                    self.dir_average = di
                else:
                    self.speed_average += spd
                    self.dir_average += di
                self.count += 1
                return (self.speed_average/self.count, self.dir_average/self.count)

            def reset(self):
                self.count = 0
                self.speed_average = -1
                self.dir_average = -1

        self.wait_and_maintain_range(
            value_name="WindEstimates",
            minimum=(want_speed-speed_tolerance, want_dir-dir_tolerance),
            maximum=(want_speed+speed_tolerance, want_dir+dir_tolerance),
            current_value_getter=value_getter,
            value_averager=ValueAverager(),
            validator=lambda last, _min, _max: validator(last, _min, _max),
            timeout=timeout,
            **kwargs
        )

    def test_replay_bit(self, bit):

        self.context_push()
        current_log_filepath = bit()

        self.progress("Running replay on (%s) (%u bytes)" % (
            (current_log_filepath, os.path.getsize(current_log_filepath))
        ))

        # a log with dropped blocks is incomplete and cannot be replayed
        # faithfully, so check before wasting time on the replay itself:
        self.assert_log_has_no_dropped_blocks(current_log_filepath)

        self.zero_throttle()
        self.run_replay(current_log_filepath)

        replay_log_filepath = self.current_onboard_log_filepath()

        self.context_pop()

        self.progress("Replay log path: %s" % str(replay_log_filepath))

        check_replay = util.load_local_module("Tools/Replay/check_replay.py")

        ok = check_replay.check_log(replay_log_filepath, self.progress, verbose=True)
        if not ok:
            raise NotAchievedException("check_replay (%s) failed" % current_log_filepath)

    def test_replay_wind_and_airspeed_bit(self):
        self.set_parameters({
            "LOG_REPLAY": 1,
            "LOG_DISARMED": 1,
            "EK2_ENABLE": 1,

            "SIM_WIND_SPD": 5,
            "SIM_WIND_DIR": 45,
        })
        self.reboot_sitl()

        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_LOGGING, True, True, True)

        current_log_filepath = self.current_onboard_log_filepath()
        self.progress("Current log path: %s" % str(current_log_filepath))

        self.wait_ready_to_arm(require_absolute=True)
        # make sure airspeed is being used
        self.wait_sensor_state(mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE, True, True, True)

        self.takeoff(70)  # default wind sim wind is a sqrt function up to 60m
        self.change_mode('LOITER')
        self.wait_and_maintain_wind_estimate(5, 45, timeout=120)

        # make sure airspeed was fused at some point after some flying
        m = self.assert_receive_message('EKF_STATUS_REPORT')
        if m.airspeed_variance == 0:
            raise NotAchievedException("never fused airspeed")

        self.fly_home_land_and_disarm()

        self.reboot_sitl()

        return current_log_filepath

    def check_EAHRS_position_lag(self, max_mean_error_m):
        '''check logged AHRS position against simulator truth.  POS and
        SIM are written back-to-back by AP_AHRS logging, so each pair
        compares the estimate and the truth at effectively the same
        instant.  ExternalAHRS extrapolates its position forward by
        velocity from the last solution timestamp; a backend which
        fails to stamp that timestamp reports a position stale by up
        to one solution interval, an error proportional to speed.'''
        dfreader = self.dfreader_for_current_onboard_log()
        prev_sim = None
        pos = None
        count = 0
        error_sum = 0.0
        error_max = 0.0
        while True:
            m = dfreader.recv_match(type=['POS', 'SIM'])
            if m is None:
                break
            if m.get_type() == 'POS':
                pos = m
                continue
            sim = m
            last_sim = prev_sim
            prev_sim = sim
            if pos is None or last_sim is None:
                continue
            if sim.TimeUS - pos.TimeUS > 20000:
                # not the POS from this logging cycle; logging dropout?
                continue
            # only consider samples where the vehicle is moving
            # quickly; staleness shows up as an error proportional
            # to speed:
            dt = (sim.TimeUS - last_sim.TimeUS)*1e-6
            if dt <= 0:
                continue
            speed = self.get_distance(
                Location.latlon_only(last_sim.Lat, last_sim.Lng),
                Location.latlon_only(sim.Lat, sim.Lng)) / dt
            if speed < 15:
                continue
            error = self.get_distance(
                Location.latlon_only(pos.Lat, pos.Lng),
                Location.latlon_only(sim.Lat, sim.Lng))
            count += 1
            error_sum += error
            error_max = max(error_max, error)
        if count < 100:
            raise NotAchievedException(
                "Insufficient fast-flight POS/SIM samples (%u)" % count)
        mean_error = error_sum / count
        self.progress("AHRS-vs-truth position error: mean=%.3fm max=%.3fm n=%u" %
                      (mean_error, error_max, count))
        if mean_error > max_mean_error_m:
            raise NotAchievedException(
                "AHRS position lags simulator truth (mean error %.3fm > %.3fm)" %
                (mean_error, max_mean_error_m))

    # The gains a completed autotune saves.  The firmware writes these,
    # so the suite has no record of them and they would otherwise persist
    # for the rest of the session.
    autotune_saved_gains = [
        "PTCH2SRV_RMAX_DN", "PTCH2SRV_RMAX_UP", "PTCH2SRV_TCONST",
        "PTCH_RATE_D", "PTCH_RATE_FF", "PTCH_RATE_FLTD",
        "PTCH_RATE_FLTT", "PTCH_RATE_I", "PTCH_RATE_P",
        "RLL2SRV_RMAX", "RLL2SRV_TCONST",
        "RLL_RATE_D", "RLL_RATE_FF", "RLL_RATE_FLTD",
        "RLL_RATE_FLTT", "RLL_RATE_I", "RLL_RATE_P",
    ]

    def run_autotune(self):
        self.takeoff(100)
        self.change_mode('AUTOTUNE')
        self.context_collect('STATUSTEXT')
        tstart = self.get_sim_time()
        axis = "Roll"
        rc_value = 1000
        while True:
            timeout = 600
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not complete within %u seconds" % timeout)
            m = self.statustext_in_collections("%s: Finished" % axis)
            if m is not None:
                self.progress("Got %s" % str(m))
                if axis == "Roll":
                    axis = "Pitch"
                    # Center sticks to allow roll to return to neutral before starting pitch
                    self.set_rc_from_map({1: 1500, 2: 1500})
                    self.delay_sim_time(15, reason="vehicle to stabilise between tune axes")

                    # Reset toggle value so the initial input is in a consistent direction
                    rc_value = 1000

                elif axis == "Pitch":
                    break
                else:
                    raise ValueError("Bug: %s" % axis)
            # clear collected statustexts so the next poll stays cheap; the
            # autopilot re-sends "<axis>: Finished" continuously so we won't
            # miss it
            self.context_clear_collection("STATUSTEXT")
            self.delay_sim_time(2, reason="autotune iteration")

            if rc_value == 1000:
                rc_value = 2000
            elif rc_value == 2000:
                rc_value = 1000
            else:
                raise ValueError("Bug")

            if axis == "Roll":
                self.set_rc_from_map({1: rc_value, 2: 1500})
            elif axis == "Pitch":
                self.set_rc_from_map({1: 1500, 2: rc_value})
            else:
                raise ValueError("Bug")

        tdelta = self.get_sim_time() - tstart
        self.progress("Finished in %0.1f seconds" % (tdelta,))

        self.set_rc_from_map({1: 1500, 2: 1500})

        self.change_mode('FBWA')
        self.fly_home_land_and_disarm(timeout=tdelta+240)

    def EFITest(self, efi_type, name, sim_name, check_fuel_flow=True):
        '''method to be called by EFI tests'''
        self.start_subtest("EFI Test for (%s)" % name)
        self.assert_not_receiving_message('EFI_STATUS')
        self.set_parameters({
            'SIM_EFI_TYPE': efi_type,
            'EFI_TYPE': efi_type,
            'SERIAL5_PROTOCOL': 24,
            'RPM1_TYPE': 10,
        })

        self.customise_SITL_commandline(
            ["--serial5=sim:%s" % sim_name,
             ],
        )
        self.wait_ready_to_arm()

        baro_m = self.assert_receive_message("SCALED_PRESSURE")
        self.progress(self.dump_message_verbose(baro_m))
        baro_temperature = baro_m.temperature / 100.0  # cDeg->deg
        m = self.assert_received_message_field_values("EFI_STATUS", {
            "throttle_out": 0,
            "health": 1,
        }, very_verbose=1)

        if abs(baro_temperature - m.intake_manifold_temperature) > 1:
            raise NotAchievedException(
                "Bad intake manifold temperature (want=%f got=%f)" %
                (baro_temperature, m.intake_manifold_temperature))

        self.arm_vehicle()

        self.set_rc(3, 1300)

        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > 10:
                raise NotAchievedException("RPM1 and EFI_STATUS.rpm did not match")
            rpm_m = self.assert_receive_message("RPM", verbose=1)
            want_rpm = 1000
            if rpm_m.rpm1 < want_rpm:
                continue

            m = self.assert_receive_message("EFI_STATUS", verbose=1)
            if abs(m.rpm - rpm_m.rpm1) > 100:
                continue

            break

        self.progress("now we're started, check a few more values")
        # note that megasquirt drver doesn't send throttle, so throttle_out is zero!
        m = self.assert_received_message_field_values("EFI_STATUS", {
            "health": 1,
        }, very_verbose=1)
        m = self.wait_message_field_values("EFI_STATUS", {
            "throttle_position": 31,
            "intake_manifold_temperature": 28,
        }, very_verbose=1, epsilon=2)

        if check_fuel_flow:
            if abs(m.fuel_flow - 0.2) < 0.0001:
                raise NotAchievedException("Expected fuel flow")

        self.set_rc(3, 1000)

        # need to force disarm as the is_flying flag can trigger with the engine running
        self.disarm_vehicle(force=True)

    def _MAV_CMD_DO_FLIGHTTERMINATION(self, command):
        self.set_parameters({
            "AFS_ENABLE": 1,
            "MAV_GCS_SYSID": self.mav.source_system,
            "AFS_TERM_ACTION": 42,
        })
        # AFS_TERMINATE magically set-and-saved by code:
        self.context_preserve_parameters(["AFS_TERMINATE"])
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.context_collect('STATUSTEXT')
        command(mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, p1=1)
        self.wait_disarmed()
        self.wait_text('Terminating due to GCS request', check_context=True)
        self.reboot_sitl()

    def install_planedaa_scripts(self):
        """Install planedaa.lua and every module it requires.

        planedaa.lua is split so that the applet holds avoidance POLICY and the modules
        hold mechanism; a module missing at runtime is a load failure, not a degraded
        mode, so they are installed together and this is the single place to add another.
        """
        self.install_applet_script_context("planedaa.lua")
        for module in ("mavlink_wrappers.lua", "daageo.lua", "daaobs.lua", "daaltr.lua", "daacore.lua"):
            self.install_script_module_context(
                self.script_modules_source_path(module),
                module,
            )
        # planedaa + its modules need more than the default 200 KB SITL heap to run.
        self.set_parameters({"SCR_HEAP_SIZE": 262144})

    def daa_watch_no_reaction(self, duration, forbidden_texts, inject):
        '''Common body for the ParkedXIgnored tests below: call inject() and check for no
        STATUSTEXT containing any of forbidden_texts, repeatedly for duration (sim) seconds.
        Paced by wait_heartbeat(), not a real-time-blocking recv_match() - the default
        autotest SITL speedup (100x) means a real-time wait of even one second lets the sim
        clock run far past a modest duration budget, starving inject() of most of its
        intended calls. Relies on the caller's own context_collect('STATUSTEXT') rather than
        consuming messages itself, so it never races a caller also reading STATUSTEXT
        directly (e.g. to wait for an expected, non-forbidden announcement).'''
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < duration:
            inject()
            self.wait_heartbeat()
            for text in forbidden_texts:
                if self.statustext_in_collections(text) is not None:
                    raise NotAchievedException(
                        "planedaa reacted to a parked contact: %s" % text)

    def _PlaneDAAModeChangeDuringDetection(self, trap):
        '''Place a pilot mode change inside the cycle, after the navigation snapshot.

        Changing mode from the test over MAVLink cannot reliably hit this window.
        Instrument only the installed applet, immediately before the decision under
        test, and require the production guard to leave that change in force.
        '''
        self.install_planedaa_scripts()
        script = self.installed_script_path("planedaa.lua")
        with open(script) as f:
            content = f.read()
        if trap:
            anchor = "        local mode_now = vehicle:get_mode()\n"
        else:
            anchor = "    function DAA.avoid(new_target_loc)\n"
        injection = '''        if aircraft_avoiding ~= nil and current_mode == PLANE_MODE.AUTO then
            assert(vehicle:set_mode(PLANE_MODE.MANUAL))
            gcs:send_text(MAV_SEVERITY.WARNING, "DAA test pilot takeover")
        end
'''
        if content.count(anchor) != 1:
            raise NotAchievedException("DAA takeover injection point changed")
        if trap:
            content = content.replace(anchor, injection + anchor)
        else:
            content = content.replace(anchor, anchor + injection)
        self.install_script_content("planedaa.lua", content)
        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,
            "AVD_ENABLE": 1,
            "AVD_WCLR_XY": 300,
            "AVD_WCLR_Z": 100,
            "AVD_NMAC_XY": 150,
            "AVD_NMAC_Z": 50,
        })
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.set_parameters({
            "DAA_AVD_ALT": 50,
            "DAA_AVD_ALT_TP": 1,
            "DAA_TRAP_ACT": 1 if trap else 0,
            "DAA_TRAP_S": 0,
        })
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3000, 0, 100),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 30:
            here = self.get_location()
            contact = self.offset_location_ne(here, 0, 100)
            self.mav.mav.adsb_vehicle_send(
                0xABC123, int(contact.lat * 1e7), int(contact.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000),
                0, 0, 0, b"TAKEOVER", mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,
                1, 65535, 1200,
            )
            self.wait_heartbeat()
            if self.statustext_in_collections("DAA test pilot takeover") is not None:
                break
        else:
            raise NotAchievedException("Did not exercise the DAA takeover window")

        # Watch long enough to see a command sent in the injection cycle. Force
        # disarm afterwards: MANUAL is intentionally left in the pilot's control.
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 5:
            self.wait_heartbeat()
            if self.mav.flightmode != "MANUAL":
                raise NotAchievedException("DAA overrode pilot takeover: %s" % self.mav.flightmode)
        for text in ("LOITER AIRCRAFT", "TRAPPED ("):
            if self.statustext_in_collections(text) is not None:
                raise NotAchievedException("DAA commanded takeover after pilot change: %s" % text)
        self.disarm_vehicle(force=True)

    def _PlaneDAALoiterHandbackModeChange(self, force_stop):
        '''Inject an external mode change at the handback boundary.

        Instrument only the installed loiter module so the takeover lands after
        the applet's live-mode guard, immediately before stop() can restore AUTO.
        Cover expired-cooldown release and forced cleanup with a cooldown pending.
        '''
        self.install_planedaa_scripts()
        module = self.installed_script_module_path("daaltr.lua")
        with open(module) as f:
            content = f.read()
        anchor = "    function self.stop(force_stop)\n"
        if content.count(anchor) != 1:
            raise NotAchievedException("DAA handback injection point changed")
        injection = '''        if self.active and force_stop == %s then
            assert(vehicle:set_mode(PLANE_MODE.RTL))
            gcs:send_text(MAV_SEVERITY.WARNING, "DAA test handback takeover")
        end
''' % ("true" if force_stop else "false")
        with open(module, "w") as f:
            f.write(content.replace(anchor, anchor + injection))

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,
            "AVD_ENABLE": 1,
            "AVD_WCLR_XY": 300,
            "AVD_WCLR_Z": 100,
            "FENCE_ENABLE": 0,
            "RC7_OPTION": 308,
        })
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        self.set_parameters({
            "DAA_AVD_ALT": 50,
            "DAA_AVD_ALT_TP": 1,
            "DAA_TRAP_ACT": 0,
            "DAA_LTR_COOL_S": 60 if force_stop else 0,
        })
        self.set_rc(7, 1000)
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3000, 0, 100),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        def inject_aircraft():
            here = self.get_location()
            contact = self.offset_location_ne(here, 0, 100)
            self.mav.mav.adsb_vehicle_send(
                0xABC125, int(contact.lat * 1e7), int(contact.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000),
                0, 0, 0, b"HANDBACK", mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,
                1, 65535, 1200,
            )

        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 60:
            inject_aircraft()
            self.wait_heartbeat()
            if self.mav.flightmode == "GUIDED":
                break
        else:
            raise NotAchievedException("Did not engage the aircraft loiter before handback")

        if force_stop:
            # Keep the contact present: switching DAA off must exercise forced
            # cleanup, rather than passing through an ordinary traffic release.
            self.set_rc(7, 2000)
        # Otherwise stop injecting and let the contact expire. With zero cooldown
        # stop(false) must release immediately, which exposed the original defect.
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 30:
            if force_stop:
                inject_aircraft()
            self.wait_heartbeat()
            if self.statustext_in_collections("DAA test handback takeover") is not None:
                break
        else:
            raise NotAchievedException("Did not exercise the DAA handback window")

        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 5:
            if force_stop:
                inject_aircraft()
            self.wait_heartbeat()
            if self.mav.flightmode != "RTL":
                raise NotAchievedException("DAA overrode handback takeover: %s" % self.mav.flightmode)
        if self.statustext_in_collections("Loiter Done set mode:") is not None:
            raise NotAchievedException("DAA restored the saved mode over the handback takeover")
        self.disarm_vehicle(force=True)

    def _PlaneDAATrapResumeDuringLoiter(self, dropout=None):
        self.install_planedaa_scripts()
        if dropout is not None:
            with open(self.installed_script_path("planedaa.lua")) as f:
                content = f.read()
            anchor = "    function DAA.get_vehicle_state(fetched_current_loc)\n"
            content = content.replace(anchor, "    local test_trap_dropout_cycle = nil\n" + anchor)
            anchor = "        current_loc = fetched_current_loc\n"
            injection = '''
        if test_trap_dropout_cycle == nil and trap_active and loiteralt.active and
            current_mode == trap_fs_mode then
            test_trap_dropout_cycle = 0
        end
        if test_trap_dropout_cycle ~= nil and test_trap_dropout_cycle < 4 then
            test_trap_dropout_cycle = test_trap_dropout_cycle + 1
            if test_trap_dropout_cycle <= 3 then
                %s
                %s
            else
                gcs:send_text(6, "DAA test trapped dropout ended")
            end
        end
''' % (
                "current_target_loc = nil" if dropout == "target" else "current_loc = nil",
                '''assert(vehicle:set_mode(PLANE_MODE.MANUAL))
                current_mode = vehicle:get_mode()''' if dropout == "pilot takeover" else "-- trap retains ownership")
            if content.count(anchor) != 1:
                raise NotAchievedException("DAA trapped dropout injection point changed")
            self.install_script_content("planedaa.lua", content.replace(anchor, anchor + injection))

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "ADSB_TYPE": 1,     # MAVLink: ingest ADSB_VEHICLE with no ADS-B hardware
            "AVD_ENABLE": 1,
            "FENCE_ENABLE": 0,
            "AVD_WCLR_XY": 300,
            "AVD_WCLR_Z": 50,
            "AVD_NMAC_XY": 100,
            "AVD_NMAC_Z": 50,
        })

        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.set_parameters({
            "DAA_AVD_ALT": 50,     # required for the aircraft loiter (GUIDED) to engage
            "DAA_TRAP_ACT": 1,     # RTL
            "DAA_TRAP_S": 2,
            "DAA_TRAP_CLR_S": 3,
        })

        icao = 0xF000A4

        def inject_aircraft(distance_m):
            here = self.get_location()
            contact = self.offset_location_ne(here, 0, distance_m)
            self.mav.mav.adsb_vehicle_send(
                icao,
                int(contact.lat * 1e7),
                int(contact.lng * 1e7),
                mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,
                int(here.get_alt_m(AltFrame.ABSOLUTE) * 1000),
                0, 0, 0,
                "GAJET04".encode("ascii"),
                mavutil.mavlink.ADSB_EMITTER_TYPE_LIGHT,   # crewed -> find_aircraft/loiter path
                1, 65535, 1200,
            )

        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 60),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3000, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        # --- engage the aircraft loiter (GUIDED) with the contact outside NMAC range
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 60:
            inject_aircraft(200)
            if self.mav.flightmode == "GUIDED":
                break
            self.wait_heartbeat()
        if self.mav.flightmode != "GUIDED":
            raise NotAchievedException("did not engage the aircraft loiter (GUIDED)")

        # --- close to inside NMAC range and hold until the trap fires (RTL)
        tstart = self.get_sim_time()
        while self.get_sim_time() - tstart < 30:
            inject_aircraft(60)
            pilot_takeover = dropout == "pilot takeover" and self.mav.flightmode == "MANUAL"
            if self.mav.flightmode == "RTL" or pilot_takeover:
                break
            self.wait_heartbeat()
        if self.mav.flightmode != "RTL" and not (dropout == "pilot takeover" and self.mav.flightmode == "MANUAL"):
            raise NotAchievedException("trapped-failsafe never fired for the near-miss contact")

        # --- withdraw the contact (stop injecting - AP_Avoidance prunes it after 5 s)
        #     so the trap's recovery resumes into a clear sky rather than immediately
        #     re-tripping, then wait past DAA_TRAP_CLR_S for the recovery to GUIDED and,
        #     since the contact is gone, the aircraft loiter's own release back to AUTO.
        if dropout is not None:
            self.wait_text("DAA test trapped dropout ended", check_context=True, timeout=30)
        if dropout == "pilot takeover":
            self.wait_mode("MANUAL", timeout=10)
            self.delay_sim_time(10, reason="verify the trap and loiter preserve pilot ownership")
            if self.mav.flightmode != "MANUAL":
                raise NotAchievedException("dropout recovery overrode pilot mode")
        else:
            self.wait_mode("AUTO", timeout=60)
        self.disarm_vehicle(force=True)

    def _PlaneDAAHungTrapFires(self, alert_only, cancel=None):
        self.install_planedaa_scripts()

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "AVD_ENABLE": 1,
            "FENCE_ENABLE": 0,  # enabled in flight so arming is unimpeded
            "FENCE_ACTION": 0,  # report only: the core must not rescue this, that is the point
            "FENCE_TYPE": 4,    # polygon/circle fences
        })

        home = self.home_position_as_location()
        excl_north_m = 1600
        excl_radius_m = 300
        margin_m = 150                          # keep-out = 450 m from the centre
        wp_from_centre_m = 350                  # 100 m inside the keep-out, 50 m outside the fence
        self.upload_fences_from_locations([(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            {"radius": excl_radius_m, "loc": self.offset_location_ne(home, excl_north_m, 0)},
        )])

        trap_status = {"seen": False, "holding": True, "value": None, "cleared_early": False}
        avoid_values = []

        def watch_trap_status(mav, message):
            if message.get_type() != 'NAMED_VALUE_STRING' or message.name != 'DAA-AVOID':
                return
            avoid_values.append(message.value)
            if message.value == "TRAPPED":
                trap_status["seen"] = True
            elif trap_status["seen"] and trap_status["holding"]:
                trap_status["cleared_early"] = True
            trap_status["value"] = message.value

        self.install_message_hook_context(watch_trap_status)
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.set_parameters({
            "WP_RADIUS": 20,            # the shortfall must not be swallowed by acceptance
            "DAA_MARGIN_FENCE": margin_m,
            "DAA_HUNG_ALRT_S": 20,
            "DAA_TRAP_S": 0,  # latch the genuine HUNG event before the orbit geometry changes
            "DAA_TRAP_ACT": 0 if alert_only else 1,  # RTL: no VTOL on this airframe, and it retargets to
                                        # home - which is what the release rule must survive
            # The 100 m shortfall this test is built around (excl_radius/margin/
            # wp_from_centre above) was tuned against 1000 m probe/target-placement
            # distances - pin them explicitly rather than riding whatever DAA_LKAHD_M/
            # DAA_PLAN_M's shipping defaults are. A smaller DAA_PLAN_M in particular lets
            # the commanded avoidance target sit close enough to the real waypoint that
            # the mission reports it "skipped" without the vehicle ever truly closing the
            # standoff - collapsing the irreconcilable-geometry deadlock this test needs.
            "DAA_LKAHD_M": 1000,
            "DAA_PLAN_M": 1000,
        })

        # waypoint 2 sits inside the keep-out on the near side: approachable to 450 m from
        # the centre and no closer, so it can never be reached.  Waypoint 3 is well south of
        # the circle and reachable, for the release.
        self.start_flying_simple_relhome_mission([
            (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 50),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, excl_north_m - wp_from_centre_m, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 400, 0, 80),
            (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
        ])
        self.wait_current_waypoint(2, timeout=120)
        self.do_fence_enable()
        self.wait_text("Plane DAA", check_context=True, timeout=60)

        def require_text(text, why, timeout=120):
            try:
                self.wait_text(text, check_context=True, timeout=timeout)
            except AutoTestTimeoutException:
                raise NotAchievedException(why)

        require_text(
            "HUNG: no progress",
            "avoidance orbited an unreachable waypoint without ever reporting it as hung",
            timeout=300)
        if alert_only:
            self.set_current_waypoint(3)
            require_text("HUNG clear", "ordinary HUNG recovery did not announce clearing", timeout=30)
            self.wait_mode("AUTO", timeout=10)
            if self.statustext_in_collections("TRAPPED") is not None:
                raise NotAchievedException("alert-only HUNG unexpectedly engaged the trap")
            self.do_fence_disable()
            self.disarm_vehicle(force=True)
            return
        self.progress("hung avoidance detected - the failsafe must follow")

        require_text(
            "TRAPPED (hung)",
            "the hung avoidance was alerted but never became a compromise: DAA_TRAP_ACT "
            "did not fire, so the vehicle is still orbiting",
            timeout=60)
        self.wait_mode("RTL", timeout=30)

        self.delay_sim_time(5, reason="verify TRAPPED status remains visible while RTL is held")
        if not trap_status["seen"]:
            raise NotAchievedException("GCS never received the TRAPPED named-string status")
        if trap_status["cleared_early"] or trap_status["value"] != "TRAPPED":
            raise NotAchievedException("GCS TRAPPED status cleared while the hung trap still held RTL")
        if self.statustext_in_collections("HUNG clear") is not None:
            raise NotAchievedException("HUNG clear announced while the hung trap still held RTL")
        trap_status["holding"] = False
        release_index = len(avoid_values)
        if cancel is not None:
            if cancel == "disabled":
                self.set_parameter("DAA_TRAP_ACT", 0)
                expected_mode = "RTL"
            else:
                self.change_mode(cancel)
                expected_mode = cancel
            self.delay_sim_time(7, reason="receive cancellation status after the parameter refresh")
            if "" not in avoid_values[release_index:]:
                raise NotAchievedException("GCS TRAPPED status was not cleared on " + cancel)
            if "TRAPPED" in avoid_values[release_index:]:
                raise NotAchievedException("trap re-engaged after " + cancel)
            if self.mav.flightmode != expected_mode:
                raise NotAchievedException("trap cancellation overrode the current mode")
            self.do_fence_disable()
            self.disarm_vehicle(force=True)
            return
        # the pilot advances the mission past the unreachable waypoint: the geometry that
        # hung the vehicle is no longer what it is being asked to fly, so the trap must let go
        self.set_current_waypoint(3)
        require_text(
            "trap clear (mission moved)",
            "the mission was advanced to a reachable waypoint but the hung trap held on",
            timeout=60)
        self.wait_mode("AUTO", timeout=30)
        self.delay_sim_time(1, reason="receive the trap-release named-string status")
        if "" not in avoid_values[release_index:]:
            raise NotAchievedException("GCS TRAPPED status was not cleared on release")
        self.progress("hung trap released after the mission moved on")

        self.set_parameter("DAA_TRAP_ACT", 0)   # stand the failsafe down before cleanup
        self.do_fence_disable()
        self.disarm_vehicle(force=True)

    def _PlaneDAAAltitudeFenceBand(self, loiter, guided=False, dropout=None):
        self.install_planedaa_scripts()
        if dropout is not None:
            # Inject at the Lua state-fetch boundary: GPS loss alone need not make
            # AHRS position unavailable. Refuse cleanup commands for three cycles.
            with open(self.installed_script_path("planedaa.lua")) as f:
                content = f.read()
            anchor = "    function DAA.get_vehicle_state(fetched_current_loc)\n"
            content = content.replace(anchor, "    local test_dropout_cycle = nil\n" + anchor)
            anchor = "        current_loc = fetched_current_loc\n"
            injection = '''
        if test_dropout_cycle == nil and daa_target_loc ~= nil and current_mode == PLANE_MODE.GUIDED then
            test_dropout_cycle = 0
            DAA.enabled = false
        end
        if test_dropout_cycle ~= nil and test_dropout_cycle < 4 then
            test_dropout_cycle = test_dropout_cycle + 1
            if test_dropout_cycle <= 3 then
                %s
            else
                %s
                gcs:send_text(6, "DAA test dropout ended")
            end
        end
''' % ("current_target_loc = nil" if dropout == "target" else "current_loc = nil", {
                "new goal": '''local goal = navigation_target_loc:copy()
                goal:set_alt_m(110, 1)
                assert(vehicle:set_target_location(goal))
                current_target_loc = vehicle:get_target_location()''',
                "mode change": '''assert(vehicle:set_mode(PLANE_MODE.RTL))
                current_mode = vehicle:get_mode()
                current_target_loc = vehicle:get_target_location()''',
            }.get(dropout, "DAA.enabled = true"))
            if content.count(anchor) != 1:
                raise NotAchievedException("DAA dropout injection point changed")
            content = content.replace(anchor, anchor + injection)
            anchor = "    local function update_target_location(new_target_loc)\n"
            content = content.replace(anchor, anchor + '''
        if test_dropout_cycle ~= nil and test_dropout_cycle <= 3 then return false end
''')
            self.install_script_content("planedaa.lua", content)
        self.install_script_content_context("daa-altitude-band-target.lua", """
local SCRIPT_NAME = "DAA Altitude Band Test"
local SCRIPT_NAME_SHORT = "DAABand"
local SCRIPT_VERSION = "1.0.0"
local function update()
    local target = vehicle:get_target_location()
    if target then
        local altitude = target:get_alt_m(1)
        if altitude then gcs:send_named_float(SCRIPT_NAME_SHORT, altitude) end
    end
    return update, 200
end
gcs:send_text(6, SCRIPT_NAME .. " " .. SCRIPT_VERSION)
return update, 200
""")
        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "AVD_ENABLE": 1,
            "FENCE_ENABLE": 0,
            "FENCE_TYPE": 9,
            "FENCE_ACTION": 0,
            "FENCE_MARGIN": 2,
            "FENCE_ALT_MIN_TP": 1,
            "FENCE_ALT_MAX_TP": 1,
            "RC7_OPTION": 308,
        })
        self.set_rc(7, 1000)
        scenarios = [
            # Both reported failures, plus a normal-width control and mixed frames.
            dict(name="narrow band", lo=90, hi=100, margin=20, cruise=95, target=95, max_frame=1),
            dict(name="large DAA buffer", lo=50, hi=100, margin=50, cruise=75, target=75, max_frame=1),
            dict(name="normal band", lo=50, hi=150, margin=20, cruise=140, target=128, max_frame=1),
            dict(name="mixed altitude frames", lo=90, hi=100, margin=20, cruise=95, target=95, max_frame=0),
        ]
        if guided:
            scenarios = [scenarios[2]]
        if loiter:
            scenarios = [scenarios[0], scenarios[2],
                         dict(name="unclamped loiter control", lo=50, hi=150, margin=20,
                              cruise=95, target=95, max_frame=1)]
        for scenario in scenarios:
            self.start_subtest(scenario["name"])
            self.context_push()
            self.context_collect('STATUSTEXT')
            home_alt = self.home_position_as_location().get_alt_m(AltFrame.ABSOLUTE)
            self.set_parameters({
                "FENCE_ALT_MIN": scenario["lo"],
                "FENCE_ALT_MAX": scenario["hi"] + (home_alt if scenario["max_frame"] == 0 else 0),
                "FENCE_ALT_MAX_TP": scenario["max_frame"],
            })
            self.reboot_sitl()
            self.wait_ready_to_arm()
            self.set_parameters({"DAA_MARGIN_ALT": scenario["margin"],
                                 "DAA_TRAP_ACT": 1 if loiter else 0,
                                 "DAA_HUNG_ALRT_S": 20, "DAA_TRAP_S": 2})
            # No traffic is injected: altitude-fence handling alone must stay contained.
            self.start_flying_simple_relhome_mission([
                (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, scenario["cruise"]),
                (mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM if loiter else
                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1000 if loiter else 6000, 0, scenario["cruise"]),
                (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0),
            ])
            self.wait_current_waypoint(2, timeout=180)
            self.wait_altitude(scenario["cruise"]-2, scenario["cruise"]+2,
                               relative=True, minimum_duration=2, timeout=120)
            self.wait_text("Plane DAA", check_context=True, timeout=60)
            if guided:
                self.change_mode("GUIDED")
                goal = self.offset_location_ne(self.get_location(), 3000, 0)
                self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                                 p5=int(goal.lat * 1e7), p6=int(goal.lng * 1e7),
                                 p7=home_alt + scenario["cruise"],
                                 frame=mavutil.mavlink.MAV_FRAME_GLOBAL)
            if loiter:
                self.wait_distance_to_location(self.offset_location_ne(self.home_position_as_location(), 1000, 0),
                                               50, 150, timeout=120)
            self.do_fence_enable()
            if dropout is not None:
                self.wait_text("DAA test dropout ended", check_context=True, timeout=30)
                if dropout == "mode change":
                    self.wait_mode("RTL", timeout=10)
                else:
                    self.wait_message_field_values('NAMED_VALUE_FLOAT', {
                        'name': 'DAABand', 'value': 110 if dropout == "new goal" else scenario["target"],
                    }, epsilon=0.2, timeout=30)
                self.do_fence_disable()
                if dropout != "mode change":
                    expected = 110 if dropout == "new goal" else scenario["cruise"]
                    self.wait_message_field_values('NAMED_VALUE_FLOAT', {
                        'name': 'DAABand', 'value': expected,
                    }, epsilon=0.2, timeout=30)
                self.delay_sim_time(5, reason="verify recovery respects current ownership")
                if self.mav.flightmode != ("RTL" if dropout == "mode change" else "GUIDED"):
                    raise NotAchievedException("dropout recovery changed flight mode")
                self.disarm_vehicle(force=True)
                self.context_pop()
                continue
            self.wait_message_field_values('NAMED_VALUE_FLOAT', {
                'name': 'DAABand', 'value': scenario["target"],
            }, epsilon=0.2, timeout=30)

            failures = []

            def check_target(mav, message):
                if message.get_type() == 'NAMED_VALUE_FLOAT' and message.name == 'DAABand':
                    if abs(message.value - scenario["target"]) > 0.2:
                        failures.append("target changed to %.2f m" % message.value)
                elif message.get_type() == 'GLOBAL_POSITION_INT':
                    altitude = message.relative_alt * 0.001
                    if not scenario["lo"] <= altitude <= scenario["hi"]:
                        failures.append("aircraft left fence band at %.2f m" % altitude)

            self.install_message_hook_context(check_target)
            self.delay_sim_time(45 if loiter else 12, reason="verify stable target and physical containment")
            if loiter:
                if self.mav.flightmode != "AUTO":
                    raise NotAchievedException("altitude-only avoidance interrupted mission loiter")
                if self.statustext_in_collections("HUNG:") is not None:
                    raise NotAchievedException("altitude-only avoidance reported false HUNG")
            if failures:
                raise NotAchievedException("%s: %s" % (scenario["name"], failures[0]))
            if self.statustext_in_collections("fence breached") is not None:
                raise NotAchievedException("altitude fence breached during %s" % scenario["name"])
            if scenario["name"] == "normal band":
                # Force the revert path while the fence still clamps the goal.
                # Merely lifting an active clamp does not exercise goal mutation.
                self.set_rc(7, 2000)
                self.delay_sim_time(2, reason="clear avoidance while the altitude clamp remains enabled")
                self.set_rc(7, 1000)
                self.set_parameter("FENCE_ALT_MAX", 200)
                self.wait_message_field_values('NAMED_VALUE_FLOAT', {
                    'name': 'DAABand', 'value': scenario["cruise"],
                }, epsilon=0.2, timeout=30)
                self.set_parameter("FENCE_ALT_MAX", 150)
                self.wait_message_field_values('NAMED_VALUE_FLOAT', {
                    'name': 'DAABand', 'value': scenario["target"],
                }, epsilon=0.2, timeout=30)
            self.do_fence_disable()
            self.wait_message_field_values('NAMED_VALUE_FLOAT', {
                'name': 'DAABand', 'value': scenario["cruise"],
            }, epsilon=0.2, timeout=30)
            if guided:
                self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                                 p5=int(goal.lat * 1e7), p6=int(goal.lng * 1e7), p7=110,
                                 frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT)
                self.wait_message_field_values('NAMED_VALUE_FLOAT', {
                    'name': 'DAABand', 'value': 110,
                }, epsilon=0.2, timeout=30)
                self.delay_sim_time(3, reason="verify the new GUIDED goal remains in charge")
                self.wait_message_field_values('NAMED_VALUE_FLOAT', {
                    'name': 'DAABand', 'value': 110,
                }, epsilon=0.2, timeout=5)
            self.disarm_vehicle(force=True)
            self.context_pop()

    class ValidateVFRHudClimbAgainstSimState(vehicle_test_suite.TestSuite.MessageHook):
        '''monitors VFR_HUD to make sure reported climbrate is in-line with SIM_STATE.vd'''
        def __init__(self, suite, max_allowed_divergence=5):
            super(AutoTestPlane.ValidateVFRHudClimbAgainstSimState, self).__init__(suite)
            self.max_allowed_divergence = max_allowed_divergence  # m/s
            self.max_divergence = 0
            self.vfr_hud = None
            self.sim_state = None
            self.last_print = 0
            self.min_print_interval = 1  # seconds
            self.instafail = True
            self.failed = False

        def progress_prefix(self):
            return "VVHCASS: "

        def process(self, mav, m):
            if m.get_type() == 'VFR_HUD':
                self.vfr_hud = m
            elif m.get_type() == 'SIM_STATE':
                self.sim_state = m
            if self.vfr_hud is None:
                return
            if self.sim_state is None:
                return

            vfr_hud_climb = self.vfr_hud.climb
            sim_state_climb = -self.sim_state.vd
            divergence = abs(vfr_hud_climb - sim_state_climb)
            if (time.time() - self.last_print > self.min_print_interval or
                    divergence > self.max_divergence):
                self.progress(f"climb delta is {divergence}")
                self.last_print = time.time()
            if divergence > self.max_divergence:
                self.max_divergence = divergence
            if divergence > self.max_allowed_divergence:
                msg = f"VFR_HUD.climb diverged from SIM_STATE.vd by {divergence}m/s (max={self.max_allowed_divergence}m/s"
                if self.instafail:
                    raise NotAchievedException(msg)
                else:
                    self.failed = True
                    self.progress(msg)

        def hook_removed(self):
            if self.vfr_hud is None:
                raise ValueError("Did not receive VFR_HUD")
            if self.sim_state is None:
                raise ValueError("Did not receive SIM_STATE")
            msg = f"Maximum divergence was {self.max_divergence}m/s (max={self.max_allowed_divergence}m/s)"
            if self.failed:
                raise NotAchievedException(msg)

            self.progress(msg)

    def tests(self):
        '''return list of all tests'''
        ret = []
        ret.extend(self.tests1a())
        ret.extend(self.tests1b())
        ret.extend(self.tests1c())
        return ret

    def tests1a(self):
        '''return list of all tests'''
        ret = super(AutoTestPlane, self).tests()
        ret.extend([
            self.FRSkyPassThroughSensorIDs,
            self.FRSkyD,
            self.DEVO,
            self.AHRS_ORIENTATION,
            self.MAVFTPBurstEOFOffset,
            self.MAVFTPListDirectoryRoot,
            self.MAVFTPShortReplyPadding,
            self.MAVFTPReadFile,
            self.MAVFTPRename,
            self.MAVFTPGapReadMAVProxy,
            self.AdvancedFailsafeBadBaro,
        ])
        ret.extend(self.discovered_tests("PlaneTests1a"))
        return ret

    def tests1b(self):
        '''return list of all tests'''
        ret = ([
            self.Button,
            self.FRSkySPort,
            self.FRSkyPassThroughStatustext,
            self.MAVFTP,
            self.MAVFTPListDirectoryFullPacket,
            self.MAVFTPMavLogDirectory,
            self.MAVFTPListDirectoryWithTime,
            self.MAVFTPListDirectoryWithTimeTabInName,
            self.MAVFTPListDirectoryWithTimeMAVProxy,
            self.MAVFTPListDirectoryWithTimeMAVProxyTabInName,
            self.MAVFTPListDirectoryUnknownTimeMAVProxy,
            self.MAVFTPListDirectoryFallbackMAVProxy,
            self.MAVFTPListDirectoryLossyRetry,
            self.MAVFTPDuplicateRequest,
            self.MAVFTPUnknownOpcodeNack,
            self.MAVFTPFileCommandsMAVProxy,
            self.MAVFTPListDirectoryInterleavedPut,
            self.MAVFTPListDirectoryInterleavedGet,
            self.MAVFTPListDirectoryTabInNameMAVProxy,
        ])
        ret.extend(self.discovered_tests("PlaneTests1b"))
        return ret

    def tests1c(self):
        '''return list of all tests'''
        ret = ([
            self.FRSkyMAVlite,
            self.LTM,
            self.AdvancedFailsafe,
            self.AHRSTrim,
            self.MAVFTPBurstMissionDat,
            self.MAVFTPParamPck,
            self.MAVFTPListDirectoryEdgeCases,
            self.MAVFTPListDirectoryLongNames,
            self.MAVFTPCalcFileCRC32,
            self.MAVFTPCrcCompareMAVProxy,
            self.MAVFTPVirtualWriteBounds,
            self.MAVFTPParamUploadBounds,
            self.MAVFTPListROMFS,
            self.MAVFTPListROMFSLongNames,
            self.MAVFTPListROMFSMissingDirectory,
            self.MAVFTPListROMFSFile,
            self.MSP_DJI,
            self.HIGH_LATENCY2,
            self.NoArmWithoutMissionItems,
        ])
        ret.extend(self.discovered_tests("PlaneTests1c"))
        return ret

    ##########################################################################
    # Tests using last_letter as a simulation backend
    ##########################################################################

    def testsLastLetter(self):
        '''tests flown against the last_letter external simulator

        Deliberately not part of tests(): these need last_letter_ardupilot
        installed, which only .github/workflows/test_sitl_last_letter.yml does,
        so they are reached through AutoTestLastLetterPlane below and run as
        test.LastLetterPlane rather than as part of test.Plane.
        '''
        ret = []
        ret.extend(self.discovered_tests("LastLetterPlane"))
        return ret

    ##########################################################################

    def disabled_tests(self):
        ret = {}
        if not self.mavproxy_ftp_module_has_command("crccmp"):
            # added to MAVProxy in 328d7de20 (2026-07-27) and not in any
            # release up to v1.8.74; skipped only where it is missing, so
            # CI - which installs MAVProxy from git master - still runs it
            ret["MAVFTPCrcCompareMAVProxy"] = "needs a MAVProxy which has the ftp crclocal and crccmp commands; see https://github.com/ArduPilot/MAVProxy"  # noqa:E501
        return ret


class AutoTestPlaneTests1a(AutoTestPlane):
    def tests(self):
        return self.tests1a()


class AutoTestPlaneTests1b(AutoTestPlane):
    def tests(self):
        return self.tests1b()


class AutoTestPlaneTests1c(AutoTestPlane):
    def tests(self):
        return self.tests1c()


class AutoTestLastLetterPlane(AutoTestPlane):
    def tests(self):
        return self.testsLastLetter()
