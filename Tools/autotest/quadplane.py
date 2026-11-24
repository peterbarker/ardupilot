'''
Fly ArduPlane QuadPlane in SITL

AP_FLAKE8_CLEAN

'''

import math
import os

import numpy

from pymavlink import mavutil

import vehicle_test_suite

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException
from vehicle_test_suite import Test

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
WIND = "0,180,0.2"  # speed,direction,variance
SITL_START_LOCATION = Location(-27.274439, 151.290064, 343, AltFrame.ABSOLUTE)
SITL_START_HEADING = 8.7


class AutoTestQuadPlane(vehicle_test_suite.TestSuite):

    @staticmethod
    def get_not_armable_mode_list():
        return []

    @staticmethod
    def get_not_disarmed_settable_modes_list():
        return []

    @staticmethod
    def get_no_position_not_settable_modes_list():
        return []

    @staticmethod
    def get_position_armable_modes_list():
        return []

    @staticmethod
    def get_normal_armable_modes_list():
        return []

    def vehicleinfo_key(self):
        return 'ArduPlane'

    def default_frame(self):
        return "quadplane"

    def test_filepath(self):
        return os.path.realpath(__file__)

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def sitl_start_heading(self):
        return SITL_START_HEADING

    def log_name(self):
        return "QuadPlane"

    test_root = "QuadPlane_Tests"
    test_resource_fallback_roots = ("ArduPlane_Tests",)

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

    def fly_mission(self, filename, fence=None, height_accuracy=-1):
        """Fly a mission from a file."""
        self.progress("Flying mission %s" % filename)
        num_wp = self.load_mission(filename)
        if self.mavproxy is not None:
            self.mavproxy.send('wp list\n')
        if fence is not None:
            self.load_fence(fence)
        # self.install_terrain_handlers_context()
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.wait_waypoint(1, num_wp-1)
        self.wait_disarmed(timeout=120) # give quadplane a long time to land

    def hover(self, hover_throttle=1500):
        self.set_rc(3, hover_throttle)

    def takeoff(self, height, mode, timeout=30):
        """climb to specified height and set throttle to 1500"""
        self.set_current_waypoint(0, check_afterwards=False)
        self.change_mode(mode)
        self.wait_ready_to_arm()
        self.arm_vehicle()
        if mode == 'GUIDED':
            self.user_takeoff(alt_min=height, timeout=timeout)
            return
        self.set_rc(3, 1800)
        self.wait_altitude(height,
                           height+5,
                           relative=True,
                           timeout=timeout)
        self.set_rc(3, 1500)

    def do_RTL(self):
        self.change_mode("QRTL")
        self.wait_altitude(-5, 1, relative=True, timeout=60)
        self.wait_disarmed()
        self.zero_throttle()

    def fly_home_land_and_disarm(self, timeout=30):
        self.context_push()
        self.change_mode('LOITER')
        self.set_parameter('RTL_AUTOLAND', 2)
        filename = "QuadPlaneDalbyRTL.txt"
        self.progress("Using %s to fly home" % filename)
        self.load_generic_mission(filename)
        self.send_cmd_do_set_mode("RTL")
        self.wait_mode('AUTO')
        self.wait_current_waypoint(4)
        self.wait_statustext('Land descend started')
        self.wait_statustext('Land final started', timeout=60)
        self.wait_disarmed(timeout=timeout)
        self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        # the following command is accepted, but doesn't actually
        # work!  Should be able to remove check_afterwards!
        self.set_current_waypoint(0, check_afterwards=False)
        self.change_mode('MANUAL')
        self.context_pop()

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

    def fly_left_circuit(self):
        """Fly a left circuit, 200m on a side."""
        self.mavproxy.send('switch 4\n')
        self.change_mode('FBWA')
        self.set_rc(3, 1700)
        self.wait_level_flight()

        self.progress("Flying left circuit")
        # do 4 turns
        for i in range(0, 4):
            # hard left
            self.progress("Starting turn %u" % i)
            self.set_rc(1, 1000)
            self.wait_heading(270 - (90*i), accuracy=10)
            self.set_rc(1, 1500)
            self.progress("Starting leg %u" % i)
            self.wait_distance(100, accuracy=20)
        self.progress("Circuit complete")
        self.change_mode('QHOVER')
        self.set_rc(3, 1100)
        self.wait_altitude(10, 15,
                           relative=True,
                           timeout=60)
        self.set_rc(3, 1500)

    def hover_and_check_matched_frequency(self, dblevel=-15, minhz=200, maxhz=300, fftLength=32, peakhz=None):

        # find a motor peak
        self.takeoff(10, mode="QHOVER")

        hover_time = 15
        tstart = self.get_sim_time()
        self.progress("Hovering for %u seconds" % hover_time)
        while self.get_sim_time_cached() < tstart + hover_time:
            self.assert_receive_message('ATTITUDE')
        vfr_hud = self.assert_receive_message('VFR_HUD')
        tend = self.get_sim_time()

        self.do_RTL()
        psd = self.mavfft_fttd(1, 0, tstart * 1.0e6, tend * 1.0e6)

        # batch sampler defaults give 1024 fft and sample rate of 1kz so roughly 1hz/bin
        scale = 1000. / 1024.
        sminhz = int(minhz * scale)
        smaxhz = int(maxhz * scale)
        freq = psd["F"][numpy.argmax(psd["X"][sminhz:smaxhz]) + sminhz]
        peakdb = numpy.amax(psd["X"][sminhz:smaxhz])
        if peakdb < dblevel or (peakhz is not None and abs(freq - peakhz) / peakhz > 0.05):
            raise NotAchievedException("No motor peak, found %fHz at %fdB" % (freq, peakdb))
        else:
            self.progress("motor peak %fHz, thr %f%%, %fdB" % (freq, vfr_hud.throttle, peakdb))

        # we have a peak make sure that the FFT detected something close
        # logging is at 10Hz
        mlog = self.dfreader_for_current_onboard_log()
        # accuracy is determined by sample rate and fft length, given our use of quinn we could probably use half of this
        freqDelta = 1000. / fftLength
        pkAvg = freq
        freqs = []

        while True:
            m = mlog.recv_match(
                type='FTN1',
                blocking=True,
                condition="FTN1.TimeUS>%u and FTN1.TimeUS<%u" % (tstart * 1.0e6, tend * 1.0e6))
            if m is None:
                break
            freqs.append(m.PkAvg)

        # peak within resolution of FFT length
        pkAvg = numpy.median(numpy.asarray(freqs))
        if abs(pkAvg - freq) > freqDelta:
            raise NotAchievedException("FFT did not detect a motor peak at %f, found %f, wanted %f" % (dblevel, pkAvg, freq))

        return freq

    def PIDTuning(self):
        '''Test PID Tuning'''
        self.change_mode("FBWA") # we don't update PIDs in MANUAL
        super(AutoTestQuadPlane, self).PIDTuning()

    def rc_defaults(self):
        ret = super(AutoTestQuadPlane, self).rc_defaults()
        ret[3] = 1000
        return ret

    def default_mode(self):
        return "MANUAL"

    def disabled_tests(self):
        return {
            "FRSkyPassThrough": "Currently failing",
            "ConfigErrorLoop": "failing because RC values not settable",
        }

    def CPUFailsafe(self):
        '''In lockup Plane should copy RC inputs to RC outputs'''
        self.plane_CPUFailsafe()

    def setup_ICEngine_vehicle(self):
        '''restarts SITL with an IC Engine setup'''
        model = "quadplane-ice"
        self.customise_SITL_commandline(
            [],
            model=model,
            wipe=False,
        )

    # The gains quicktune saves when it finishes.  Written by the applet
    # or the C++ implementation rather than by us, so the suite cannot
    # revert them and they would persist for the rest of the session.
    # everything quicktune can save; the axes and suffixes here are
    # AP_Quicktune::Param (libraries/AP_Quicktune/AP_Quicktune.h)
    quicktune_saved_gains = [
        "Q_A_RAT_%s_%s" % (axis, suffix)
        for axis in ("RLL", "PIT", "YAW")
        for suffix in ("P", "I", "D", "SMAX", "FLTT", "FLTD", "FLTE", "FF")
    ]

    def FenceRelative_TakeoffMode(self):
        '''method for the FenceRelative test to call'''
        return 'QLOITER'

    def tests(self):
        '''return list of all tests'''
        ret = []
        ret.extend(self.tests1a())
        ret.extend(self.tests1b())
        ret.extend(self.tests1c())
        return ret

    def tests1a(self):
        '''return list of all tests'''
        ret = super(AutoTestQuadPlane, self).tests()
        ret.extend([
            self.TestLogDownload,
            Test(self.MotorTest, kwargs={  # tests motors 4 and 2
                "mot1_servo_chan": 8,  # quad-x second motor cw from f-r
                "mot4_servo_chan": 6,  # quad-x third motor cw from f-r
                "wait_finish_text": False,
                "quadplane": True,
            }),
            self.MAVFTPBadReadOffset,
            self.FenceRelativeToHomeMaxAlt,
            self.FenceRelativeToHomeCliff,
            self.FenceRelativeToOriginMinAlt,
            self.FenceRelativeToOriginMaxAltHomeAbove,
            self.FenceRelativeToOriginMinAltHomeAbove,
        ])
        ret.extend(self.discovered_tests("QuadPlaneTests1a"))
        return ret

    def tests1b(self):
        '''return list of all tests'''
        ret = ([
            self.FenceRelativePreArms,
            self.FenceRelativeToHomeMinAlt,
            self.FenceRelativeToOriginMaxAlt,
            self.FenceRelativeToAMSLMinAlt,
            self.FenceRelativeToTerrainMaxAlt,
            self.FenceRelativeToTerrainMinAlt,
        ])
        ret.extend(self.discovered_tests("QuadPlaneTests1b"))
        return ret

    def tests1c(self):
        '''return list of all tests'''
        ret = ([
            self.TestLogDownloadWrap,
            self.FenceRelativeToHomeMaxAltOriginAbove,
            self.FenceRelativeToHomeMinAltOriginAbove,
            self.FenceRelativeToAMSLMaxAlt,
            self.FenceRelativeToAMSLCliff,
        ])
        ret.extend(self.discovered_tests("QuadPlaneTests1c"))
        return ret


class AutoTestQuadPlaneTests1a(AutoTestQuadPlane):
    def tests(self):
        return self.tests1a()


class AutoTestQuadPlaneTests1b(AutoTestQuadPlane):
    def tests(self):
        return self.tests1b()


class AutoTestQuadPlaneTests1c(AutoTestQuadPlane):
    def tests(self):
        return self.tests1c()
