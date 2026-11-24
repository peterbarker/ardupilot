'''
QuadPlane autotest: LoiterAltQLand

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class LoiterAltQLand(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def LoiterAltQLand(self):
        '''test loitering and qland with terrain involved'''
        self.LoiterAltQLand_Terrain(
            home="LakeGeorgeLookout",
            ofs_n=0,
            ofs_e=300,
        )
#        self.LoiterAltQLand_Terrain(
#            home="KalaupapaCliffs",
#            ofs_n=500,
#            ofs_e=500,
#        )
        self.LoiterAltQLand_Relative()

    def LoiterAltQLand_Relative(self):
        '''test failsafe where vehicle loiters in fixed-wing mode to a
        specific altitude then changes mode to QLAND'''
        self.set_parameters({
            'BATT_MONITOR': 4,  # LoiterAltQLand
            'BATT_FS_LOW_ACT': 6,  # LoiterAltQLand
        })
        self.reboot_sitl()
        takeoff_alt = 5
        self.takeoff(takeoff_alt, mode='QLOITER')
        loc = self.get_location()
        self.location_offset_ne(loc, 500, 500)
        new_alt = 100
        initial_altitude = self.get_altitude(relative=False, timeout=2)
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,
            1,  # reposition flags; 1 means "change to guided"
            0,
            0,
            int(loc.lat * 1e7),
            int(loc.lng * 1e7),
            new_alt,    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        )
        self.wait_altitude(
            new_alt-1,
            new_alt+1,
            timeout=60,
            relative=True,
            minimum_duration=10)
        self.wait_location(loc, timeout=120, accuracy=100, height_accuracy=None)
        self.progress("Triggering failsafe")
        self.set_parameter('BATT_LOW_VOLT', 50)
        self.wait_mode("LOITERALTQLAND")
        self.drain_mav()
        m = self.assert_receive_message('POSITION_TARGET_GLOBAL_INT', very_verbose=True)
        q_rtl_alt = self.get_parameter('Q_RTL_ALT')
        expected_alt = initial_altitude - takeoff_alt + q_rtl_alt

        if abs(m.alt - expected_alt) > 20:
            raise NotAchievedException("Unexpected altitude; expected=%f got=%f" %
                                       (expected_alt, m.alt))
        self.assert_mode('LOITERALTQLAND')
        self.wait_mode('QLAND')
        alt = self.get_altitude(relative=True)
        if abs(alt - q_rtl_alt) > 2:
            raise NotAchievedException("qland too late; want=%f got=%f" %
                                       (alt, q_rtl_alt))

        self.wait_disarmed(timeout=300)

    def LoiterAltQLand_Terrain(self,
                               home=None,
                               ofs_n=None,
                               ofs_e=None,
                               reposition_alt=100):
        '''test failsafe where vehicle loiters in fixed-wing mode to a
        specific altitude then changes mode to QLAND'''
        self.context_push()
        self.install_terrain_handlers_context()
        self.set_parameters({
            'BATT_MONITOR': 4,  # LoiterAltQLand
            'BATT_FS_LOW_ACT': 6,  # LoiterAltQLand
            'TERRAIN_FOLLOW': 1,  # enabled in all modes
        })
        self.customise_SITL_commandline(
            ["--home", home]
        )
        takeoff_alt = 5
        self.takeoff(takeoff_alt, mode='QLOITER')
        loc = self.get_location()
        self.location_offset_ne(loc, ofs_n, ofs_e)
        initial_altitude = self.get_altitude(relative=False, timeout=2)
        self.run_cmd_int(
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0,
            1,  # reposition flags; 1 means "change to guided"
            0,
            0,
            int(loc.lat * 1e7),
            int(loc.lng * 1e7),
            reposition_alt,    # alt
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        )
        self.wait_altitude(
            reposition_alt-1,
            reposition_alt+1,
            timeout=60,
            relative=True,
            minimum_duration=10)

        self.wait_location(loc, timeout=500, accuracy=100, height_accuracy=None)

        self.progress("Triggering failsafe")
        self.set_parameter('BATT_LOW_VOLT', 50)
        self.wait_mode("LOITERALTQLAND")
        terrain_alt = self.get_terrain_height(verbose=True)
        self.drain_mav()
        m = self.assert_receive_message('POSITION_TARGET_GLOBAL_INT', very_verbose=True)
        q_rtl_alt = self.get_parameter('Q_RTL_ALT')
        expected_alt = terrain_alt + q_rtl_alt

        if abs(m.alt - expected_alt) > 20:
            raise NotAchievedException("Unexpected altitude; expected=%f got=%f" %
                                       (expected_alt, m.alt))
        self.assert_mode('LOITERALTQLAND')
        self.wait_mode('QLAND')
        alt = initial_altitude + self.get_altitude(relative=True)
        if abs(alt - expected_alt) > 10:
            raise NotAchievedException("qland too late; want=%f got=%f" %
                                       (expected_alt, alt))

        self.wait_disarmed(timeout=300)
        self.zero_throttle()
        self.reset_SITL_commandline()
        self.context_pop()
