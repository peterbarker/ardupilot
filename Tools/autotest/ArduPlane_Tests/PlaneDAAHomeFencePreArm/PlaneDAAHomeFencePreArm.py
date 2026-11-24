'''
Plane autotest: PlaneDAAHomeFencePreArm

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAAHomeFencePreArm(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAAHomeFencePreArm(self):
        """Home inside the DAA fence buffer must fail auxiliary arming checks."""
        self.install_planedaa_scripts()
        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "FENCE_ENABLE": 0,
            "FENCE_TYPE": 4,
            "FENCE_ACTION": 0,
            "RC7_OPTION": 308,
        })
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.wait_ready_to_arm()
        home = self.home_position_as_location()
        self.upload_fences_from_locations([(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            {"radius": 60, "loc": self.offset_location_ne(home, 100, 0)},
        )])
        self.wait_text("Plane DAA", check_context=True, timeout=60)
        self.set_rc(7, 1000)
        failure = "pDAA: Fence too close to home for DAA"

        def refresh_check():
            self.delay_sim_time(6, reason="allow DAA parameters and pre-arm check to refresh")

        def expect_arm_success():
            refresh_check()
            self.arm_vehicle()
            self.disarm_vehicle()

        # Stored but disabled fences must not block arming. Once enabled, home
        # is physically clear by 40 m, but inside the default turn-radius buffer.
        expect_arm_success()
        self.set_parameter("FENCE_ENABLE", 1)
        refresh_check()
        self.assert_arm_failure(failure)

        # Clearance changes must release and re-apply authorization without
        # restarting the script. Successful arms prove this is not just text.
        self.set_parameter("DAA_MARGIN_FENCE", 20)
        expect_arm_success()
        self.set_parameter("DAA_MARGIN_FENCE", 60)
        refresh_check()
        self.assert_arm_failure(failure)

        self.set_rc(7, 2000)
        expect_arm_success()
        self.set_rc(7, 1000)
        refresh_check()
        self.assert_arm_failure(failure)

        self.set_parameter("DAA_AVD_ACTION", 0)
        expect_arm_success()
        self.set_parameter("DAA_AVD_ACTION", 1)
        refresh_check()
        self.assert_arm_failure(failure)

        # Uploading a fence farther from home must also clear the failure.
        self.upload_fences_from_locations([(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
            {"radius": 60, "loc": self.offset_location_ne(home, 200, 0)},
        )])
        expect_arm_success()

        # Match the flight-log case with a polygon boundary close to home.
        self.upload_fences_from_locations([(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, [
                self.offset_location_ne(home, 40, -100),
                self.offset_location_ne(home, 40, 100),
                self.offset_location_ne(home, 100, 100),
                self.offset_location_ne(home, 100, -100),
            ],
        )])
        refresh_check()
        self.assert_arm_failure(failure)
        self.set_parameter("FENCE_ENABLE", 0)
        expect_arm_success()

        self.set_parameter("FENCE_ENABLE", 1)

        # Home is inside an inclusion circle, but only 40 m from its edge.
        # A smaller DAA margin must permit arming; the normal margin must not.
        self.upload_fences_from_locations([(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
            {"radius": 100, "loc": self.offset_location_ne(home, 60, 0)},
        )])
        refresh_check()
        self.assert_arm_failure(failure)
        self.set_parameter("DAA_MARGIN_FENCE", 20)
        expect_arm_success()
        self.set_parameter("DAA_MARGIN_FENCE", 60)

        # Keep the vehicle inside a generous inclusion polygon, then move only
        # home outside it. This isolates DAA's home check from the core check
        # for a vehicle that is itself breaching the fence.
        self.upload_fences_from_locations([(
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, [
                self.offset_location_ne(home, -200, -200),
                self.offset_location_ne(home, -200, 200),
                self.offset_location_ne(home, 200, 200),
                self.offset_location_ne(home, 200, -200),
            ],
        )])
        expect_arm_success()
        self.set_home(self.offset_location_ne(home, 240, 0))
        refresh_check()
        self.assert_arm_failure(failure)
        self.set_home(home)
        expect_arm_success()
