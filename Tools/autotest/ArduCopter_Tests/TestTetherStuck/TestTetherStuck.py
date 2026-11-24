'''
Copter autotest: TestTetherStuck

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class TestTetherStuck(TestGroup):
    @testcase(bucket="CopterTests1a")
    def TestTetherStuck(self):
        """Test tethered vehicle stuck because of tether"""
        # Enable tether simulation
        self.set_parameters({
            "SIM_TETH_ENABLE": 1,
        })
        self.delay_sim_time(2, reason="tether parameter to take effect")
        self.reboot_sitl()

        # Set tether line length
        self.set_parameters({
            "SIM_TETH_LINELEN": 10,
        })
        self.delay_sim_time(2, reason="tether line length parameter to take effect")

        # Prepare and take off
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.takeoff(10, mode='LOITER')

        # Simulate vehicle getting stuck by increasing RC throttle
        self.set_rc(3, 1900)
        self.delay_sim_time(10, reason='let tether get stuck')

        # Monitor behavior for 10 seconds
        tstart = self.get_sim_time()
        initial_alt = self.get_altitude(altitude_source='SIM_STATE.alt')
        self.progress(f"initial_alt={initial_alt}")
        stuck = True  # Assume it's stuck unless proven otherwise

        while self.get_sim_time() - tstart < 10:
            # Fetch current altitude
            current_alt = self.get_altitude(altitude_source='SIM_STATE.alt')
            self.progress(f"current_alt={current_alt}")

            # Fetch and log battery status
            battery_status = self.assert_receive_message('BATTERY_STATUS')
            if battery_status:
                self.progress(f"Battery: {battery_status}")

            # Check if the vehicle is stuck.
            # We assume the vehicle is stuck if the current is high and the altitude is not changing
            if battery_status and (battery_status.current_battery < 6500 or abs(current_alt - initial_alt) > 3):
                stuck = False  # Vehicle moved or current is abnormal
                break

        if not stuck:
            raise NotAchievedException("Vehicle did not get stuck as expected")

        # Land and disarm the vehicle to ensure we can go down
        self.land_and_disarm()
