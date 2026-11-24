'''
QuadPlane autotest: ShipLanding

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class ShipLanding(TestGroup):
    @testcase(bucket="QuadPlaneTests1a")
    def ShipLanding(self):
        '''ship landing test'''
        self.install_applet_script_context("plane_ship_landing.lua")

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SIM_SHIP_ENABLE": 1,
            "SIM_SHIP_SPEED": 5,
            "Q_WP_SPD": 7.0,
            "Q_P_NE_POS_P": 0.25,
            "Q_P_NE_VEL_D": 0.25,
            "Q_P_NE_VEL_I": 0.25,
            "Q_P_NE_VEL_P": 1.0,
            "SIM_SHIP_DSIZE": 10,
            "FOLL_ENABLE": 1,
            "FOLL_SYSID": 17,
            "FOLL_OFS_TYPE": 1,
            "SIM_TERRAIN" : 0,
            "TERRAIN_ENABLE" : 0,
        })

        self.load_mission("takeoff100.txt")

        self.reboot_sitl(check_position=False)

        self.context_collect('STATUSTEXT')
        self.set_parameters({
            "SHIP_ENABLE" : 1,
            "SIM_SPEEDUP" : 10,
            })

        self.scripting_restart()
        self.wait_text("ShipLanding: loaded", check_context=True)

        self.wait_ready_to_arm()
        self.change_mode("AUTO")
        self.arm_vehicle()
        self.wait_altitude(95, 105, relative=True, timeout=90)
        self.drain_mav()

        self.wait_text("Mission complete, changing mode to RTL", check_context=True, timeout=60)
        self.wait_text("Descending for approach", check_context=True, timeout=60)
        self.wait_text("Reached target altitude", check_context=True, timeout=120)
        self.wait_text("Starting approach", check_context=True, timeout=120)
        self.wait_text("Land complete", check_context=True, timeout=120)

        self.wait_disarmed(timeout=180)

        # we confirm successful landing on the ship from our ground speed. The
        # deck is just 10m in size, so we must be within 10m if we are moving
        # with the deck
        self.wait_groundspeed(4.8, 5.2)

        tstart = self.get_sim_time_cached()
        ship_gpi = None
        vehicle_gpi = None
        while ship_gpi is None or vehicle_gpi is None:
            if self.get_sim_time_cached() - tstart > 5:
                raise NotAchievedException("Did not get GPI for ship")
            gpi = self.assert_receive_message('GLOBAL_POSITION_INT')
            if gpi.get_srcSystem() == 17:
                ship_gpi = gpi
            elif gpi.get_srcSystem() == 1:
                vehicle_gpi = gpi

        distance = self.get_distance_int(vehicle_gpi, ship_gpi)
        self.progress(f"{distance=}")
        max_distance = 1.2
        if distance > max_distance:
            raise NotAchievedException(f"Did not land within {max_distance}m of ship {distance=}")

        # we are not at the home location - reboot so the next test starts there
        self.set_parameter("SIM_SHIP_ENABLE", 0)
        self.reboot_sitl()
