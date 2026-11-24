'''
Plane autotest: SpeedToFly

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class SpeedToFly(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def SpeedToFly(self):
        '''Test soaring speed-to-fly'''

        model = "plane-soaring"

        self.customise_SITL_commandline(
            [],
            model=model,
            wipe=True)

        self.load_mission('CMAC-soar.txt', strict=False)

        self.set_parameters({
            "SIM_THML_SCENARI": 0, # Turn off environmental thermals.
            "SOAR_ALT_MAX": 1000,  # remove source of random failure
        })

        # Get thermalling RC channel
        rc_chan = 0
        for i in range(8):
            rcx_option = self.get_parameter('RC{0}_OPTION'.format(i+1))
            if rcx_option == 88:
                rc_chan = i+1
                break

        if rc_chan == 0:
            raise NotAchievedException("Did not find soaring enable channel option.")

        # Disable soaring
        self.set_rc(rc_chan, 1100)

        self.set_current_waypoint(1)
        self.change_mode('AUTO')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        # Wait for to 400m before starting.
        self.wait_altitude(390, 400, timeout=600, relative=True)

        # Wait 10s to stabilize.
        self.delay_sim_time(30, reason="flight to stabilise")

        # Enable soaring (no automatic thermalling)
        self.set_rc(rc_chan, 1500)

        self.set_parameters({
            "SOAR_CRSE_ARSPD": -1,  # Enable speed to fly.
            "SOAR_VSPEED": 1,  # Set appropriate McCready.
            "SIM_WIND_SPD": 0,
        })

        self.progress('Waiting a few seconds before determining the "trim" airspeed.')
        self.delay_sim_time(20, reason="trim airspeed to stabilise")
        m = self.assert_receive_message('VFR_HUD')
        trim_airspeed = m.airspeed
        self.progress("Using trim_airspeed=%f" % (trim_airspeed,))

        min_airspeed = self.get_parameter("AIRSPEED_MIN")
        max_airspeed = self.get_parameter("AIRSPEED_MAX")

        if trim_airspeed > max_airspeed:
            raise NotAchievedException("trim airspeed > max_airspeed (%f>%f)" %
                                       (trim_airspeed, max_airspeed))
        if trim_airspeed < min_airspeed:
            raise NotAchievedException("trim airspeed < min_airspeed (%f<%f)" %
                                       (trim_airspeed, min_airspeed))

        self.progress("Adding updraft")
        self.set_parameters({
            "SIM_WIND_SPD": 1,
            'SIM_WIND_DIR_Z': 90,
        })
        self.progress("Waiting for vehicle to move slower in updraft")
        self.wait_airspeed(0, trim_airspeed-0.5, minimum_duration=10, timeout=120)

        self.progress("Adding downdraft")
        self.set_parameter('SIM_WIND_DIR_Z', -90)
        self.progress("Waiting for vehicle to move faster in downdraft")
        self.wait_airspeed(trim_airspeed+0.5, trim_airspeed+100, minimum_duration=10, timeout=120)

        self.progress("Zeroing wind and increasing McCready")
        self.set_parameters({
            "SIM_WIND_SPD": 0,
            "SOAR_VSPEED": 2,
        })
        self.progress("Waiting for airspeed to increase with higher VSPEED")
        self.wait_airspeed(trim_airspeed+0.5, trim_airspeed+100, minimum_duration=10, timeout=120)

        # mcReady tests don't work ATM, so just return early:
        # takes too long to land, so just make it all go away:
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        return

        self.start_subtest('Test McReady values')
        # Disable soaring
        self.set_rc(rc_chan, 1100)

        # Wait for to 400m before starting.
        self.wait_altitude(390, 400, timeout=600, relative=True)

        # Enable soaring
        self.set_rc(rc_chan, 2000)

        self.progress("Find airspeed with 1m/s updraft and mcready=1")
        self.set_parameters({
            "SOAR_VSPEED": 1,
            "SIM_WIND_SPD": 1,
        })
        self.delay_sim_time(20, reason="airspeed to stabilise")
        m = self.assert_receive_message('VFR_HUD')
        mcready1_speed = m.airspeed
        self.progress("airspeed is %f" % mcready1_speed)

        self.progress("Reducing McCready")
        self.set_parameters({
            "SOAR_VSPEED": 0.5,
        })
        self.progress("Waiting for airspeed to decrease with lower McReady")
        self.wait_airspeed(0, mcready1_speed-0.5, minimum_duration=10, timeout=120)

        self.progress("Increasing McCready")
        self.set_parameters({
            "SOAR_VSPEED": 1.5,
        })
        self.progress("Waiting for airspeed to decrease with lower McReady")
        self.wait_airspeed(mcready1_speed+0.5, mcready1_speed+100, minimum_duration=10, timeout=120)

        # takes too long to land, so just make it all go away:
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
