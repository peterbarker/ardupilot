'''
Copter autotest: OSDParamConfig

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class OSDParamConfig(TestGroup):
    @testcase(bucket="CopterTests2b")
    def OSDParamConfig(self):
        '''test OSD parameter screen configuration over MAVLink and stick menu'''
        self.set_parameters({
            "SERIAL5_PROTOCOL": 42,  # MSP DisplayPort
            "OSD_TYPE": 5,           # MSP DisplayPort
            "OSD1_ENABLE": 0,
            "OSD5_ENABLE": 1,
            "OSD_CHAN": 12,
            "OSD_SW_METHOD": 1,      # select screen by PWM range
        })
        port = self.spare_network_port()
        self.customise_SITL_commandline([
            "--serial5=tcp:%u" % port  # serial5 listens on localhost port
        ])
        self.set_rc(12, 1500)
        # arming is blocked while the parameter screen is displayed
        self.wait_statustext("PreArm: In OSD menu", timeout=60)

        screen = 5  # first parameter screen
        request_id = 0

        def show_config(index):
            nonlocal request_id
            request_id += 1
            self.mav.mav.osd_param_show_config_send(1, 1, request_id, screen, index)
            return self.assert_receive_message(
                'OSD_PARAM_SHOW_CONFIG_REPLY',
                condition='OSD_PARAM_SHOW_CONFIG_REPLY.request_id==%u' % request_id,
                timeout=5,
                verbose=True,
            )

        def show_config_name(index):
            m = show_config(index)
            if m.result != mavutil.mavlink.OSD_PARAM_SUCCESS:
                raise NotAchievedException("Bad OSD_PARAM_SHOW_CONFIG result %u" % m.result)
            return m.param_id

        self.start_subtest("Read the default parameter slot")
        show_config_name(1)

        self.start_subtest("Configure a slot to the first parameter")
        request_id += 1
        self.mav.mav.osd_param_config_send(1, 1, request_id, screen, 1, "FORMAT_VERSION".encode('ascii'), 0, 0, 0, 0)
        m = self.assert_receive_message(
            'OSD_PARAM_CONFIG_REPLY',
            condition='OSD_PARAM_CONFIG_REPLY.request_id==%u' % request_id,
            timeout=5,
        )
        if m.result != mavutil.mavlink.OSD_PARAM_SUCCESS:
            raise NotAchievedException("Bad OSD_PARAM_CONFIG result %u" % m.result)
        name = show_config_name(1)
        if name != "FORMAT_VERSION":
            raise NotAchievedException("Expected FORMAT_VERSION got %s" % name)

        self.start_subtest("Enter parameter selection on the OSD menu")
        # menu gestures need throttle low and yaw centred
        self.set_rc_from_map({1: 1500, 2: 1500, 3: 1000, 4: 1500})
        # hold MENU_ENTER to go from value modify to parameter modify
        self.set_rc(1, 2000)
        self.delay_sim_time(5, reason="MENU_ENTER hold")
        self.set_rc(1, 1500)
        self.delay_sim_time(1, reason="stick re-centre")

        self.start_subtest("Holding MENU_UP from the first parameter")
        # MENU_UP from either of the first two parameters selects the second
        # one, and both paths start from the initial previous-token values
        self.set_rc(2, 1000)
        self.delay_sim_time(5, reason="MENU_UP hold")
        self.set_rc(2, 1500)
        self.delay_sim_time(1, reason="stick re-centre")
        name = show_config_name(1)
        if name != "PILOT_THR_FILT":
            raise NotAchievedException("Expected PILOT_THR_FILT got %s" % name)

        self.reboot_sitl()
