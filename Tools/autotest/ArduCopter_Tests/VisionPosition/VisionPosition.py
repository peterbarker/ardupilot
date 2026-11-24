'''
Copter autotest: VisionPosition

AP_FLAKE8_CLEAN
'''

import time

from vehicle_test_suite import AutoTestTimeoutException

from test_discovery import TestGroup
from test_discovery import testcase


class VisionPosition(TestGroup):
    @testcase(bucket="CopterTests2a")
    def VisionPosition(self):
        """Disable GPS navigation, enable Vicon input."""
        # scribble down a location we can set origin to:

        self.customise_SITL_commandline(["--serial5=sim:vicon:"])
        self.progress("Waiting for location")
        self.change_mode('LOITER')
        self.wait_ready_to_arm()

        old_pos = self.assert_receive_message('GLOBAL_POSITION_INT')
        print("old_pos=%s" % str(old_pos))

        # configure EKF to use external nav instead of GPS
        ahrs_ekf_type = self.get_parameter("AHRS_EKF_TYPE")
        if ahrs_ekf_type == 2:
            self.set_parameter("EK2_GPS_TYPE", 3)
        if ahrs_ekf_type == 3:
            self.set_parameters({
                "EK3_SRC1_POSXY": 6,
                "EK3_SRC1_VELXY": 6,
                "EK3_SRC1_POSZ": 6,
                "EK3_SRC1_VELZ": 6,
            })
        self.set_parameters({
            "GPS1_TYPE": 0,
            "VISO_TYPE": 1,
            "SERIAL5_PROTOCOL": 1,
        })
        self.reboot_sitl()
        # without a GPS or some sort of external prompting, AP
        # doesn't send system_time messages.  So prompt it:
        self.mav.mav.system_time_send(int(time.time() * 1000000), 0)
        self.set_origin(old_pos)

        self.takeoff()
        self.set_rc(1, 1600)
        tstart = self.get_sim_time()
        while True:
            vicon_pos = self.assert_receive_message('VISION_POSITION_ESTIMATE')
            # print("vpe=%s" % str(vicon_pos))
            # gpi = self.assert_receive_message('GLOBAL_POSITION_INT')
            # self.progress("gpi=%s" % str(gpi))
            if vicon_pos.x > 40:
                break

            if self.get_sim_time_cached() - tstart > 100:
                raise AutoTestTimeoutException("Vicon showed no movement")

        # recenter controls:
        self.set_rc(1, 1500)
        self.progress("# Enter RTL")
        self.change_mode('RTL')
        self.set_rc(3, 1500)
        tstart = self.get_sim_time()
        # self.install_messageprinter_handlers_context(['SIMSTATE', 'GLOBAL_POSITION_INT'])
        self.wait_disarmed(timeout=200)
