'''
Copter autotest: GuidedEKFLaneChange

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class GuidedEKFLaneChange(TestGroup):
    @testcase(bucket="CopterTests2b")
    def GuidedEKFLaneChange(self):
        '''test lane change with GPS diff on startup'''
        self.set_parameters({
            "EK3_SRC1_POSZ": 3,
            "EK3_AFFINITY" : 1,
            "GPS2_TYPE" : 1,
            "SIM_GPS2_ENABLE" : 1,
            "SIM_GPS2_GLTCH_Z" : -30
            })
        self.reboot_sitl()

        self.change_mode("GUIDED")
        self.wait_ready_to_arm()

        self.delay_sim_time(10, reason='"both EKF lanes to init"')

        self.set_parameters({
            "SIM_GPS2_GLTCH_Z" : 0
            })

        self.delay_sim_time(20, reason="EKF to do a position Z reset")

        self.arm_vehicle()
        self.user_takeoff(alt_min=20)
        gps_alt = self.get_altitude(altitude_source='GPS_RAW_INT.alt')
        self.progress("Initial guided alt=%.1fm" % gps_alt)

        self.context_collect('STATUSTEXT')
        self.progress("force a lane change")
        self.set_parameters({
            "INS_ACCOFFS_X" : 5
            })
        self.wait_statustext("EKF3 lane switch 1", timeout=10, check_context=True)

        self.wait_altitude(
            altitude_min=gps_alt-2,
            altitude_max=gps_alt+2,
            relative=True,
            altitude_source='GPS_RAW_INT.alt',
            minimum_duration=10,
            timeout=11,
        )

        self.disarm_vehicle(force=True)
        self.reboot_sitl()
