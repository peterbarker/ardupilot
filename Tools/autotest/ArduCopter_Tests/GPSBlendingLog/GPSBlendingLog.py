'''
Copter autotest: GPSBlendingLog

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class GPSBlendingLog(TestGroup):
    @testcase(bucket="CopterTests1c")
    def GPSBlendingLog(self):
        '''Test GPS Blending'''
        '''ensure we get dataflash log messages for blended instance'''
        # configure:
        self.set_parameters({
            "GPS2_TYPE": 1,
            "SIM_GPS2_TYPE": 1,
            "SIM_GPS2_ENABLE": 1,
            "GPS_AUTO_SWITCH": 2,
        })
        self.reboot_sitl()

        # ensure we're seeing the second GPS:
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 60:
                raise NotAchievedException("Did not get good GPS2_RAW message")
            m = self.assert_receive_message('GPS2_RAW', verbose=True)
            if m.lat == 0:
                continue
            break

        # create a log we can expect blended data to appear in:
        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.delay_sim_time(5, reason="GPS blend data to accumulate in log")
        self.disarm_vehicle()

        # inspect generated log for messages:
        dfreader = self.dfreader_for_current_onboard_log()
        wanted = set([0, 1, 2])
        seen_primary_change = False
        while True:
            m = dfreader.recv_match(type=["GPS", "EV"]) # disarmed
            if m is None:
                break
            mtype = m.get_type()
            if mtype == 'GPS':
                try:
                    wanted.remove(m.I)
                except KeyError:
                    continue
            elif mtype == 'EV':
                if m.Id == 67:  # GPS_PRIMARY_CHANGED
                    seen_primary_change = True
            if len(wanted) == 0 and seen_primary_change:
                break

        if len(wanted):
            raise NotAchievedException("Did not get all three GPS types")
        if not seen_primary_change:
            raise NotAchievedException("Did not see primary change")
