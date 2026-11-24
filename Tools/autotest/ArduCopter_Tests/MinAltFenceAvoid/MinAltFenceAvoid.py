'''
Copter autotest: MinAltFenceAvoid

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MinAltFenceAvoid(TestGroup):
    # MinAltFenceAvoid - fly down and make sure fence action does not trigger
    # Also check that the vehicle will not try and ascend too fast when trying to backup from a min alt fence due to avoidance
    @testcase(bucket="CopterTests2b")
    def MinAltFenceAvoid(self):
        '''Test Min Alt Fence Avoidance'''

        # enable fence, only min altitude
        # No action, rely on avoidance to prevent the breach
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": 8,
            "FENCE_ALT_MIN": 20,
            "FENCE_ACTION": 0,
        })
        self.reboot_sitl()

        self.takeoff(30, mode="LOITER")
        """Hold loiter position."""

        # Try and fly past the fence
        self.set_rc(3, 1120)

        # Avoid should prevent the vehicle flying past the fence, so the altitude wait should timeouts
        try:
            self.wait_altitude(10, 15, timeout=90, relative=True)
            raise NotAchievedException("Avoid should prevent reaching altitude")
        except AutoTestTimeoutException:
            pass
        except Exception as e:
            raise e

        # Check ascent is not too fast, allow 10% above the configured backup speed
        max_ascent_rate = self.get_parameter("AVOID_BACKUP_SPD") * 1.1

        def get_climb_rate(mav, m):
            m_type = m.get_type()
            if m_type != 'VFR_HUD':
                return
            if m.climb > max_ascent_rate:
                raise NotAchievedException("Ascending too fast want %f got %f" % (max_ascent_rate, m.climb))

        self.context_push()
        self.install_message_hook_context(get_climb_rate)

        # Reduce fence alt, this will result in a fence breach, but there is no action.
        # Avoid should then backup the vehicle to be over the new fence alt.
        self.set_parameters({
            "FENCE_ALT_MIN": 30,
        })
        self.wait_altitude(30, 40, timeout=90, relative=True)

        self.context_pop()

        self.set_rc(3, 1500)
        self.do_RTL()
