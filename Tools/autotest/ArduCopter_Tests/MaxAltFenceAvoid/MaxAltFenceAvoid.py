'''
Copter autotest: MaxAltFenceAvoid

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AutoTestTimeoutException
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class MaxAltFenceAvoid(TestGroup):
    # MaxAltFence - fly up and make sure fence action does not trigger
    # Also check that the vehicle will not try and descend too fast when trying to backup from a max alt fence due to avoidance
    @testcase(bucket="CopterTests1d")
    def MaxAltFenceAvoid(self):
        '''Test Max Alt Fence Avoidance'''
        self.takeoff(10, mode="LOITER")
        """Hold loiter position."""

        # enable fence, only max altitude, default is 100m
        # No action, rely on avoidance to prevent the breach
        self.set_parameters({
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": 1,
            "FENCE_ACTION": 0,
        })

        # Try and fly past the fence
        self.set_rc(3, 1920)

        # Avoid should prevent the vehicle flying past the fence, so the altitude wait should timeouts
        try:
            self.wait_altitude(140, 150, timeout=90, relative=True)
            raise NotAchievedException("Avoid should prevent reaching altitude")
        except AutoTestTimeoutException:
            pass
        except Exception as e:
            raise e

        # Check descent is not too fast, allow 10% above the configured backup speed
        max_descent_rate = -self.get_parameter("AVOID_BACKUP_SPD") * 1.1

        def get_climb_rate(mav, m):
            m_type = m.get_type()
            if m_type != 'VFR_HUD':
                return
            if m.climb < max_descent_rate:
                raise NotAchievedException("Descending too fast want %f got %f" % (max_descent_rate, m.climb))

        self.context_push()
        self.install_message_hook_context(get_climb_rate)

        # Reduce fence alt, this will result in a fence breach, but there is no action.
        # Avoid should then backup the vehicle to be under the new fence alt.
        self.set_parameters({
            "FENCE_ALT_MAX": 50,
        })
        self.wait_altitude(40, 50, timeout=90, relative=True)

        self.context_pop()

        self.set_rc(3, 1500)
        self.do_RTL()
