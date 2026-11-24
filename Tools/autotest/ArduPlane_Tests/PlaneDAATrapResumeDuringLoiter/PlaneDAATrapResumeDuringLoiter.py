'''
Plane autotest: PlaneDAATrapResumeDuringLoiter

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class PlaneDAATrapResumeDuringLoiter(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def PlaneDAATrapResumeDuringLoiter(self):
        '''A trapped-failsafe that fires and clears WHILE the aircraft loiter is active
        must not strand the mission in GUIDED.

        The aircraft loiter (daaltr.lua) treats any mode other than GUIDED as the pilot
        taking over, and drops its saved pre-loiter mode when that happens. It cached
        the mode once per cycle (get_vehicle_state(), read at the START of the cycle)
        rather than reading it live - so when the trap's own recovery (DAA_TRAP_CLR_S)
        restores GUIDED, the loiter's next update() in that same cycle still saw the
        stale pre-restore mode and misread the trap's OWN restore as a pilot mode
        change, dropping the state and leaving the AUTO mission stuck in GUIDED forever
        instead of eventually resuming.

        AVD_NMAC_XY/Z are widened so the same contact that engages the aircraft loiter
        also counts as a near-miss once it closes in further, firing the trap without a
        second contact. DAA_TRAP_S/CLR_S are both shortened for test speed; the contact
        is withdrawn before DAA_TRAP_CLR_S elapses so the trap resumes GUIDED into a
        clear sky and does not immediately re-trap.'''
        self._PlaneDAATrapResumeDuringLoiter()
