'''
QuadPlane autotest: AVAILABLE_MODES

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AVAILABLE_MODES(TestGroup):
    @testcase(bucket="QuadPlaneTests1c")
    def AVAILABLE_MODES(self):
        '''check AVAILABLE_MODES lists QuadPlane's modes'''
        expected_modes = {
            0: "Manual",
            1: "Circle",
            2: "Stabilize",
            3: "Training",
            4: "Acro",
            5: "FBWA",
            6: "FBWB",
            7: "Cruise",
            8: "Autotune",
            10: "Auto",
            11: "RTL",
            12: "Loiter",
            13: "Takeoff",
            14: "Avoid ADSB",
            15: "Guided",
            16: "Initialising",
            17: "QStabilize",
            18: "QHover",
            19: "QLoiter",
            20: "QLand",
            21: "QRTL",
            22: "QAutotune",
            23: "QAcro",
            24: "Thermal",
            25: "Loiter to QLand",
            26: "Autoland",
            27: "Glide",
        }
        initialising = self.get_mode_from_mode_mapping("INITIALISING")
        self.assert_available_modes(expected_modes, not_user_selectable=[initialising])

        self.start_subtest("VTOL modes blocked by FLTMODE_GCSBLOCK are not user-selectable")
        qhover = self.get_mode_from_mode_mapping("QHOVER")
        qloiter = self.get_mode_from_mode_mapping("QLOITER")
        self.set_parameter("FLTMODE_GCSBLOCK", (1 << 16) | (1 << 17))  # QHOVER and QLOITER
        modes = self.assert_available_modes(expected_modes, not_user_selectable=[initialising, qhover, qloiter])

        self.start_subtest("request a VTOL mode by index")
        index = len(modes)
        single = self.request_available_modes(index=index)
        if list(single.keys()) != [index] or single[index].custom_mode != modes[index].custom_mode:
            raise NotAchievedException(f"Did not get mode_index {index} ({modes[index].mode_name})")
