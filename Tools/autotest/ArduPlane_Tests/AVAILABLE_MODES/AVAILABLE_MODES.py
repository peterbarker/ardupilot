'''
Plane autotest: AVAILABLE_MODES

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class AVAILABLE_MODES(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def AVAILABLE_MODES(self):
        '''check AVAILABLE_MODES lists Plane's modes'''
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
            24: "Thermal",
            26: "Autoland",
            27: "Glide",
        }
        initialising = self.get_mode_from_mode_mapping("INITIALISING")
        modes = self.assert_available_modes(expected_modes, not_user_selectable=[initialising])
        for m in modes.values():
            if m.standard_mode != mavutil.mavlink.MAV_STANDARD_MODE_NON_STANDARD:
                raise NotAchievedException(f"{m.mode_name}: unexpected standard_mode {m.standard_mode}")

        self.start_subtest("modes blocked by FLTMODE_GCSBLOCK are not user-selectable")
        circle = self.get_mode_from_mode_mapping("CIRCLE")
        guided = self.get_mode_from_mode_mapping("GUIDED")
        glide = 27  # Glide, need pymavlink support to use text name
        self.set_parameter("FLTMODE_GCSBLOCK", (1 << 1) | (1 << 13) | (1 << 22))  # CIRCLE, GUIDED and Glide
        self.assert_available_modes(expected_modes, not_user_selectable=[initialising, circle, guided, glide])
        self.set_parameter("FLTMODE_GCSBLOCK", 0)

        self.start_subtest("request the first and last modes by index")
        for index in 1, len(modes):
            single = self.request_available_modes(index=index)
            if list(single.keys()) != [index]:
                raise NotAchievedException(f"Asked for mode_index {index} got {list(single.keys())}")
            if single[index].custom_mode != modes[index].custom_mode:
                raise NotAchievedException(f"mode_index {index} is not the mode sent when all were requested")

        self.start_subtest("request an index beyond the last mode")
        beyond = self.request_available_modes(index=len(modes)+1, timeout=3)
        if len(beyond) != 0:
            raise NotAchievedException("Received AVAILABLE_MODES for a mode which does not exist")
