'''
Rover autotest: DriveEachFrame

AP_FLAKE8_CLEAN
'''

from pysim import vehicleinfo

from test_discovery import TestGroup
from test_discovery import testcase


class DriveEachFrame(TestGroup):
    @testcase(bucket="Rover")
    def DriveEachFrame(self):
        '''drive each frame (except BalanceBot) in a mission to ensure basic functionality'''
        vinfo = vehicleinfo.VehicleInfo()
        vinfo_options = vinfo.options[self.vehicleinfo_key()]
        known_broken_frames = {
            "balancebot": "needs special stay-upright code",
        }
        for frame in sorted(vinfo_options["frames"].keys()):
            self.start_subtest("Testing frame (%s)" % str(frame))
            if frame in known_broken_frames:
                self.progress("Actually, no I'm not - it is known-broken (%s)" %
                              (known_broken_frames[frame]))
                continue
            frame_bits = vinfo_options["frames"][frame]
            print("frame_bits: %s" % str(frame_bits))
            if frame_bits.get("external", False):
                self.progress("Actually, no I'm not - it is an external simulation")
                continue
            model = frame_bits.get("model", frame)
            self.customise_SITL_commandline(
                [],
                model=model,
                wipe=True,
            )
            mission_file = "basic.txt"
            self.wait_ready_to_arm()
            self.set_parameters({
                "MIS_DONE_BEHAVE": 3,
                "SIM_WIND_SPD": 10,
            })
            self.arm_vehicle()
            self.drive_mission(mission_file, strict=False, ignore_MANUAL_mode_change=True)
            self.wait_mode('MANUAL')

            self.wait_distance_to_home(0, 5, timeout=1)
