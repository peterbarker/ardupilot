'''
Helicopter autotest: FlyEachFrame

AP_FLAKE8_CLEAN
'''

from pysim import vehicleinfo

from test_discovery import TestGroup
from test_discovery import testcase


class FlyEachFrame(TestGroup):
    @testcase(bucket="Helicopter")
    def FlyEachFrame(self):
        '''Fly each supported internal frame'''
        vinfo = vehicleinfo.VehicleInfo()
        vinfo_options = vinfo.options[self.vehicleinfo_key()]
        known_broken_frames = {
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
            self.takeoff(10)
            self.do_RTL()
