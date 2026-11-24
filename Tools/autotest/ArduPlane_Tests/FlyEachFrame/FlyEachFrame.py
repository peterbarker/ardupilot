'''
Plane autotest: FlyEachFrame

AP_FLAKE8_CLEAN
'''

from pysim import vehicleinfo

from test_discovery import TestGroup
from test_discovery import testcase


class FlyEachFrame(TestGroup):
    @testcase(bucket="PlaneTests1b")
    def FlyEachFrame(self):
        '''Fly each supported internal frame'''
        vinfo = vehicleinfo.VehicleInfo()
        vinfo_options = vinfo.options[self.vehicleinfo_key()]
        known_broken_frames = {
            "plane-tailsitter": "unstable in hover; unflyable in cruise",
            "quadplane-can" : "needs CAN periph",
            "stratoblimp" : "not expected to fly normally",
            "glider" : "needs balloon lift",
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
            quadplane = self.get_parameter('Q_ENABLE')
            if quadplane:
                mission_file = "basic-quadplane.txt"
            tailsitter = self.get_parameter('Q_TAILSIT_ENABLE')
            if tailsitter:
                # tailsitter needs extra re-boot to pick up the rotated AHRS view
                self.reboot_sitl()
            self.wait_ready_to_arm()
            self.arm_vehicle()
            self.fly_mission(mission_file, strict=False, quadplane=quadplane, mission_timeout=400.0)
            self.wait_disarmed()
