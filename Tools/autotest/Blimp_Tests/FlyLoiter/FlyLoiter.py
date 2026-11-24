'''
Blimp autotest: FlyLoiter

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class FlyLoiter(TestGroup):
    @testcase(bucket="Blimp")
    def FlyLoiter(self):
        '''test loiter mode'''

        self.change_mode('LOITER')
        self.wait_ready_to_arm()
        self.arm_vehicle()

        siz = 5
        tim = 60

        # make sure we don't drift:
        bl = self.get_location()
        tl = self.offset_location_ne(location=bl, metres_north=siz, metres_east=0)
        tr = self.offset_location_ne(location=bl, metres_north=siz, metres_east=siz)
        br = self.offset_location_ne(location=bl, metres_north=0, metres_east=siz)

        if self.mavproxy is not None:
            self.mavproxy.send(f"map icon {bl.lat} {bl.lng} flag\n")
            self.mavproxy.send(f"map icon {tl.lat} {tl.lng} flag\n")
            self.mavproxy.send(f"map icon {tr.lat} {tr.lng} flag\n")
            self.mavproxy.send(f"map icon {br.lat} {br.lng} flag\n")

        self.set_parameter("SIMPLE_MODE", 1)

        self.progress("Flying to top left corner.")
        self.set_rc(2, 2000)
        self.wait_distance_to_location(tl, 0, 0.2, timeout=tim)
        self.set_rc(2, 1500)

        self.progress("Flying to top right corner.")
        self.set_rc(1, 2000)
        self.wait_distance_to_location(tr, 0, 0.5, timeout=tim)
        self.set_rc(1, 1500)

        self.progress("Flying to bottom right corner.")
        self.set_rc(2, 1000)
        self.wait_distance_to_location(br, 0, 0.5, timeout=tim)
        self.set_rc(2, 1500)

        self.progress("Flying to bottom left corner.")
        self.set_rc(1, 1000)
        self.wait_distance_to_location(bl, 0, 0.5, timeout=tim)
        self.set_rc(1, 1500)

        fin = self.get_location()

        self.progress("Yawing right.")
        self.set_rc(4, 1700)
        self.wait_heading(135, accuracy=2, timeout=tim)
        self.set_rc(4, 1500)
        self.wait_distance_to_location(fin, 0, 0.15, timeout=5) # make sure we haven't moved from the spot

        self.progress("Going up and back down.")
        self.set_rc(3, 2000)
        self.wait_altitude(5, 5.5, relative=True, timeout=60)
        self.set_rc(3, 1000)
        self.wait_altitude(0, 0.5, relative=True, timeout=60)
        self.set_rc(3, 1500)
        self.wait_distance_to_location(fin, 0, 0.15, timeout=5) # make sure we haven't moved from the spot

        self.progress("Yawing left.")
        self.set_rc(4, 1300)
        self.wait_heading(0, accuracy=2, timeout=tim)
        self.set_rc(4, 1500)
        self.wait_distance_to_location(fin, 0, 0.15, timeout=5) # make sure we haven't moved from the spot

        self.disarm_vehicle()
