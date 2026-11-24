'''
Copter autotest: ModeZigZag

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ModeZigZag(TestGroup):
    @testcase(bucket="CopterTests1e")
    def ModeZigZag(self):
        '''test zigzag mode'''
        # set channel 8 for zigzag savewp and recentre it
        self.set_parameter("RC8_OPTION", 61)
        self.set_parameter("LOIT_OPTIONS", 0)

        self.takeoff(altitude_min=5, mode='LOITER')

        ZIGZAG = 24
        j = 0
        slowdown_speed = 0.3 # because Copter takes a long time to actually stop
        self.start_subtest("Conduct ZigZag test for all 4 directions")
        while j < 4:
            self.progress("## Align heading with the run-way (j=%d)##" % j)
            self.set_rc_from_map({8: 1500, 4: 1420})
            self.wait_heading(352-j*90)
            self.set_rc(4, 1500)
            self.change_mode(ZIGZAG)
            self.progress("## Record Point A ##")
            self.set_rc(8, 1100)  # record point A
            self.set_rc(1, 1700)  # fly side-way for 20m
            self.wait_distance(20)
            self.set_rc(1, 1500)
            self.wait_groundspeed(0, slowdown_speed)   # wait until the copter slows down
            self.progress("## Record Point A ##")
            self.set_rc(8, 1500)    # pilot always have to cross mid position when changing for low to high position
            self.set_rc(8, 1900)    # record point B

            i = 1
            while i < 2:
                self.start_subtest("Run zigzag A->B and B->A (i=%d)" % i)
                self.progress("## fly forward for 10 meter ##")
                self.set_rc(2, 1300)
                self.wait_distance(10)
                self.set_rc(2, 1500)    # re-centre pitch rc control
                self.wait_groundspeed(0, slowdown_speed)   # wait until the copter slows down
                self.set_rc(8, 1500)    # switch to mid position
                self.progress("## auto execute vector BA ##")
                self.set_rc(8, 1100)
                self.wait_distance(17)  # wait for it to finish
                self.wait_groundspeed(0, slowdown_speed)   # wait until the copter slows down

                self.progress("## fly forward for 10 meter ##")
                self.set_rc(2, 1300)    # fly forward for 10 meter
                self.wait_distance(10)
                self.set_rc(2, 1500)    # re-centre pitch rc control
                self.wait_groundspeed(0, slowdown_speed)   # wait until the copter slows down
                self.set_rc(8, 1500)    # switch to mid position
                self.progress("## auto execute vector AB ##")
                self.set_rc(8, 1900)
                self.wait_distance(17)  # wait for it to finish
                self.wait_groundspeed(0, slowdown_speed)   # wait until the copter slows down
                i = i + 1
            # test the case when pilot switch to manual control during the auto flight
            self.start_subtest("test the case when pilot switch to manual control during the auto flight")
            self.progress("## fly forward for 10 meter ##")
            self.set_rc(2, 1300)    # fly forward for 10 meter
            self.wait_distance(10)
            self.set_rc(2, 1500)    # re-centre pitch rc control
            self.wait_groundspeed(0, 0.3)   # wait until the copter slows down
            self.set_rc(8, 1500)    # switch to mid position
            self.progress("## auto execute vector BA ##")
            self.set_rc(8, 1100)    # switch to low position, auto execute vector BA
            self.wait_distance(8)   # purposely switch to manual halfway
            self.set_rc(8, 1500)
            self.wait_groundspeed(0, slowdown_speed)   # copter should slow down here
            self.progress("## Manual control to fly forward ##")
            self.set_rc(2, 1300)    # manual control to fly forward
            self.wait_distance(8)
            self.set_rc(2, 1500)    # re-centre pitch rc control
            self.wait_groundspeed(0, slowdown_speed)   # wait until the copter slows down
            self.progress("## continue vector BA ##")
            self.set_rc(8, 1100)    # copter should continue mission here
            self.wait_distance(8)   # wait for it to finish rest of BA
            self.wait_groundspeed(0, slowdown_speed)   # wait until the copter slows down
            self.set_rc(8, 1500)    # switch to mid position
            self.progress("## auto execute vector AB ##")
            self.set_rc(8, 1900)    # switch to execute AB again
            self.wait_distance(17)  # wait for it to finish
            self.wait_groundspeed(0, slowdown_speed)   # wait until the copter slows down
            self.change_mode('LOITER')
            j = j + 1

        self.do_RTL()
