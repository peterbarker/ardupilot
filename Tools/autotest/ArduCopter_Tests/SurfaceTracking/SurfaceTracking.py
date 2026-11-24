'''
Copter autotest: SurfaceTracking

AP_FLAKE8_CLEAN
'''

from vehicle_test_suite import AltFrame
from vehicle_test_suite import Location
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class SurfaceTracking(TestGroup):
    @testcase(bucket="CopterTests1a")
    def SurfaceTracking(self):
        '''Test Surface Tracking'''
        ex = None
        self.context_push()

        self.install_terrain_handlers_context()

        try:
            self.set_analog_rangefinder_parameters()
            self.set_parameter("RC9_OPTION", 10) # rangefinder
            self.set_rc(9, 2000)

            self.reboot_sitl() # needed for both rangefinder and initial position
            self.assert_vehicle_location_is_at_startup_location()

            self.takeoff(10, mode="LOITER")
            lower_surface_pos = Location.latlon_only(-35.362421, 149.164534)
            here = self.get_location()
            bearing = self.get_bearing(here, lower_surface_pos)

            self.change_mode("GUIDED")
            self.guided_achieve_heading(bearing, direction=1, accuracy=1)
            self.change_mode("LOITER")
            self.delay_sim_time(2, reason="loiter to stabilise")
            m = self.assert_receive_message('GLOBAL_POSITION_INT')
            orig_absolute_alt_mm = m.alt

            self.progress("Original alt: absolute=%f" % orig_absolute_alt_mm)

            self.progress("Flying somewhere which surface is known lower compared to takeoff point")
            self.set_rc(2, 1450)
            tstart = self.get_sim_time()
            while True:
                if self.get_sim_time() - tstart > 200:
                    raise NotAchievedException("Did not reach lower point")
                x = self.get_location()
                dist = self.get_distance(x, lower_surface_pos)
                delta = orig_absolute_alt_mm/1000.0 - x.get_alt_m(AltFrame.ABSOLUTE)

                self.progress("Distance: %fm abs-alt-delta: %fm" %
                              (dist, delta))
                if dist < 15:
                    if delta < 0.8:
                        raise NotAchievedException("Did not dip in altitude as expected")
                    break

            self.set_rc(2, 1500)
            self.do_RTL()

        except Exception as e:  # noqa: BLE001
            self.print_exception_caught(e)
            self.disarm_vehicle(force=True)
            ex = e

        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex
