'''
Copter autotest: FenceTypeChangeTakesEffect

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class FenceTypeChangeTakesEffect(TestGroup):
    def assert_fence_breach_reported(self, fence_name, parameter, outside, inside, timeout=20):
        """move one fence limit past the hovering vehicle and assert that the
        breach is reported by name.  A breach is reported only by a fence the
        vehicle is actually enforcing, so this distinguishes the types which
        the aggregate SYS_STATUS geofence bit cannot: that bit is still set
        when one of several types has been lost.  The limit is restored
        before returning."""
        self.context_collect('STATUSTEXT')
        try:
            self.set_parameter(parameter, outside)
            self.wait_statustext("%s fence breached" % fence_name,
                                 check_context=True,
                                 timeout=timeout)
            # the breach has to be cleared before the next case, and the
            # vehicle says so as soon as the limit is restored - which is
            # inside the set_parameter below, so this has to be looking at a
            # collection which was already running rather than start waiting
            # afterwards
            self.set_parameter(parameter, inside)
            self.wait_statustext("Fence breach cleared",
                                 check_context=True,
                                 timeout=timeout)
        finally:
            self.set_parameter(parameter, inside)
            self.context_stop_collecting('STATUSTEXT')

    def assert_no_fence_breach_reported(self, parameter, outside, inside):
        """the mirror of assert_fence_breach_reported: move a limit past the
        hovering vehicle and assert that no fence reports a breach"""
        self.context_collect('STATUSTEXT')
        self.set_parameter(parameter, outside)
        self.delay_sim_time(5, reason="see whether any fence is enforced")
        breach = self.statustext_in_collections("fence breached")
        self.context_stop_collecting('STATUSTEXT')
        self.set_parameter(parameter, inside)
        if breach is not None:
            raise NotAchievedException("Fence enforced when it should not be (%s)" %
                                       breach.text)

    @testcase(bucket="CopterTests1e")
    def FenceTypeChangeTakesEffect(self):
        """FENCE_TYPE changes take effect without re-enabling the fence"""
        self.start_subtest("a type added to an enabled fence is enforced")
        # a fence which is enabled with no types selected enforces nothing
        self.set_parameters({
            "FENCE_AUTOENABLE": 0,
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": 0,
        })
        self.assert_sensor_state(
            mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE,
            present=False,
            enabled=False,
        )

        # select the max-altitude fence, leaving FENCE_ENABLE alone.  The
        # fence the vehicle enforces is a runtime set which AC_Fence::update()
        # derives from FENCE_TYPE at 10Hz, so give it time to notice; the
        # parameter on its own is not what is enforced.
        self.set_parameter("FENCE_TYPE", 1)  # ALT_MAX
        self.delay_sim_time(1, reason="let AC_Fence::update() pick up FENCE_TYPE")
        self.assert_sensor_state(
            mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE,
            present=True,
            enabled=True,
        )

        self.start_subtest("a type change leaves a commanded fence enabled")
        # the runtime set also carries MAV_CMD_DO_FENCE_ENABLE, the RC switch
        # and the auto-enabled floor, none of which a FENCE_TYPE write asks us
        # to revisit.  Discarding them would take the fence off a flying
        # vehicle, so a type change may only add and remove types.
        self.set_parameters({
            "FENCE_ENABLE": 0,
            "FENCE_TYPE": 1,  # ALT_MAX
        })
        self.delay_sim_time(1, reason="let AC_Fence::update() settle")
        self.do_fence_enable()
        self.assert_sensor_state(
            mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE,
            present=True,
            enabled=True,
        )
        self.set_parameter("FENCE_TYPE", 3)  # add the circle fence
        self.delay_sim_time(1, reason="let AC_Fence::update() pick up FENCE_TYPE")
        self.assert_sensor_state(
            mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE,
            present=True,
            enabled=True,
        )

        # the remaining cases turn on which types are enforced rather than
        # whether any are, so they fly: a breach names the fence which
        # reported it.  FENCE_ACTION is report-only and avoidance is off, so
        # the vehicle holds its hover and one flight covers every case.
        self.set_parameters({
            "AVOID_ENABLE": 0,
            "FENCE_ACTION": 0,  # report only
            "FENCE_ALT_MAX": 50,
            "FENCE_ALT_MIN": 5,
            "FENCE_RADIUS": 300,
            "FENCE_ENABLE": 0,
            "FENCE_TYPE": 1,  # ALT_MAX
        })
        self.takeoff(20, mode='LOITER')
        self.do_fence_enable()
        self.assert_fence_breach_reported("Max Alt", "FENCE_ALT_MAX", 10, 50)

        self.start_subtest("a type removed from an enabled fence is not enforced")
        self.set_parameter("FENCE_TYPE", 0)
        self.delay_sim_time(1, reason="let AC_Fence::update() pick up FENCE_TYPE")
        self.assert_sensor_state(
            mavutil.mavlink.MAV_SYS_STATUS_GEOFENCE,
            present=False,
            enabled=False,
        )
        self.assert_no_fence_breach_reported("FENCE_ALT_MAX", 10, 50)

        self.start_subtest("a type re-added to an enabled fence is enforced")
        # emptying FENCE_TYPE is not a disable: the pilot never turned the
        # fence off, so a type configured afterwards is enforced again
        self.set_parameter("FENCE_TYPE", 1)  # ALT_MAX
        self.delay_sim_time(1, reason="let AC_Fence::update() pick up FENCE_TYPE")
        self.assert_fence_breach_reported("Max Alt", "FENCE_ALT_MAX", 10, 50)

        self.start_subtest("a one-write type swap is enforced")
        # a single write which both adds and removes a type must leave the
        # fence enforcing the type which arrived, not nothing at all
        self.do_fence_disable()
        self.set_parameter("FENCE_TYPE", 2)  # CIRCLE only
        self.delay_sim_time(1, reason="let AC_Fence::update() settle")
        self.do_fence_enable()
        self.set_parameter("FENCE_TYPE", 1)  # swap the circle for the ceiling
        self.delay_sim_time(1, reason="let AC_Fence::update() pick up FENCE_TYPE")
        self.assert_fence_breach_reported("Max Alt", "FENCE_ALT_MAX", 10, 50)

        self.start_subtest("a type change leaves the min-alt floor enforced")
        # the floor is never enabled by being configured - it is enabled on
        # reaching its altitude - but a type change must not drop the floor
        # the pilot has enabled either
        self.do_fence_disable()
        self.set_parameter("FENCE_TYPE", 8)  # ALT_MIN only
        self.delay_sim_time(1, reason="let AC_Fence::update() settle")
        self.do_fence_enable()
        self.set_parameter("FENCE_TYPE", 9)  # add the ceiling in one write
        self.delay_sim_time(1, reason="let AC_Fence::update() pick up FENCE_TYPE")
        self.assert_fence_breach_reported("Min Alt", "FENCE_ALT_MIN", 30, 5)
        self.assert_fence_breach_reported("Max Alt", "FENCE_ALT_MAX", 10, 50)

        self.start_subtest("a floor removed from FENCE_TYPE and re-added is enforced")
        # the floor is excepted from being enabled by configuration, because
        # it comes on when its altitude is reached.  That exception must not
        # swallow a floor which was already being enforced when FENCE_TYPE
        # lost it: nothing asked for it to stop, and with FENCE_ENABLE clear
        # auto_enable_fence_floor() will not bring it back either.
        self.do_fence_disable()
        self.set_parameter("FENCE_TYPE", 9)  # ALT_MAX|ALT_MIN
        self.delay_sim_time(1, reason="let AC_Fence::update() settle")
        self.do_fence_enable()  # the floor comes on with the rest
        self.set_parameter("FENCE_TYPE", 1)  # drop the floor
        self.delay_sim_time(1, reason="let AC_Fence::update() pick up FENCE_TYPE")
        self.set_parameter("FENCE_TYPE", 9)  # and configure it again
        self.delay_sim_time(1, reason="let AC_Fence::update() pick up FENCE_TYPE")
        self.assert_fence_breach_reported("Min Alt", "FENCE_ALT_MIN", 30, 5)

        self.start_subtest("the same with FENCE_ENABLE set")
        # the vehicle has to hover *inside* the floor's margin, or
        # auto_enable_fence_floor() turns the floor back on by itself and the
        # subtest passes whatever the type change did.  A 16m floor with 10m of
        # margin auto-enables at 26m, so a hover at 20m is six metres clear of
        # it rather than one; assert that rather than trust it, because a
        # subtest which has drifted out of the band proves nothing.
        self.set_parameters({
            "FENCE_ALT_MIN": 16,
            "FENCE_MARGIN": 10,
            "FENCE_ENABLE": 1,
            "FENCE_TYPE": 9,
        })
        self.assert_altitude(21, accuracy=4, relative=True)
        self.delay_sim_time(1, reason="let AC_Fence::update() settle")
        self.do_fence_enable()  # FENCE_ENABLE does not turn the floor on
        self.set_parameter("FENCE_TYPE", 1)
        self.delay_sim_time(1, reason="let AC_Fence::update() pick up FENCE_TYPE")
        self.set_parameter("FENCE_TYPE", 9)
        self.delay_sim_time(1, reason="let AC_Fence::update() pick up FENCE_TYPE")
        self.assert_fence_breach_reported("Min Alt", "FENCE_ALT_MIN", 30, 16)
        self.set_parameters({
            "FENCE_ENABLE": 0,
            "FENCE_ALT_MIN": 5,
            "FENCE_MARGIN": 2,
        })
        self.delay_sim_time(1, reason="let AC_Fence::update() settle")

        self.start_subtest("an enable between losing and regaining the floor keeps it")
        # a floor which was being enforced when FENCE_TYPE stopped carrying it
        # is still wanted, and an enable arriving in between - here one which
        # does not even name the floor - must not forget that.  Inside the
        # margin again, so the auto-enable cannot restore it for us.
        self.do_fence_disable()
        self.set_parameters({
            "FENCE_ALT_MIN": 16,
            "FENCE_MARGIN": 10,
            "FENCE_TYPE": 9,  # ALT_MAX|ALT_MIN
        })
        self.assert_altitude(21, accuracy=4, relative=True)
        self.delay_sim_time(1, reason="let AC_Fence::update() settle")
        self.do_fence_enable()  # the floor is enforced
        self.set_parameter("FENCE_TYPE", 1)  # the floor leaves FENCE_TYPE
        self.delay_sim_time(1, reason="let AC_Fence::update() pick up FENCE_TYPE")
        self.do_fence_enable_except_floor()
        self.set_parameter("FENCE_TYPE", 9)  # and comes back
        self.delay_sim_time(1, reason="let AC_Fence::update() pick up FENCE_TYPE")
        self.assert_fence_breach_reported("Min Alt", "FENCE_ALT_MIN", 30, 16)
        self.set_parameters({
            "FENCE_ALT_MIN": 5,
            "FENCE_MARGIN": 2,
        })

        self.start_subtest("a floor configured for the first time is not enforced")
        # a blanket enable names every fence type, including a floor which is
        # not configured.  That must not become a floor enforced the moment one
        # is configured: the floor comes on when its altitude is reached.  The
        # hover is inside the margin again, so nothing else can enable it.
        self.do_fence_disable()
        self.set_parameters({
            "FENCE_ALT_MIN": 16,
            "FENCE_MARGIN": 10,
            "FENCE_TYPE": 1,  # ALT_MAX only: the floor has never been configured
        })
        self.assert_altitude(21, accuracy=4, relative=True)
        self.delay_sim_time(1, reason="let AC_Fence::update() settle")
        self.do_fence_enable()  # a blanket enable, which names the floor too
        self.set_parameter("FENCE_TYPE", 9)  # configure the floor
        self.delay_sim_time(1, reason="let AC_Fence::update() pick up FENCE_TYPE")
        self.assert_no_fence_breach_reported("FENCE_ALT_MIN", 30, 16)
        self.set_parameters({
            "FENCE_ALT_MIN": 5,
            "FENCE_MARGIN": 2,
        })

        self.start_subtest("a floor-only disable leaves the other types enforced")
        # disabling the floor - which a landing does, and a GCS can ask for -
        # is a request about one type, not the pilot taking the fence off, so
        # a type configured afterwards is still enforced.  The floor may be
        # the only type enabled when it happens, so what is enforced cannot be
        # used to tell the two apart.
        self.do_fence_disable()
        self.set_parameter("FENCE_TYPE", 9)  # ALT_MAX|ALT_MIN
        self.delay_sim_time(1, reason="let AC_Fence::update() settle")
        self.do_fence_enable()
        self.set_parameter("FENCE_TYPE", 8)  # drop the ceiling
        self.delay_sim_time(1, reason="let AC_Fence::update() pick up FENCE_TYPE")
        self.do_fence_disable_floor()
        self.set_parameter("FENCE_TYPE", 9)  # and configure the ceiling again
        self.delay_sim_time(1, reason="let AC_Fence::update() pick up FENCE_TYPE")
        self.assert_fence_breach_reported("Max Alt", "FENCE_ALT_MAX", 10, 50)

        self.start_subtest("a type added after a disable at FENCE_TYPE 0 is not enforced")
        # a disable which arrives while nothing is configured changes no bit,
        # and must still take the fence off: a type configured afterwards is
        # not enforced.  Master goes on enforcing the type it had.
        self.do_fence_enable()
        self.set_parameter("FENCE_TYPE", 0)
        self.delay_sim_time(1, reason="let AC_Fence::update() pick up FENCE_TYPE")
        self.do_fence_disable()  # accepted, and changes no enforced type
        self.set_parameter("FENCE_TYPE", 1)  # ALT_MAX configured again
        self.delay_sim_time(1, reason="let AC_Fence::update() pick up FENCE_TYPE")
        self.assert_no_fence_breach_reported("FENCE_ALT_MAX", 10, 50)

        self.start_subtest("a circle fence added by a type change is enforced")
        # the circle is centred on home and FENCE_RADIUS may not go below 30m,
        # so the vehicle is flown out rather than the limit brought in
        self.reach_distance_manual(60)
        self.do_fence_disable()
        self.set_parameter("FENCE_TYPE", 8)  # ALT_MIN only
        self.delay_sim_time(1, reason="let AC_Fence::update() settle")
        self.do_fence_enable()
        self.set_parameter("FENCE_TYPE", 10)  # add the circle in one write
        self.delay_sim_time(1, reason="let AC_Fence::update() pick up FENCE_TYPE")
        self.assert_fence_breach_reported("Circle", "FENCE_RADIUS", 30, 300)

        self.do_fence_disable()
        self.set_parameter("FENCE_TYPE", 0)
        # the circle subtest left the vehicle 60m out, and a test has to end
        # where it started, so come home rather than landing where we are
        self.do_RTL()
