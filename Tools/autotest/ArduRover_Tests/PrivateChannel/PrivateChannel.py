'''
Rover autotest: PrivateChannel

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class PrivateChannel(TestGroup):
    @testcase(bucket="Rover")
    def PrivateChannel(self):
        '''test the serial option bit specifying a mavlink channel as private'''
        global mav2
        mav2 = mavutil.mavlink_connection(self.sitl_serial_endpoint(2),
                                          robust_parsing=True,
                                          source_system=7,
                                          source_component=7)
        # send a heartbeat or two to make sure ArduPilot's aware:

        def heartbeat_on_mav2(mav, m):
            '''send a heartbeat on mav2 whenever we get one on mav'''
            if mav == mav2:
                return
            if m.get_type() == 'HEARTBEAT':
                mav2.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0)
                return

        self.assert_receive_message("HEARTBEAT", mav=mav2)

        # ensure a targeted message is received:
        self.install_message_hook_context(heartbeat_on_mav2)

        self.progress("Ensuring we can get a message normally")
        self.poll_message("AUTOPILOT_VERSION", mav=mav2)

        self.progress("Polling AUTOPILOT_VERSION from random sysid")
        self.send_poll_message("AUTOPILOT_VERSION", mav=mav2, target_sysid=134)
        self.assert_not_receive_message("AUTOPILOT_VERSION", mav=mav2, timeout=10)

        # make sure we get heartbeats on the main channel from the non-private mav2:
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 5:
                raise NotAchievedException("Did not get expected heartbeat from %u" % 7)
            m = self.assert_receive_message("HEARTBEAT")
            if m.get_srcSystem() == 7:
                self.progress("Got heartbeat from (%u) on non-private channel" % 7)
                break

        # make sure we receive heartbeats from the autotest suite into
        # the component:
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 5:
                raise NotAchievedException("Did not get expected heartbeat from %u" % self.mav.source_system)
            m = self.assert_receive_message("HEARTBEAT", mav=mav2)
            if m.get_srcSystem() == self.mav.source_system:
                self.progress("Got heartbeat from (%u) on non-private channel" % self.mav.source_system)
                break

        def printmessage(mav, m):
            if mav == mav2:
                return

            print("Got (%u/%u) (%s) " % (m.get_srcSystem(), m.get_srcComponent(), str(m)))

#        self.install_message_hook_context(printmessage)

        # ensure setting the private channel mask doesn't cause us to
        # execute these commands:
        self.set_parameter("MAV3_OPTIONS", 2)
        self.reboot_sitl()  # mavlink-private is reboot-required
        mav2 = mavutil.mavlink_connection(self.sitl_serial_endpoint(2),
                                          robust_parsing=True,
                                          source_system=7,
                                          source_component=7)
#        self.send_debug_trap()
        self.send_poll_message("AUTOPILOT_VERSION", mav=mav2, target_sysid=134)
        self.assert_not_receive_message("AUTOPILOT_VERSION", mav=mav2, timeout=10)

        # make sure messages from a private channel don't make it to
        # the main channel:
        self.drain_mav(self.mav)
        self.drain_mav(mav2)

        # make sure we do NOT get heartbeats on the main channel from
        # the private mav2:
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 5:
                break
            m = self.assert_receive_message("HEARTBEAT")
            if m.get_srcSystem() == 7:
                raise NotAchievedException("Got heartbeat from private channel")

        self.progress("ensure no outside heartbeats reach private channels")
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 5:
                break
            m = self.assert_receive_message("HEARTBEAT")
            if m.get_srcSystem() == 1 and m.get_srcComponent() == 1:
                continue
            # note the above test which shows we get heartbeats from
            # both the vehicle and this tests's special heartbeat
            raise NotAchievedException("Got heartbeat on private channel from non-vehicle")
