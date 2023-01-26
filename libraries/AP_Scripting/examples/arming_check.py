#!/usr/bin/python

import select
import sys
import time

from pymavlink import mavutil


class ArmingCheckPoker(object):
    '''object to connect to an autopilot via mavlink and manipulate its
    arming state via an auxiliary function'''

    def __init__(self,
                 connect_string,
                 source_system=222,
                 source_component=222,
                 target_system=1,
                 target_component=1,
                 ):
        self.connect_string = connect_string
        self.source_system = source_system
        self.source_component = source_component
        self.target_system = target_system
        self.target_component = target_component

    def connect(self):
        self.mav = mavutil.mavlink_connection(
            self.connect_string,
            source_system=self.source_system,
            source_component=self.source_component,
            autoreconnect=True,
            dialect="all",
        )

    def set_armable(self, value):
        self.value = value

    def update(self):
        p1 = 300  # magic RCn_OPTION for SCRIPTING1
        if value:
            p2 = 2  # HIGH
        else:
            p2 = 0  # MIDDLE

        self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                    0,
                                    0,
                                    0)

        self.mav.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION,
            1,  # confirmation
            p1,
            p2,
            0,
            0,
            0,
            0,
            0)

        while True:
            m = self.mav.recv_msg()
            if m is None:
                break


if __name__ == '__main__':
    acp = ArmingCheckPoker(":14550")
    acp.connect()
    print("Press enter to toggle armability")
    value = True
    while True:
        read_line = False
        while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            read_line = True
        if read_line:
            value = not value
            if value:
                print("Making it armable")
            else:
                print("Making it NOT armable")

        acp.set_armable(value)
        acp.update()

        time.sleep(0.1)
