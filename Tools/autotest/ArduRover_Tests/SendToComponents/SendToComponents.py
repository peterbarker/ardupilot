'''
Rover autotest: SendToComponents

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class SendToComponents(TestGroup):
    @testcase(bucket="Rover")
    def SendToComponents(self):
        '''Test ArduPilot send_to_components function'''
        self.set_parameter("CAM1_TYPE", 5) # Camera with MAVlink trigger
        self.reboot_sitl() # needed for CAM1_TYPE to take effect
        self.progress("Introducing ourselves to the autopilot as a component")
        old_srcSystem = self.mav.mav.srcSystem
        self.mav.mav.srcSystem = 1
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0)

        self.progress("Sending control message")
        self.context_push()
        self.context_collect('COMMAND_LONG')
        self.mav.mav.digicam_control_send(
            1, # target_system
            1, # target_component
            1, # start or keep it up
            1, # zoom_pos
            0, # zoom_step
            0, # focus_lock
            0, # 1 shot or start filming
            17, # command id (de-dupe field)
            0, # extra_param
            0.0, # extra_value
        )
        self.mav.mav.srcSystem = old_srcSystem

        self.assert_received_message_field_values('COMMAND_LONG', {
            'command': mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
            'param6': 17,
        }, timeout=2, check_context=True)
        self.context_pop()

        # test sending via commands:
        for run_cmd in self.run_cmd, self.run_cmd_int:
            self.progress("Sending control command")
            self.context_push()
            self.context_collect('COMMAND_LONG')
            run_cmd(mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
                    p1=1, # start or keep it up
                    p2=1, # zoom_pos
                    p3=0, # zoom_step
                    p4=0, # focus_lock
                    p5=0, # 1 shot or start filming
                    p6=37, # command id (de-dupe field)
                    )

            self.assert_received_message_field_values('COMMAND_LONG', {
                'command': mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
                'param6': 37,
            }, timeout=2, check_context=True)

            self.context_pop()

        # test sending via commands:
        for run_cmd in self.run_cmd, self.run_cmd_int:
            self.progress("Sending configure command")
            self.context_push()
            self.context_collect('COMMAND_LONG')
            run_cmd(mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONFIGURE,
                    p1=1,
                    p2=1,
                    p3=0,
                    p4=0,
                    p5=12,
                    p6=37
                    )

            self.assert_received_message_field_values('COMMAND_LONG', {
                'command': mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONFIGURE,
                'param5': 12,
                'param6': 37,
            }, timeout=2, check_context=True)

            self.context_pop()

        self.mav.mav.srcSystem = old_srcSystem
