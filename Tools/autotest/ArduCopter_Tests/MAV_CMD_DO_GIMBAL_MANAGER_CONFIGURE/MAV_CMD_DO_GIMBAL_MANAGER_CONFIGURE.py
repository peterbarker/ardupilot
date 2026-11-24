'''
Copter autotest: MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE

AP_FLAKE8_CLEAN
'''

from pymavlink import mavutil

from test_discovery import TestGroup
from test_discovery import testcase


class MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE(TestGroup):
    @testcase(bucket="CopterTests2c")
    def MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE(self):
        '''test MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE mavlink command'''
        # setup mount parameters
        self.context_push()
        self.setup_servo_mount()
        self.reboot_sitl() # to handle MNT_TYPE changing

        self.context_set_message_rate_hz('GIMBAL_MANAGER_STATUS', 10)
        self.assert_received_message_field_values('GIMBAL_MANAGER_STATUS', {
            "gimbal_device_id": 1,
            "primary_control_sysid": 0,
            "primary_control_compid": 0,
        })

        for method in self.run_cmd, self.run_cmd_int:
            self.start_subtest("set_sysid-compid")
            method(
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                p1=37,
                p2=38,
            )
            self.assert_received_message_field_values('GIMBAL_MANAGER_STATUS', {
                "gimbal_device_id": 1,
                "primary_control_sysid": 37,
                "primary_control_compid": 38,
            })

            self.start_subtest("leave unchanged")
            method(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE, p1=-1)
            self.assert_received_message_field_values('GIMBAL_MANAGER_STATUS', {
                "gimbal_device_id": 1,
                "primary_control_sysid": 37,
                "primary_control_compid": 38,
            })

            self.start_subtest("release control")
            method(
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                p1=self.mav.source_system,
                p2=self.mav.source_component,
            )
            self.assert_received_message_field_values('GIMBAL_MANAGER_STATUS', {
                "gimbal_device_id": 1,
                "primary_control_sysid": self.mav.source_system,
                "primary_control_compid": self.mav.source_component,
            })
            method(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE, p1=-3)
            self.assert_received_message_field_values('GIMBAL_MANAGER_STATUS', {
                "gimbal_device_id": 1,
                "primary_control_sysid": 0,
                "primary_control_compid": 0,
            })

            def check_control(sysid, compid):
                self.drain_mav()
                self.assert_received_message_field_values('GIMBAL_MANAGER_STATUS', {
                    "primary_control_sysid": sysid if sysid <= 255 else 0,
                    "primary_control_compid": compid,
                })

            command = mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
            self.start_subtest("fractional IDs retain truncation")
            method(command, p1=37.9, p2=38.9)
            check_control(37, 38)
            method(command, p1=-1.9)
            check_control(37, 38)
            method(command, p1=-0.9, p2=38.9)
            check_control(0, 38)

            self.start_subtest("full-range IDs and sender sentinels")
            old_source = self.mav.source_system
            old_component = self.mav.source_component
            try:
                for sysid in (16777216, 16777218, 0x80000000, 0xFFFFFF00):
                    method(command, p1=sysid, p2=old_component)
                    check_control(sysid, old_component)
                    # Status has only an 8-bit sysid. Prove exact ownership by
                    # attempting release from a neighbour, then the actual ID.
                    self.mav.source_system = sysid + 1
                    self.mav.mav.srcSystem = sysid + 1
                    method(command, p1=-3)
                    check_control(sysid, old_component)
                    self.mav.source_system = sysid
                    self.mav.mav.srcSystem = sysid
                    method(command, p1=-3)
                    check_control(0, 0)

                self.mav.source_system = 0xFFFFFFFF
                self.mav.mav.srcSystem = 0xFFFFFFFF
                method(command, p1=-2.9)
                check_control(0xFFFFFFFF, old_component)
                method(command, p1=-3)
                check_control(0, 0)
            finally:
                self.mav.source_system = old_source
                self.mav.mav.srcSystem = old_source

            self.start_subtest("out-of-range IDs leave ownership unchanged")
            method(command, p1=37, p2=38)
            # Legacy range comparisons trap on NaN with SITL float exceptions enabled.
            for value in (float('inf'), float('-inf'), -3.1, 0xFFFFFFFF, 2**32, 2**40):
                method(command, p1=value, p2=38, want_result=mavutil.mavlink.MAV_RESULT_FAILED)
                check_control(37, 38)
            for value in (float('inf'), float('-inf'), -3.1, 256):
                method(command, p1=37, p2=value, want_result=mavutil.mavlink.MAV_RESULT_FAILED)
                check_control(37, 38)

        self.context_pop()
        self.reboot_sitl()
