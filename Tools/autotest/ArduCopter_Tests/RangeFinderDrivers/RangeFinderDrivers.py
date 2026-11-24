'''
Copter autotest: RangeFinderDrivers

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class RangeFinderDrivers(TestGroup):
    @testcase(bucket="CopterTests1e")
    def RangeFinderDrivers(self):
        '''Test rangefinder drivers'''
        self.set_parameters({
            "RTL_ALT_M": 5,
            "RTL_ALT_TYPE": 1,
            "SIM_TERRAIN": 0,
        })
        drivers = [
            ("lightwareserial", 8),  # autodetected between this and -binary
            ("lightwareserial-binary", 8),
            ("USD1_v0", 11),
            ("USD1_v1", 11),
            ("leddarone", 12),
            ("maxsonarseriallv", 13),
            ("nmea", 17, {"baud": 9600}),
            ("wasp", 18),
            ("benewake_tf02", 19),
            ("blping", 23),
            ("benewake_tfmini", 20),
            ("lanbao", 26),
            ("benewake_tf03", 27),
            ("gyus42v2", 31),
            ("teraranger_serial", 35),
            ("nooploop_tofsense", 37),
            ("ainsteinlrd1", 42),
            ("rds02uf", 43),
            ("lightware_grf", 45),
            ("dts6012m", 47),
        ]
        # you can use terrain - if you don't the vehicle just uses a
        # plane based on home.
        # self.install_terrain_handlers_context()
        while len(drivers):
            do_drivers = drivers[0:3]
            drivers = drivers[3:]
            command_line_args = []
            self.context_push()
            for offs in range(3):
                serial_num = offs + 4
                if len(do_drivers) > offs:
                    if len(do_drivers[offs]) > 2:
                        (sim_name, rngfnd_param_value, kwargs) = do_drivers[offs]
                    else:
                        (sim_name, rngfnd_param_value) = do_drivers[offs]
                        kwargs = {}
                    command_line_args.append("--serial%s=sim:%s" %
                                             (serial_num, sim_name))
                    sets = {
                        "SERIAL%u_PROTOCOL" % serial_num: 9, # rangefinder
                        "RNGFND%u_TYPE" % (offs+1): rngfnd_param_value,
                    }
                    if "baud" in kwargs:
                        sets["SERIAL%u_BAUD" % serial_num] = kwargs["baud"]
                    self.set_parameters(sets)
            self.customise_SITL_commandline(command_line_args)
            self.fly_rangefinder_drivers_fly([x[0] for x in do_drivers])
            self.context_pop()

        class I2CDriverToTest:
            def __init__(self, name, rngfnd_type, rngfnd_addr=None):
                self.name = name
                self.rngfnd_type = rngfnd_type
                self.rngfnd_addr = rngfnd_addr

        i2c_drivers = [
            I2CDriverToTest("maxbotixi2cxl", 2),
            I2CDriverToTest("terarangeri2c", 14, rngfnd_addr=0x31),
            I2CDriverToTest("lightware_legacy16bit", 7, rngfnd_addr=0x66),
            I2CDriverToTest("tfs20l", 46, rngfnd_addr=0x10),
            I2CDriverToTest("TFMiniPlus", 25, rngfnd_addr=0x09),
            I2CDriverToTest("TOFSenseF_I2C", 40, rngfnd_addr=0x08),
            I2CDriverToTest("lightware_grf_i2c", 48, rngfnd_addr=0x66),
        ]
        while len(i2c_drivers):
            do_drivers = i2c_drivers[0:9]
            i2c_drivers = i2c_drivers[9:]
            count = 1
            p = {}
            for d in do_drivers:
                p[f"RNGFND{count}_TYPE"] = d.rngfnd_type
                if d.rngfnd_addr is not None:
                    p[f"RNGFND{count}_ADDR"] = d.rngfnd_addr
                count += 1

            self.set_parameters(p)

            self.reboot_sitl()
            self.fly_rangefinder_drivers_fly([x.name for x in do_drivers])
