local TARGET_ABSOLUTE = {}
TARGET_ABSOLUTE.id = 510
TARGET_ABSOLUTE.crc_extra = 132
TARGET_ABSOLUTE.fields = {
             { "time_usec", "<I8" },
             { "lat", "<i4" },
             { "lon", "<i4" },
             { "alt", "<f" },
             { "vel", "<f", 3 },
             { "acc", "<f", 3 },
             { "q_target", "<f", 4 },
             { "rates", "<f", 3 },
             { "position_std", "<f", 2 },
             { "vel_std", "<f", 3 },
             { "acc_std", "<f", 3 },
             { "id", "<B" },
             { "sensor_capabilities", "<B" },
             }
return TARGET_ABSOLUTE
