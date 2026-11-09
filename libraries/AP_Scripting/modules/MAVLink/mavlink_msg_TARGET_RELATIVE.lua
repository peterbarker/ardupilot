local TARGET_RELATIVE = {}
TARGET_RELATIVE.id = 511
TARGET_RELATIVE.crc_extra = 5
TARGET_RELATIVE.fields = {
             { "time_usec", "<I8" },
             { "x", "<f" },
             { "y", "<f" },
             { "z", "<f" },
             { "position_std", "<f", 3 },
             { "yaw_std", "<f" },
             { "q_target", "<f", 4 },
             { "q_sensor", "<f", 4 },
             { "id", "<B" },
             { "frame", "<B" },
             { "type", "<B" },
             }
return TARGET_RELATIVE
