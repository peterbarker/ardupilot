--[[
   Power-On Self Test (POST) for firmware ("Code") and parameters ("Data").

   On boot this script:
   - waits 10 seconds, then waits for a valid UTC time from the GPS
   - SHA256-checksums a fixed list of parameters ("Data")
   - SHA256-checksums the firmware in flash, skipping the bootloader ("Code")
   - compares both against known-good checksums
   - emits a grouped POST report over MAVLink (statustext)
   - appends one timestamped POST line to a file on the SD card
   - blocks arming (via aux authorisation) unless both checksums match

   It runs once.

   NOTE: UTC time is derived from the GPS.  On a vehicle with no GPS (or
   before a GPS time fix) the POST waits indefinitely and arming stays
   blocked.  Adjust the "wait for valid time" handling in update() if a
   bench / no-GPS fallback is required.
--]]

local required_parameters_checksum = "eaa29a6a1c10b457f16e10b380bb4846168e548bbe0ff006116fd472501feda1"
local parameters_to_checksum = {
   "VISO_TYPE",  -- 0
   "TUNE",       -- 0
   "RELAY1_PIN"  -- 12
}

local required_firmware_checksum = "78FE958FD095EA2F06A10E59BC085C06CE385DBF997F03A3BC7E321E4C5A1B70"

local flash_file_path = "@SYS/flash.bin"

-- Bytes to skip at the start of @SYS/flash.bin before hashing the firmware:
-- the bootloader occupies FLASH_RESERVE_START_KB at the base of flash.  This
-- is board-specific (128 KB on most STM32H7 boards); see the board's hwdef.dat
-- and Tools/scripts/sha256_padded_firmware.py.
local bootloader_reserve_bytes = 128 * 1024

local sdcard_file_name = "checksum_parameters.txt"

-- UTC-from-GPS conversion constants (see AP_GPS.h)
local GPS_UNIX_EPOCH_S = 315964800   -- 1980-01-06 00:00:00 UTC, in unix seconds
local SECS_PER_WEEK    = 604800
local GPS_LEAP_SECONDS = 18          -- GPS_LEAPSECONDS_MILLIS / 1000

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- SHA256 round constants
local sha256_k = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
}

local function rrot32(x, n)
    return ((x >> n) | (x << (32 - n))) & 0xFFFFFFFF
end

local function add32(a, b)
    return (a + b) & 0xFFFFFFFF
end

local function sha256_process_block(ctx, block)
    local w = {}
    for i = 1, 16 do
        local j = (i - 1) * 4
        w[i] = (block[j+1] << 24) | (block[j+2] << 16) | (block[j+3] << 8) | block[j+4]
    end
    for i = 17, 64 do
        local s0 = rrot32(w[i-15], 7) ~ rrot32(w[i-15], 18) ~ (w[i-15] >> 3)
        local s1 = rrot32(w[i-2], 17) ~ rrot32(w[i-2], 19) ~ (w[i-2] >> 10)
        w[i] = add32(add32(add32(w[i-16], s0), w[i-7]), s1)
    end

    local a, b, c, d, e, f, g, h = table.unpack(ctx.hash)

    for i = 1, 64 do
        local S1 = rrot32(e, 6) ~ rrot32(e, 11) ~ rrot32(e, 25)
        local ch = (e & f) ~ (~e & g)
        local temp1 = add32(add32(add32(add32(h, S1), ch), sha256_k[i]), w[i])
        local S0 = rrot32(a, 2) ~ rrot32(a, 13) ~ rrot32(a, 22)
        local maj = (a & b) ~ (a & c) ~ (b & c)
        local temp2 = add32(S0, maj)

        h = g
        g = f
        f = e
        e = add32(d, temp1)
        d = c
        c = b
        b = a
        a = add32(temp1, temp2)
    end

    ctx.hash[1] = add32(ctx.hash[1], a)
    ctx.hash[2] = add32(ctx.hash[2], b)
    ctx.hash[3] = add32(ctx.hash[3], c)
    ctx.hash[4] = add32(ctx.hash[4], d)
    ctx.hash[5] = add32(ctx.hash[5], e)
    ctx.hash[6] = add32(ctx.hash[6], f)
    ctx.hash[7] = add32(ctx.hash[7], g)
    ctx.hash[8] = add32(ctx.hash[8], h)
end

local function sha256_init()
    return {
        hash = {
            0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
            0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19
        },
        buf = {},
        len = 0
    }
end

-- feed a table of uint8 byte values into the SHA256 context
local function sha256_update(ctx, bytes)
    local buf = ctx.buf
    for i = 1, #bytes do
        buf[#buf + 1] = bytes[i]
        ctx.len = ctx.len + 1
        if #buf == 64 then
            sha256_process_block(ctx, buf)
            ctx.buf = {}
            buf = ctx.buf
        end
    end
end

-- feed a Lua string into the SHA256 context
local function sha256_update_string(ctx, s)
    local bytes = {}
    for i = 1, #s do
        bytes[i] = s:byte(i)
    end
    sha256_update(ctx, bytes)
end

-- finalise and return the digest as a 64-character lowercase hex string
local function sha256_final(ctx)
    local buf = ctx.buf
    local len = ctx.len

    buf[#buf + 1] = 0x80
    while #buf ~= 56 do
        if #buf == 64 then
            sha256_process_block(ctx, buf)
            ctx.buf = {}
            buf = ctx.buf
        else
            buf[#buf + 1] = 0x00
        end
    end

    -- message length in bits as a 64-bit big-endian integer
    local bit_len = len * 8
    buf[57] = (bit_len >> 56) & 0xFF
    buf[58] = (bit_len >> 48) & 0xFF
    buf[59] = (bit_len >> 40) & 0xFF
    buf[60] = (bit_len >> 32) & 0xFF
    buf[61] = (bit_len >> 24) & 0xFF
    buf[62] = (bit_len >> 16) & 0xFF
    buf[63] = (bit_len >>  8) & 0xFF
    buf[64] =  bit_len        & 0xFF

    sha256_process_block(ctx, buf)

    local result = ""
    for i = 1, 8 do
        result = result .. string.format("%08x", ctx.hash[i])
    end
    return result
end

-- state
local param_sha256_ctx = sha256_init()
local param_checksummed = false
local fw_checksummed = false

-- "Data" = parameter checksum results
local data_match = false
local data_digest = nil          -- computed digest (lowercase hex) or nil
local data_invalid_param = nil   -- name of the first missing parameter, if any

-- "Code" = firmware checksum results
local code_match = false
local code_digest = nil          -- computed digest (lowercase hex) or nil on read failure

local CALC_RESULT_INCOMPLETE = 0
local CALC_RESULT_DONE = 1

local auth_id = arming:get_aux_auth_id()

-- current UTC time from the GPS, in unix seconds, or nil if not yet known
local function gps_utc_seconds()
   local i = gps:primary_sensor()
   local week = gps:time_week(i)
   if week == nil or week == 0 then
      return nil
   end
   local tow_ms = gps:time_week_ms(i):toint()
   -- Computed in integer seconds: the result (~1.77e9) fits a 32-bit Lua
   -- integer, whereas the equivalent in milliseconds would overflow.  This
   -- overflows in 2038 (Y2038), in common with the rest of the firmware.
   return GPS_UNIX_EPOCH_S + week * SECS_PER_WEEK + (tow_ms // 1000) - GPS_LEAP_SECONDS
end

-- "DD/MM/YYYY HH:MM:SS UTC" from the GPS, or nil if time is not yet known
local function utc_timestamp()
   local secs = gps_utc_seconds()
   if secs == nil then
      return nil
   end
   local year, month, day, hour, min, sec = rtc:clock_s_to_date_fields(uint32_t(secs))
   if year == nil then
      return nil
   end
   -- clock_s_to_date_fields returns month as 0-11
   return string.format("%02d/%02d/%04d %02d:%02d:%02d UTC",
                        day, month + 1, year, hour, min, sec)
end

-- send a full 64-char digest over MAVLink: statustext is capped at 50 bytes,
-- so the label + first 32 hex go on one line and the last 32 hex on the next
local function send_digest(severity, label, digest)
   if digest == nil then
      gcs:send_text(severity, label .. " unavailable")
      return
   end
   local d = digest:lower()
   gcs:send_text(severity, label .. " " .. d:sub(1, 32))
   gcs:send_text(severity, d:sub(33, 64))
end

local function append_to_sdcard_file(text)
   local file = io.open(sdcard_file_name, 'a')
   if file ~= nil then
      file:write(text .. "\n")
      io.close(file)
   end
end

-- compute the parameter ("Data") checksum, a few parameters per call
local function update_param_calculation()
   local count = 0
   for i = #parameters_to_checksum, 1, -1 do
      local s = parameters_to_checksum[i]
      local parameter_value = param:get(s)
      if parameter_value == nil then
         data_invalid_param = s
         data_match = false
         param_checksummed = true
         return CALC_RESULT_DONE
      end
      table.remove(parameters_to_checksum, i)

      -- pack the value as a 4-byte little-endian IEEE 754 float
      sha256_update_string(param_sha256_ctx, string.pack("<f", parameter_value))

      count = count + 1
      -- limit parameters processed per loop to avoid overflowing vmcount
      if count >= 10 then
         return CALC_RESULT_INCOMPLETE
      end
   end

   data_digest = sha256_final(param_sha256_ctx)
   data_match = data_digest:lower() == required_parameters_checksum:lower()
   param_checksummed = true
   return CALC_RESULT_DONE
end

-- compute the firmware ("Code") checksum
local function update_firmware_calculation()
   code_digest = fs:sha256(flash_file_path, bootloader_reserve_bytes)
   if code_digest ~= nil then
      code_match = code_digest:lower() == required_firmware_checksum:lower()
   else
      code_match = false
   end
   fw_checksummed = true
end

-- emit the MAVLink report, write the SD log line and set the arming result
local function emit_report(timestamp)
   local overall_ok = code_match and data_match

   -- POST headline
   local headline
   if overall_ok then
      headline = "POST: PASS"
   else
      local parts = {}
      if not code_match then parts[#parts + 1] = "Code" end
      if not data_match then parts[#parts + 1] = "Data" end
      headline = "POST: FAIL (" .. table.concat(parts, ", ") .. ")"
   end
   gcs:send_text(overall_ok and MAV_SEVERITY.NOTICE or MAV_SEVERITY.ERROR, headline)

   -- Code (firmware) checksum lines (full digest, split across two lines)
   if code_match then
      send_digest(MAV_SEVERITY.NOTICE, "Code Checksum:", code_digest)
   elseif code_digest == nil then
      gcs:send_text(MAV_SEVERITY.ERROR, "Code Checksum: read failed")
   else
      gcs:send_text(MAV_SEVERITY.ERROR, "Code Checksum:")
      send_digest(MAV_SEVERITY.ERROR, "Received:", code_digest)
      send_digest(MAV_SEVERITY.ERROR, "Expected:", required_firmware_checksum)
   end

   -- Data (parameter) checksum lines (full digest, split across two lines)
   if data_match then
      send_digest(MAV_SEVERITY.NOTICE, "Data Checksum:", data_digest)
   elseif data_invalid_param ~= nil then
      gcs:send_text(MAV_SEVERITY.ERROR, "Data Checksum: invalid param " .. data_invalid_param)
   else
      gcs:send_text(MAV_SEVERITY.ERROR, "Data Checksum:")
      send_digest(MAV_SEVERITY.ERROR, "Received:", data_digest)
      send_digest(MAV_SEVERITY.ERROR, "Expected:", required_parameters_checksum)
   end

   -- SD card log line
   local log_line
   if overall_ok then
      log_line = string.format("%s POST: PASS (FW: %s (%s))",
                               timestamp, FWVersion:string(), FWVersion:hash())
   else
      log_line = timestamp .. " " .. headline
   end
   append_to_sdcard_file(log_line)

   -- arming aux authorisation
   if auth_id ~= nil then
      if overall_ok then
         arming:set_aux_auth_passed(auth_id)
      else
         local parts = {}
         if not code_match then parts[#parts + 1] = "Code" end
         if not data_match then parts[#parts + 1] = "Data" end
         arming:set_aux_auth_failed(auth_id, table.concat(parts, " & ") .. " Checksum FAIL")
      end
   end
end

local function update()
   -- wait until we've been booted 10 seconds before starting:
   if millis() < 10000 then
      return update, 100
   end

   -- wait for a valid UTC time from the GPS before running the POST
   local timestamp = utc_timestamp()
   if timestamp == nil then
      return update, 100
   end

   if not param_checksummed then
      if update_param_calculation() == CALC_RESULT_INCOMPLETE then
         return update, 100
      end
   end

   if not fw_checksummed then
      update_firmware_calculation()
   end

   emit_report(timestamp)

   return -- one-shot script
end

if auth_id ~= nil then
   arming:set_aux_auth_failed(auth_id, "Checksum POST pending")
end

return update()
