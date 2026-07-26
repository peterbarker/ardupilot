--[[
   stream DroneCAN broadcasts as fast as the driver accepts them, to
   find what a link carries with no request and response in the way.
   This is a measurement tool, not an applet: see dronecan_bench.md
   beside it

     DCST_SIZE  payload bytes per broadcast, 1 to 1000
     DCST_RUN   1 to stream, 0 to stop

   nothing subscribes at the far end.  The frames still arrive and are
   counted by the receiving driver, so the rate is read out of
   @SYS/canN_stats.txt at both ends rather than by a script, which
   keeps the receiving interpreter out of the measurement entirely
--]]

local MAV_SEVERITY = {ERROR=3, INFO=6}

local PARAM_TABLE_KEY = 121
local PARAM_TABLE_PREFIX = "DCST_"

-- a data type of our own; nothing else on this bus uses it
local STREAM_DATA_TYPE = 20500
local STREAM_SIGNATURE = uint64_t(0x5A5A0001, 0x0BADF00D)

-- how many to offer per scheduler pass before yielding
local MAX_BURST = 64

param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2)
assert(param:add_param(PARAM_TABLE_KEY, 1, 'SIZE', 256), 'could not add DCST_SIZE')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'RUN', 0), 'could not add DCST_RUN')
local DCST_SIZE = Parameter(PARAM_TABLE_PREFIX .. 'SIZE')
local DCST_RUN = Parameter(PARAM_TABLE_PREFIX .. 'RUN')

if DroneCAN_Handle == nil then
   gcs:send_text(MAV_SEVERITY.ERROR, "DCST: no DroneCAN_Handle")
   return
end

local handle = DroneCAN_Handle(0, STREAM_SIGNATURE, STREAM_DATA_TYPE, true)

local payload = nil
local payload_size = 0
local sent = 0
local last_report_ms = millis()

local function update()
   if DCST_RUN:get() <= 0 then
      payload = nil
      return update, 200
   end

   local want = math.floor(DCST_SIZE:get())
   if want < 1 then want = 1 end
   if want > 1000 then want = 1000 end
   if payload == nil or want ~= payload_size then
      payload_size = want
      payload = string.rep("A", payload_size)
      sent = 0
      last_report_ms = millis()
      gcs:send_text(MAV_SEVERITY.INFO,
                    string.format("DCST: streaming %dB", payload_size))
   end

   -- offer until the driver stops taking them
   for _ = 1, MAX_BURST do
      if not handle:broadcast(payload) then
         break
      end
      sent = sent + 1
   end

   local now = millis()
   if (now - last_report_ms):toint() >= 2000 then
      local dt = (now - last_report_ms):tofloat() * 0.001
      gcs:send_text(MAV_SEVERITY.INFO,
                    string.format("DCST: %dB x%d %.0f/s %.1fkB/s",
                                  payload_size, sent, sent / dt,
                                  sent * payload_size / dt / 1000))
      sent = 0
      last_report_ms = now
   end
   return update, 1
end

gcs:send_text(MAV_SEVERITY.INFO, "DCST_stream loaded")

return update, 1000
