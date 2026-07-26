--[[
   measure what limits AP_DroneCAN_FileClient

   fetches the same thing repeatedly so the rate can be compared with
   and without a filesystem at either end:

     DCBM_NODE  node to fetch from, cleared when the fetch ends
     DCBM_MODE  0 writes the file locally, 1 discards as it arrives
     DCBM_SRC   0 fetches REMOTE_PATH, 1 fetches a RAM backed file

   REMOTE_PATH below names a log on the far node and will need editing
   for whatever is on yours.  This is a measurement tool, not an
   applet: see dronecan_bench.md beside it
--]]

local MAV_SEVERITY = {ERROR=3, INFO=6}

local PARAM_TABLE_KEY = 120
local PARAM_TABLE_PREFIX = "DCBM_"

local REMOTE_PATH = "/APM/LOGS/00000007.BIN"
-- RAM backed on the far node, so fetching it takes that node's SD card
-- out of the measurement.  Small, so it is fetched repeatedly
local RAM_PATH = "@SYS/storage.bin"
local RAM_REPS = 40
local LOCAL_PATH = "dcbm_fetched.BIN"

local STATE = {IDLE=0, RUNNING=1, DONE=2, FAILED=3}

param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3)
assert(param:add_param(PARAM_TABLE_KEY, 1, 'NODE', 0), 'could not add DCBM_NODE')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'MODE', 0), 'could not add DCBM_MODE')
assert(param:add_param(PARAM_TABLE_KEY, 3, 'SRC', 0), 'could not add DCBM_SRC')
local DCBM_NODE = Parameter(PARAM_TABLE_PREFIX .. 'NODE')
local DCBM_MODE = Parameter(PARAM_TABLE_PREFIX .. 'MODE')
local DCBM_SRC = Parameter(PARAM_TABLE_PREFIX .. 'SRC')

if dronecan_file == nil then
   gcs:send_text(MAV_SEVERITY.ERROR, "DCBM: no dronecan_file binding")
   return
end

local running = false
local start_ms = nil
local discard = false
local from_ram = false
local reps_left = 0
local total_bytes = 0
local node_id = 0

local function begin_one()
   local dest = discard and "" or LOCAL_PATH
   return dronecan_file:start(node_id, from_ram and RAM_PATH or REMOTE_PATH, dest)
end

local function update()
   if not running then
      local node = math.floor(DCBM_NODE:get())
      if node > 0 and node < 128 then
         node_id = node
         discard = DCBM_MODE:get() > 0
         from_ram = DCBM_SRC:get() > 0
         reps_left = from_ram and RAM_REPS or 1
         total_bytes = 0
         if begin_one() then
            running = true
            start_ms = millis()
         else
            gcs:send_text(MAV_SEVERITY.ERROR, "DCBM: start refused")
            DCBM_NODE:set(0)
         end
      end
      return update, 200
   end

   if dronecan_file:get_status() == STATE.RUNNING then
      return update, 20
   end

   total_bytes = total_bytes + dronecan_file:get_bytes():toint()
   reps_left = reps_left - 1
   if reps_left > 0 and begin_one() then
      return update, 20
   end

   running = false
   DCBM_NODE:set(0)
   local elapsed = (millis() - start_ms):tofloat() * 0.001
   gcs:send_text(MAV_SEVERITY.INFO,
                 string.format("DCBM: %s%s %dB %.1fs %.1fkB/s",
                               from_ram and "ram/" or "sd/",
                               discard and "discard" or "write", total_bytes, elapsed,
                               elapsed > 0 and (total_bytes / elapsed / 1000) or 0))
   return update, 200
end

gcs:send_text(MAV_SEVERITY.INFO, "DCBM_bench loaded")

return update, 1000
