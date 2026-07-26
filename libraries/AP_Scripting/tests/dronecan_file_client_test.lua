--[[
   drive AP_DroneCAN_FileClient, which does the transfer in C++ rather
   than moving every byte through this interpreter

   set DCFC_NODE to the node to fetch from.  DCFC_CASE picks what to
   ask for, so that the paths which end in a failure can be driven as
   well as the one which succeeds
--]]

local MAV_SEVERITY = {ERROR=3, INFO=6}

local PARAM_TABLE_KEY = 118
local PARAM_TABLE_PREFIX = "DCFC_"

local LOCAL_PATH = "dcfc_fetched.BIN"

-- what DCFC_CASE selects
local CASES = {
   [0] = "logs/00000042.BIN",   -- the ordinary one
   [1] = "logs/nosuch.BIN",     -- the node answers with an error
   [2] = "logs/empty.BIN",      -- nothing to fetch at all
   [3] = "logs/aligned.BIN",    -- ends exactly on a read boundary
}

-- these match AP_DroneCAN_FileClient::State
local STATE = {IDLE=0, RUNNING=1, DONE=2, FAILED=3}

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'NODE', 0), 'could not add DCFC_NODE')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'CASE', 0), 'could not add DCFC_CASE')
local DCFC_NODE = Parameter(PARAM_TABLE_PREFIX .. 'NODE')
local DCFC_CASE = Parameter(PARAM_TABLE_PREFIX .. 'CASE')

if dronecan_file == nil then
   gcs:send_text(MAV_SEVERITY.ERROR, "DCFC: no dronecan_file binding")
   return
end

local running = false
local start_ms = nil
local last_report_ms = nil

local function update()
   if not running then
      local node = math.floor(DCFC_NODE:get())
      if node > 0 and node < 128 then
         local path = CASES[math.floor(DCFC_CASE:get())] or CASES[0]
         if dronecan_file:start(node, path, LOCAL_PATH) then
            gcs:send_text(MAV_SEVERITY.INFO,
                          string.format("DCFC: fetching from node %u", node))
            running = true
            start_ms = millis()
         else
            gcs:send_text(MAV_SEVERITY.ERROR, "DCFC: start refused")
            DCFC_NODE:set(0)
         end
      end
      return update, 200
   end

   local status = dronecan_file:get_status()
   if status == STATE.RUNNING then
      local now = millis()
      if last_report_ms == nil or (now - last_report_ms):toint() > 1000 then
         last_report_ms = now
         gcs:send_text(MAV_SEVERITY.INFO,
                       string.format("DCFC: %d bytes", dronecan_file:get_bytes():toint()))
      end
      return update, 50
   end

   running = false
   DCFC_NODE:set(0)
   local elapsed = (millis() - start_ms):tofloat() * 0.001
   if status == STATE.DONE then
      gcs:send_text(MAV_SEVERITY.INFO,
                    string.format("DCFC: done %d bytes %.2fs",
                                  dronecan_file:get_bytes():toint(), elapsed))
   else
      gcs:send_text(MAV_SEVERITY.ERROR,
                    string.format("DCFC: failed after %d bytes",
                                  dronecan_file:get_bytes():toint()))
   end
   return update, 200
end

gcs:send_text(MAV_SEVERITY.INFO, "dronecan_file_client_test loaded")

return update, 1000
