--[[
   download dataflash logs from another node on a DroneCAN bus, using
   AP_DroneCAN_FileClient to move the bytes

   DroneCAN_log_download.lua does the same job with the transfer itself
   written in Lua, which is simpler but slower: every byte of every
   reply passes through the interpreter, and only one read is in flight
   at a time.  This script keeps the part which decides what to fetch -
   listing the remote directory, skipping what is already here - and
   leaves the transfer to the C++ client, which pipelines its reads.

   set DCLF_NODE to the node ID to fetch from
--]]

local MAV_SEVERITY = {ERROR=3, WARNING=4, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 119
local PARAM_TABLE_PREFIX = "DCLF_"

local UAVCAN_PROTOCOL_FILE_GETDIRECTORYENTRYINFO_ID = 46

local FLAG_DIRECTORY = 2

local REQUEST_TIMEOUT_MS = 1000
local MAX_RETRIES = 10

-- ChibiOS boards keep logs here, SITL uses a relative path
local LOG_DIR_CANDIDATES = {"/APM/LOGS", "logs", "APM/LOGS"}

-- these match AP_DroneCAN_FileClient::State
local STATE = {IDLE=0, RUNNING=1, DONE=2, FAILED=3}

param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3)

local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
          string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: DCLF_NODE
  // @DisplayName: DroneCAN log fetch node ID
  // @Description: node ID to fetch logs from. Set to the node ID of a DroneCAN node with a file server enabled; reset to zero automatically when the fetch completes
  // @Range: 0 127
  // @User: Standard
--]]
local DCLF_NODE = bind_add_param('NODE', 1, 0)

--[[
  // @Param: DCLF_CANDRV
  // @DisplayName: DroneCAN log fetch CAN driver
  // @Description: DroneCAN driver index to use, 0 is the first driver
  // @Range: 0 1
  // @User: Standard
--]]
local DCLF_CANDRV = bind_add_param('CANDRV', 2, 0)

--[[
  // @Param: DCLF_DEBUG
  // @DisplayName: DroneCAN log fetch debug
  // @Description: enable debug output
  // @Values: 0:Disabled,1:Enabled
  // @User: Advanced
--]]
local DCLF_DEBUG = bind_add_param('DEBUG', 3, 0)

if DroneCAN_Handle == nil or dronecan_file == nil then
   gcs:send_text(MAV_SEVERITY.ERROR, "DCLF: DroneCAN file client not available")
   return
end

local can_driver = math.floor(DCLF_CANDRV:get())
local direntry_handle = DroneCAN_Handle(can_driver,
                                        uint64_t(0x8C46E8AB, 0x568BDA79),
                                        UAVCAN_PROTOCOL_FILE_GETDIRECTORYENTRYINFO_ID)

local target_node = 0
local pending = nil
local remote_dir = nil
local dir_candidate_idx = 1
local list_index = 0
local file_queue = {}
local current = nil
local stats = nil

local function debug(msg)
   if DCLF_DEBUG:get() > 0 then
      gcs:send_text(MAV_SEVERITY.DEBUG, "DCLF: " .. msg)
   end
end

local function basename(path)
   return path:match("([^/]+)$") or path
end

local function local_name_for(remote_path)
   -- a prefix of our own, so this and DroneCAN_log_download.lua can
   -- both be installed and fetch the same file for comparison
   return string.format("dcf_%u_%s", target_node, basename(remote_path))
end

local function start_request(handle, payload, on_reply, on_fail)
   pending = {
      handle = handle,
      payload = payload,
      on_reply = on_reply,
      on_fail = on_fail,
      retries = 0,
      sent_ms = millis(),
   }
   handle:request(target_node, payload)
end

local function poll_pending()
   if pending == nil then
      return
   end
   local payload, nodeid = pending.handle:check_message()
   if payload ~= nil and nodeid == target_node then
      local p = pending
      pending = nil
      p.on_reply(payload)
      return
   end
   if millis() - pending.sent_ms > REQUEST_TIMEOUT_MS then
      if pending.retries >= MAX_RETRIES then
         local p = pending
         pending = nil
         p.on_fail()
         return
      end
      pending.retries = pending.retries + 1
      pending.sent_ms = millis()
      pending.handle:request(target_node, pending.payload)
   end
end

local function direntry_request(index, path)
   return string.pack("<I4", index) .. path
end

local function finish()
   local elapsed = (millis() - stats.start_ms):tofloat() * 0.001
   gcs:send_text(MAV_SEVERITY.INFO,
                 string.format("DCLF: node %u done: %d files %d bytes %.1fs",
                               target_node, stats.files, stats.bytes, elapsed))
   target_node = 0
   DCLF_NODE:set(0)
end

local start_next_file

-- watch the C++ client through to the end of the file it is fetching
local function poll_transfer()
   local status = dronecan_file:get_status()
   if status == STATE.RUNNING then
      return
   end
   local n = dronecan_file:get_bytes():toint()
   if status == STATE.DONE then
      local elapsed = (millis() - current.start_ms):tofloat() * 0.001
      -- a STATUSTEXT is fifty characters, so keep this short enough
      -- that the rate is not cut off the end
      gcs:send_text(MAV_SEVERITY.INFO,
                    string.format("DCLF: %s %dB %.1fs %.1fkB/s",
                                  basename(current.path):gsub("%.BIN$", ""), n, elapsed,
                                  elapsed > 0 and (n / elapsed / 1000) or 0))
      stats.files = stats.files + 1
      stats.bytes = stats.bytes + n
   else
      gcs:send_text(MAV_SEVERITY.WARNING,
                    string.format("DCLF: %s failed after %d bytes",
                                  current.local_name, n))
   end
   current = nil
   start_next_file()
end

start_next_file = function()
   if #file_queue == 0 then
      finish()
      return
   end
   local path = table.remove(file_queue, 1)
   local lname = local_name_for(path)
   -- skip anything already here; a fetch is expensive
   local st = io.open(lname, "r")
   if st ~= nil then
      st:close()
      debug(string.format("already have %s", lname))
      start_next_file()
      return
   end
   if not dronecan_file:start(target_node, path, lname) then
      gcs:send_text(MAV_SEVERITY.WARNING, string.format("DCLF: cannot start %s", path))
      start_next_file()
      return
   end
   current = {path = path, local_name = lname, start_ms = millis()}
   debug(string.format("fetching %s", path))
end

local request_next_direntry
local probe_log_directory

request_next_direntry = function()
   start_request(direntry_handle, direntry_request(list_index, remote_dir),
                 function(payload)
                    local err, flags = string.unpack("<i2B", payload)
                    local fullpath = payload:sub(4)
                    if err ~= 0 or flags == 0 then
                       debug(string.format("%u logs to fetch", #file_queue))
                       start_next_file()
                       return
                    end
                    if (flags & FLAG_DIRECTORY) == 0 and fullpath:upper():match("%.BIN$") then
                       table.insert(file_queue, fullpath)
                    end
                    list_index = list_index + 1
                    request_next_direntry()
                 end,
                 function()
                    gcs:send_text(MAV_SEVERITY.WARNING, "DCLF: directory listing timed out")
                    finish()
                 end)
end

probe_log_directory = function()
   if dir_candidate_idx > #LOG_DIR_CANDIDATES then
      gcs:send_text(MAV_SEVERITY.WARNING,
                    string.format("DCLF: no log directory on node %u", target_node))
      finish()
      return
   end
   remote_dir = LOG_DIR_CANDIDATES[dir_candidate_idx]
   dir_candidate_idx = dir_candidate_idx + 1
   list_index = 0
   -- ask for the first entry; a directory which is not there answers
   -- with an error and we move on to the next candidate
   start_request(direntry_handle, direntry_request(0, remote_dir),
                 function(payload)
                    local err, flags = string.unpack("<i2B", payload)
                    if err ~= 0 or flags == 0 then
                       probe_log_directory()
                       return
                    end
                    debug(string.format("using %s", remote_dir))
                    local fullpath = payload:sub(4)
                    if (flags & FLAG_DIRECTORY) == 0 and fullpath:upper():match("%.BIN$") then
                       table.insert(file_queue, fullpath)
                    end
                    list_index = 1
                    request_next_direntry()
                 end,
                 probe_log_directory)
end

local function start_fetch()
   target_node = math.floor(DCLF_NODE:get())
   stats = {start_ms = millis(), files = 0, bytes = 0}
   gcs:send_text(MAV_SEVERITY.INFO,
                 string.format("DCLF: fetching logs from node %u", target_node))
   dir_candidate_idx = 1
   file_queue = {}
   probe_log_directory()
end

local function update()
   if target_node == 0 then
      local node = DCLF_NODE:get()
      if node > 0 and node < 128 then
         start_fetch()
      end
      return update, 500
   end
   if current ~= nil then
      poll_transfer()
   else
      poll_pending()
   end
   return update, 20
end

gcs:send_text(MAV_SEVERITY.INFO, "DroneCAN_log_fetch loaded")

return update, 1000
