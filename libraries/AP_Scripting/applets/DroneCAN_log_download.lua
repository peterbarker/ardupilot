--[[
   download dataflash logs from another node on a DroneCAN bus

   this is a client for the uavcan.protocol.file services (GetInfo,
   GetDirectoryEntryInfo and Read), as served by ArduPilot nodes with
   the DroneCAN file server enabled (CAN_Dn_UC_OPTION bit
   EnableFileServer on main firmwares, AP_PERIPH_FILE_SERVER_ENABLED
   on AP_Periph)

   set DCLD_NODE to the node ID to download from; the script fetches
   any .BIN logs from the remote node's log directory which are not
   already present locally, saving them as dcl_<node>_<name>, then
   sets DCLD_NODE back to zero
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 117
local PARAM_TABLE_PREFIX = "DCLD_"

local UAVCAN_PROTOCOL_FILE_GETINFO_ID = 45
local UAVCAN_PROTOCOL_FILE_GETDIRECTORYENTRYINFO_ID = 46
local UAVCAN_PROTOCOL_FILE_READ_ID = 48

local FLAG_DIRECTORY = 2

local READ_CHUNK_SIZE = 256
local REQUEST_TIMEOUT_MS = 250
local MAX_RETRIES = 10

-- candidate log directories on the remote node; ChibiOS boards use
-- /APM/LOGS, SITL uses logs
local LOG_DIR_CANDIDATES = {"/APM/LOGS", "logs", "APM/LOGS"}

param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3)

local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: DCLD_NODE
  // @DisplayName: DroneCAN log download node ID
  // @Description: node ID to download logs from. Set to the node ID of a DroneCAN node with a file server enabled; reset to zero automatically when the download completes
  // @Range: 0 127
  // @User: Standard
--]]
local DCLD_NODE = bind_add_param('NODE', 1, 0)

--[[
  // @Param: DCLD_CANDRV
  // @DisplayName: DroneCAN log download CAN driver
  // @Description: DroneCAN driver index to use, 0 is the first driver
  // @Range: 0 1
  // @User: Standard
--]]
local DCLD_CANDRV = bind_add_param('CANDRV', 2, 0)

--[[
  // @Param: DCLD_DEBUG
  // @DisplayName: DroneCAN log download debug
  // @Description: enable debug output
  // @Values: 0:Disabled,1:Enabled
  // @User: Advanced
--]]
local DCLD_DEBUG = bind_add_param('DEBUG', 3, 0)

if DroneCAN_Handle == nil then
   gcs:send_text(MAV_SEVERITY.ERROR, "DCLD: DroneCAN_Handle not available")
   return
end

local can_driver = math.floor(DCLD_CANDRV:get())
local getinfo_handle = DroneCAN_Handle(can_driver, uint64_t(0x5004891E, 0xE8A27531), UAVCAN_PROTOCOL_FILE_GETINFO_ID)
local direntry_handle = DroneCAN_Handle(can_driver, uint64_t(0x8C46E8AB, 0x568BDA79), UAVCAN_PROTOCOL_FILE_GETDIRECTORYENTRYINFO_ID)
local read_handle = DroneCAN_Handle(can_driver, uint64_t(0x8DCDCA93, 0x9F33F678), UAVCAN_PROTOCOL_FILE_READ_ID)

local target_node = 0

-- outstanding request, nil when idle
local pending = nil

local function debug(msg)
   if DCLD_DEBUG:get() > 0 then
      gcs:send_text(MAV_SEVERITY.DEBUG, "DCLD: " .. msg)
   end
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

--[[
   check the outstanding request for a reply or timeout, invoking its
   callbacks as appropriate
--]]
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

-- request encoders.  DSDL tail array optimisation applies to the
-- trailing Path, so it is sent with no length prefix
local function getinfo_request(path)
   return path
end

local function direntry_request(index, path)
   return string.pack("<I4", index) .. path
end

local function read_request(offset, path)
   return string.pack("<I5", offset) .. path
end

-- download state
local remote_dir = nil
local dir_candidate_idx = 1
local list_index = 0
local file_queue = {}
local current = nil    -- {path=, name=, size=, local_name=, fd=, offset=}
local stats = nil

local function basename(path)
   return path:match("([^/]+)$") or path
end

local function local_name_for(remote_path)
   return string.format("dcl_%u_%s", target_node, basename(remote_path))
end

-- returns the size of a local file, or nil if it does not exist
local function local_file_size(name)
   local f = io.open(name, "rb")
   if f == nil then
      return nil
   end
   local size = f:seek("end")
   f:close()
   return size
end

local function finish_all()
   if stats ~= nil then
      local elapsed_s = (millis() - stats.start_ms):tofloat() * 0.001
      gcs:send_text(MAV_SEVERITY.INFO, string.format("DCLD: node %u done: %u files %u bytes %.1fs",
                                                     target_node, stats.files, stats.bytes, elapsed_s))
   end
   remote_dir = nil
   dir_candidate_idx = 1
   list_index = 0
   file_queue = {}
   current = nil
   stats = nil
   pending = nil
   target_node = 0
   DCLD_NODE:set(0)
end

local function abort_current_file(msg)
   gcs:send_text(MAV_SEVERITY.WARNING, string.format("DCLD: %s: %s", current and current.name or "?", msg))
   if current ~= nil and current.fd ~= nil then
      current.fd:close()
      remove(current.local_name)
   end
   current = nil
end

local start_next_file  -- forward declaration

local function send_next_read()
   local c = current
   start_request(read_handle, read_request(c.offset, c.path),
                 function(payload)
                    local err = string.unpack("<i2", payload)
                    if err ~= 0 then
                       abort_current_file(string.format("read error %d at offset %u", err, c.offset))
                       start_next_file()
                       return
                    end
                    local data = payload:sub(3)
                    c.fd:write(data)
                    c.offset = c.offset + #data
                    stats.bytes = stats.bytes + #data
                    if #data < READ_CHUNK_SIZE then
                       -- short read signals EOF
                       c.fd:close()
                       c.fd = nil
                       stats.files = stats.files + 1
                       gcs:send_text(MAV_SEVERITY.INFO, string.format("DCLD: saved %s (%u bytes)", c.local_name, c.offset))
                       current = nil
                       start_next_file()
                       return
                    end
                    send_next_read()
                 end,
                 function()
                    abort_current_file("read timed out")
                    start_next_file()
                 end)
end

start_next_file = function()
   if #file_queue == 0 then
      finish_all()
      return
   end
   local path = table.remove(file_queue, 1)
   -- fetch the size first so we can skip files we already hold
   start_request(getinfo_handle, getinfo_request(path),
                 function(payload)
                    local size, err, flags = string.unpack("<I5i2B", payload)
                    if err ~= 0 then
                       gcs:send_text(MAV_SEVERITY.WARNING, string.format("DCLD: GetInfo error %d for %s", err, path))
                       start_next_file()
                       return
                    end
                    local lname = local_name_for(path)
                    if local_file_size(lname) == size then
                       debug(string.format("already have %s", lname))
                       start_next_file()
                       return
                    end
                    local fd = io.open(lname, "wb")
                    if fd == nil then
                       gcs:send_text(MAV_SEVERITY.WARNING, string.format("DCLD: cannot open %s", lname))
                       start_next_file()
                       return
                    end
                    current = {
                       path = path,
                       name = basename(path),
                       size = size,
                       local_name = lname,
                       fd = fd,
                       offset = 0,
                    }
                    debug(string.format("downloading %s (%u bytes)", path, size))
                    send_next_read()
                 end,
                 function()
                    gcs:send_text(MAV_SEVERITY.WARNING, string.format("DCLD: GetInfo timed out for %s", path))
                    start_next_file()
                 end)
end

local function request_next_direntry()
   start_request(direntry_handle, direntry_request(list_index, remote_dir),
                 function(payload)
                    local err, flags = string.unpack("<i2B", payload)
                    local fullpath = payload:sub(4)
                    if err ~= 0 or flags == 0 then
                       -- end of directory
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
                    gcs:send_text(MAV_SEVERITY.WARNING, "DCLD: directory listing timed out")
                    finish_all()
                 end)
end

local function probe_log_directory()
   if dir_candidate_idx > #LOG_DIR_CANDIDATES then
      gcs:send_text(MAV_SEVERITY.WARNING, string.format("DCLD: no log directory found on node %u", target_node))
      finish_all()
      return
   end
   local dir = LOG_DIR_CANDIDATES[dir_candidate_idx]
   dir_candidate_idx = dir_candidate_idx + 1
   start_request(getinfo_handle, getinfo_request(dir),
                 function(payload)
                    local _, err, flags = string.unpack("<I5i2B", payload)
                    if err == 0 and (flags & FLAG_DIRECTORY) ~= 0 then
                       remote_dir = dir
                       debug(string.format("using log directory %s", dir))
                       list_index = 0
                       file_queue = {}
                       request_next_direntry()
                       return
                    end
                    probe_log_directory()
                 end,
                 probe_log_directory)
end

local function start_download()
   target_node = math.floor(DCLD_NODE:get())
   stats = {start_ms=millis(), files=0, bytes=0}
   gcs:send_text(MAV_SEVERITY.INFO, string.format("DCLD: fetching logs from node %u", target_node))
   dir_candidate_idx = 1
   probe_log_directory()
end

local function update()
   if target_node == 0 then
      local node = DCLD_NODE:get()
      if node > 0 and node < 128 then
         start_download()
      end
      return update, 500
   end
   poll_pending()
   -- run quickly while a transfer is in progress; each Read is a
   -- request/response round trip
   return update, 2
end

gcs:send_text(MAV_SEVERITY.INFO, "DroneCAN_log_download loaded")

return update, 1000
