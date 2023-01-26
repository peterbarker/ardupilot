-- This script runs a custom arming check.  It returns false
--   initially, and then updates the value based on the value of
--   AUX_FUNC::SCRIPTING1 auxiliary function.

auth_id = arming:get_aux_auth_id()

RC_OPTION = 300
reschedule_interval_ms = 1000

function update() -- this is the loop which periodically runs
  if (auth_id == nil) then
    gcs:send_text(6, 'LUA:Failed to retrieve auth_id')  -- MAV_SEVERITY_INFO
    return update, 10000
  end

  local rcin_select = rc:get_aux_cached(RC_OPTION)
  if (rcin_select == 0) then
    arming:set_aux_auth_failed(auth_id, "External system denies arming")
    return update, reschedule_interval_ms
  end					
     
  if (rcin_select == 2) then
    arming:set_aux_auth_passed(auth_id)
  end

  return update, reschedule_interval_ms
end

return update() -- run immediately before starting to reschedule
