'''
Rover autotest: ScriptingLocationBindings

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptingLocationBindings(TestGroup):
    @testcase(bucket="Rover")
    def ScriptingLocationBindings(self):
        '''test scripting bindings'''
        self.install_script_content_context("test-scripting-bindings.lua", """
function main()
  here = ahrs:get_location()
  if here == nil then
     gcs:send_text(0, "Need location")
     return
  end
  if here.alt == 0 then
     gcs:send_text(0, "zero altitude?!")
     return
  end
  ALT_FRAME = { ABSOLUTE = 0, ABOVE_HOME = 1, ABOVE_ORIGIN = 2, ABOVE_TERRAIN = 3 }
  here:set_alt_m(12.34, ALT_FRAME.ABSOLUTE)
  want = 1234
  if here:alt() ~= want then
     gcs:send_text(0, string.format("Bad altitude" .. here:alt() .. " want " .. want))
     return
  end
  gcs:send_text(0, "Tests OK")
end
function update()
  main()
  return update, 1000
end

return update()
""")
        self.set_parameter('SCR_ENABLE', 1)
        self.reboot_sitl()
        self.wait_statustext('Tests OK')
