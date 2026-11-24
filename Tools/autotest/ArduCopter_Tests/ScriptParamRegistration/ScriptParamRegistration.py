'''
Copter autotest: ScriptParamRegistration

AP_FLAKE8_CLEAN
'''

from test_discovery import TestGroup
from test_discovery import testcase


class ScriptParamRegistration(TestGroup):
    @testcase(bucket="CopterTests1d")
    def ScriptParamRegistration(self):
        '''test parameter script registration'''
        self.set_parameters({
            'SCR_ENABLE': 1,
        })
        script_content = """
local PARAM_TABLE_KEY = 10
local PARAM_TABLE_PREFIX = "TST_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 16), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

local PARAM_A = bind_add_param("A", 5, 22)
local PARAM_B = bind_add_param("B", 1, 33)

function update()
  gcs:send_text(3, string.format("test script running"))
  return update, 1000
end

return update, 1000
"""  # noqa: E501
        self.install_script_content_context("test.lua", script_content)
        self.reboot_sitl()
        self.wait_statustext('test script running')
        self.assert_parameter_value('TST_A', 22)
        self.assert_parameter_value('TST_B', 33)

        all_params = self.fetch_all_parameters()
        for pname in "TST_A", "TST_B":
            if pname not in all_params:
                raise ValueError(f"{pname} not in fetched-all-parameters")

        self.start_subtest("Remove parameter at runtime")
        script_content = """
local PARAM_TABLE_KEY = 10
local PARAM_TABLE_PREFIX = "TST_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 16), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

local PARAM_B = bind_add_param("B", 1, 33)

function update()
  gcs:send_text(3, string.format("test script running"))
  return update, 1000
end

return update, 1000
"""  # noqa: E501
        self.install_script_content_context("test.lua", script_content)
        self.scripting_restart()
        self.wait_statustext('restart')
        self.wait_statustext('test script running')
        self.assert_parameter_value('TST_B', 33)

        all_params = self.fetch_all_parameters()
        for pname in ["TST_B"]:
            if pname not in all_params:
                raise ValueError(f"{pname} not in fetched-all-parameters")
        for pname in ["TST_A"]:
            if pname in all_params:
                raise ValueError(f"{pname} in fetched-all-parameters when it should have gone away")
