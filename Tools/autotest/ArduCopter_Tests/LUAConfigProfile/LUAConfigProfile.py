'''
Copter autotest: LUAConfigProfile

AP_FLAKE8_CLEAN
'''

import pathlib
import re

from test_discovery import TestGroup
from test_discovery import testcase


class LUAConfigProfile(TestGroup):
    @testcase(bucket="CopterTests2c")
    def LUAConfigProfile(self):
        '''test the config_profiles.lua example script'''
        self.customise_SITL_commandline(
            [],
            **self.callisto_sitl_kwargs()
        )
        self.install_mavlink_module_context("MAVLink")
        self.install_example_script_context("config_profiles.lua")
        self.set_parameters({
            'SCR_ENABLE': 1,
        })
        self.context_push()

        self.context_push()
        self.context_collect('STATUSTEXT')
        self.reboot_sitl()
        self.assert_prearm_failure('CFG: Reboot needed for config change', other_prearm_failures_fatal=False)
        self.context_pop()
        self.reboot_sitl()
        self.wait_ready_to_arm()

        self.set_parameter("CFG_PAY_SEL", 1)
        self.wait_parameter_values({
            "GRIP_ENABLE": 1,
            "RC8_OPTION": 19,
        })
        self.set_parameter("CFG_PAY_SEL", 3)  # avt gimbal
        self.wait_parameter_values({
            "GRIP_ENABLE": 0,
            "RC8_OPTION": 0,
        })
        self.set_parameter("CFG_PAY_SEL", 2)  # hook slung
        self.wait_parameter_values({
            "GRIP_ENABLE": 1,
            "RC8_OPTION": 19,
        })
        self.progress("Apply default parameters")
        self.set_parameter("CFG_PAY_SEL", 0)
        self.wait_parameter_values({
            "GRIP_ENABLE": 0,
            "RC8_OPTION": 0,
        })

        self.set_parameter("CFG_PAY_SEL", 22)
        self.wait_statustext("Invalid profile selected for PAY")

        self.start_subtest("Test the different modes (nothing / apply / defaults)")

        self.context_pop()

        self.start_subtest("Testing of different validations")

        def set_config_profiles_config_domains(new_domains):
            path = pathlib.Path(self.installed_script_path("config_profiles.lua"))
            content = path.read_text()

            start_marker = '-- This is a marker for the start of the config_domains; it is used to swap these out for CI testing'  # noqa: E501
            end_marker = '-- This is a marker for the end of the config_domains; it is used to swap these out for CI testing'  # noqa: E501
            pattern = re.compile(
                rf"({re.escape(start_marker)})(.*?){re.escape(end_marker)}",
                re.DOTALL
            )

            if not pattern.search(content):
                raise ValueError("Start or end marker not found in file")

            replaced = pattern.sub(rf"\1\n{new_domains}\n{end_marker}", content)
            path.write_text(replaced)

        def assert_prearm_failure_for_domains_string(domains_string, prearm_failure_reason):
            self.start_subtest(f"Testing domains for ({prearm_failure_reason})")
            set_config_profiles_config_domains(domains_string)
            self.context_push()
            self.context_collect('STATUSTEXT')
            self.reboot_sitl()  # can't just restart scripting as we reuse indexes for different parameters
            # self.wait_statustext('config_profiles .* starting', regex=True, check_context=True)
            self.assert_prearm_failure(prearm_failure_reason, other_prearm_failures_fatal=False, timeout=120)
            self.context_pop()

        domain_param_not_in_defaults = """
-- a table of param indexes to help avoid annoying conflicts:
local param_index = {
   ["enable"]    =  1,
   ["pm_filter"] =  2,

   ["DMN_A"]       = 10,
}

local config_domains = {
   DOMAIN_A = {
      param_name = "DMN_A",
      param_sel_index = 3,
      all_param_defaults = {  -- all parameters present in the params for each option
      },
      profiles = {
         [1] = {
            name = "Param Not in defaults",
            params = {
               ["ACRO_BAL_PITCH"] = 1,
            },
         },
      },
   }
}
"""
        assert_prearm_failure_for_domains_string(domain_param_not_in_defaults, "Not in defaults")

        domain_param_not_in_defaults = """
local param_index = {
   ["enable"]    =  1,
   ["pm_filter"] =  2,

   ["DMN_A"]       = 10,
   ["DMN_B"]       = 11,
}

local config_domains = {
   DOMAIN_A = {
      param_name = "DMN_A",
      param_sel_index = 5,
      all_param_defaults = {  -- all parameters present in the params for each option
        ["ACRO_BAL_PITCH"] = 1,
      },
      profiles = {
         [1] = {
            name = "exists in two domains",
            params = {
               ["ACRO_BAL_PITCH"] = 1,
            },
         },
      },
   },
   DOMAIN_B = {
      param_name = "DMN_B",
      param_sel_index = 3,
      all_param_defaults = {  -- all parameters present in the params for each option
        ["ACRO_BAL_PITCH"] = 1,
      },
      profiles = {
         [1] = {
            name = "param in two domains",
            params = {
               ["ACRO_BAL_PITCH"] = 1,
            },
         },
      },
   },
}

"""
        assert_prearm_failure_for_domains_string(domain_param_not_in_defaults, "CFG: ACRO_BAL_PITCH exists in two")

        domain_profile_param_same_as_domain_default_param_value = """
local param_index = {
   ["enable"]    =  1,
   ["pm_filter"] =  2,

   ["DMN_A"]       = 10,
}

local config_domains = {
   DOMAIN_A = {
      param_name = "DMN_A",
      param_sel_index = 5,
      all_param_defaults = {  -- all parameters present in the params for each option
        ["BATT_CAPACITY"] = 6,
      },
      profiles = {
         [1] = {
            name = "param value same as default",
            params = {
               ["BATT_CAPACITY"] = 6,
            },
         },
      },
   },
}

"""
        assert_prearm_failure_for_domains_string(
            domain_profile_param_same_as_domain_default_param_value,
            "has the same value as"
        )

        domain_profile_param_must_be_set_when_mode_is_apply_defaults = """
local param_index = {
   ["enable"]    =  1,
   ["pm_filter"] =  2,

   ["DMN_A"]       = 10,
}

local config_domains = {
   DOMAIN_A = {
      param_name = "DMN_A",
      param_sel_index = 0,
      all_param_defaults = {  -- all parameters present in the params for each option
        ["BATT_CAPACITY"] = must_be_set,
      },
      profiles = {
         [1] = {
            name = "param must be set",
            params = {
               ["BATT_CAPACITY"] = 6,
            },
         },
      },
   },
}

"""
        assert_prearm_failure_for_domains_string(
            domain_profile_param_must_be_set_when_mode_is_apply_defaults,
            "BATT_CAPACITY in DMN_A is must-be-set but al"
        )
