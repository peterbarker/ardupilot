'''
Rover autotest: NetworkingWebServer

AP_FLAKE8_CLEAN
'''

import os

from pysim import util
from vehicle_test_suite import NotAchievedException

from test_discovery import TestGroup
from test_discovery import testcase


class NetworkingWebServer(TestGroup):
    def TestWebServer(self, url):
        '''test active web server'''
        self.progress("Accessing webserver main page")
        import urllib.request

        main_page = urllib.request.urlopen(url).read().decode('utf-8')
        if main_page.find('ArduPilot Web Server') == -1:
            raise NotAchievedException("Expected banner on main page")

        board_status = urllib.request.urlopen(url + '/@DYNAMIC/board_status.shtml').read().decode('utf-8')
        if board_status.find('0 hours') == -1:
            raise NotAchievedException("Expected uptime in board status")
        if board_status.find('40.713') == -1:
            raise NotAchievedException("Expected latitude in board status")

        self.progress("WebServer tests OK")

    @testcase(bucket="Rover")
    def NetworkingWebServer(self):
        '''web server'''
        applet_script = "net_webserver.lua"

        self.context_push()
        self.install_applet_script_context(applet_script)

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "SIM_SPEEDUP": 20,
            "NET_ENABLE": 1,
        })

        self.reboot_sitl()

        self.context_push()
        self.context_collect('STATUSTEXT')

        self.set_parameters({
            "WEB_BIND_PORT": 8081,
        })

        self.scripting_restart()
        self.wait_text("WebServer: starting on port 8081", check_context=True)

        self.wait_ready_to_arm()

        self.TestWebServer("http://127.0.0.1:8081")

        self.context_pop()
        self.context_pop()
        self.reboot_sitl()

    @testcase(bucket="Rover")
    def NetworkingWebServerPPP(self):
        '''web server over PPP'''
        applet_script = "net_webserver.lua"

        self.context_push()
        self.install_applet_script_context(applet_script)

        self.set_parameters({
            "SCR_ENABLE": 1,
            "SCR_VM_I_COUNT": 1000000,
            "SIM_SPEEDUP": 20,
            "NET_ENABLE": 1,
            "SERIAL5_PROTOCOL": 48,
        })

        self.progress('rebuilding rover with ppp enabled')
        import shutil
        shutil.copy('build/sitl/bin/ardurover', 'build/sitl/bin/ardurover.noppp')
        # --enable-math-check-indexes matches the CI rover build
        # configuration, so in CI this rebuild is a ccache hit against
        # the pre-built PPP rover from the build job
        util.build_SITL('bin/ardurover', clean=False, configure=True,
                        extra_configure_args=['--enable-PPP', '--enable-math-check-indexes', '--debug'])

        self.reboot_sitl()

        # the applet creates its WEB_* parameters at runtime, and they
        # only go away when the vehicle comes up without it; removing the
        # script from disk at context_pop() does not unload the copy
        # already running.  Nothing here has customised the SITL
        # commandline, so ask for the standard reset explicitly.
        self.context_get().sitl_commandline_customised = True

        self.progress("Starting PPP daemon")
        pppd = util.start_PPP_daemon("192.168.14.15:192.168.14.13", '127.0.0.1:5765')

        self.context_push()
        self.context_collect('STATUSTEXT')

        pppd.expect("remote IP address 192.168.14.13")

        self.progress("PPP daemon started")

        self.set_parameters({
            "WEB_BIND_PORT": 8081,
        })

        self.scripting_restart()
        self.wait_text("WebServer: starting on port 8081", check_context=True)

        self.wait_ready_to_arm()

        self.TestWebServer("http://192.168.14.13:8081")

        self.context_pop()
        self.context_pop()

        # restore rover without ppp enabled for next test
        os.unlink('build/sitl/bin/ardurover')
        shutil.copy('build/sitl/bin/ardurover.noppp', 'build/sitl/bin/ardurover')
        self.reboot_sitl()
