#!/usr/bin/env python

'''
Run a dronekit-python script
'''

from __future__ import print_function

import pysim
import re
import subprocess

def progress(message):
    print("DK: %s\n" % message)

def run_dronekit_script(binary,
                        step,
                        defaults_file=None,
                        gdb=False,
                        gdbserver=False,
                        model=None,
                        speedup=10,
                        use_map=False,
                        valgrind=False,
                        viewerip=None):

    if binary is None:
        progress("No binary")
        return False

    dronekit_match = re.match("dronekit:([^:]*):(.*)", step)
    if dronekit_match is None:
        progress("Bad dronekit step (%s)" % dronekit_match)
        return False

    vehicle = dronekit_match.group(1)
    script = dronekit_match.group(2)

    if model is None:
        model = '+'

    vinfo = pysim.vehicleinfo.VehicleInfo()
    if defaults_file is None:
        defaults_file = vinfo.options["ArduCopter"]["frames"][model]["default_params_filename"]

    sitl = pysim.util.start_SITL(binary,
                                 model=model,
                                 speedup=speedup,
                                 valgrind=valgrind,
                                 gdb=gdb,
                                 gdbserver=gdbserver,
                                 defaults_file=defaults_file
    )
    try:
        # FIXME: should be able to do better than this static string:
        cmd = ["python", script, '--connect', 'tcp:127.0.0.1:5760']
        retcode = subprocess.call(cmd)
    except subprocess.CalledProcessError as e:
        progress("Failed with timeout")
        failed = True
        fail_list.append("timeout")
        util.pexpect_close(sitl)
        return False

    pysim.util.pexpect_close(sitl)

    return retcode == 0

