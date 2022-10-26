import argparse
import logging
import sys
import time
from pathlib import Path
import traceback
import os

import psutil
import yaml
from common.params import Params, ParamKeyType
from common.basedir import BASEDIR
from common import system
import cereal.messaging as messaging

from flowinit.config import Config
from flowinit.daemon import Daemon, DaemonSig
from flowinit.filelock import FileLock
from flowinit.utils import get_cpu_times, get_device_state_msg, get_memory_logs
from flowinit.services import Service, killswitch

from selfdrive.version import is_dirty, get_commit, get_version, get_origin, get_short_branch, \
                              terms_version, training_version
from selfdrive.swaglog import cloudlog
from flowinit.process_config import managed_processes

logger = logging.getLogger(__name__)
os.chdir(BASEDIR)

POSSIBLE_PNAME_MATRIX = [
    "java",  # linux
    "ai.flow.android",  # android
    "java.exe",  # windows
]
ANDROID_APP = "ai.flow.app"
ENV_VARS = ["USE_CUDA", "USE_PARAMS_CLIENT", "ZMQ_MESSAGING_PROTOCOL", "ZMQ_MESSAGING_ADDRESS"
            "USE_VIDEO_STREAM", "SIMULATION", "FINGERPRINT", "MSGQ", "PASSIVE"]

params = Params()

def flowpilot_running():
    logger.debug("Checking if flowpilot is running")
    ret = False
    
    pid_bytes = params.get("FlowpilotPID")
    pid = int.from_bytes(pid_bytes, "little") if pid_bytes is not None else None
    if pid is not None and psutil.pid_exists(pid):
        p = psutil.Process(pid)
        if p.name() in POSSIBLE_PNAME_MATRIX:
            logger.debug("flowpilot is running")
            ret = True

    return ret


def wait_for_start_signal(daemon):
    logger.info("Waiting for the start signal")
    while (not flowpilot_running()) and daemon.recv() != DaemonSig.START:
        time.sleep(Config.FREQUENCY)
    logger.debug("Got the start signal")


def parse_services(service_file):
    """Parses the service.yaml file and returns a list of Services"""

    services: list[Service] = []
    nomonitor_services: list[Service] = []

    platform = "android" if system.is_android() else "desktop" 

    with open(service_file) as f:
        parsed = yaml.safe_load(f)

        for sname, sdata in parsed["services"].items():
            target_platforms = sdata.get("platforms", None)
            if target_platforms is not None:
                if platform not in target_platforms:
                    continue

            service = Service(
                    sname,
                    sdata["command"],
                    sdata.get("args", []),
                    sdata.get("restart", False),
                    nomonitor=sdata.get("nomonitor", False),
                    nowait = sdata.get("nowait", False)
                )
            if sdata.get("nomonitor", False):
                nomonitor_services.append(service)
            else:
                services.append(service)
    return services, nomonitor_services


def parse_args(args):
    """Argument Parsing"""
    parser = argparse.ArgumentParser(
        description="A lightweight init system for flowpilot"
    )

    parser.add_argument(
        "-c",
        "--config",
        action="store",
        type=str,
        default=f"{Config.FLOWINIT_ROOT}/services.yaml",
        help="Path to the service yaml file",
    )

    parser.add_argument(
        "-o",
        "--logpath",
        action="store",
        type=str,
        default=Config.LOGPATH,
        help="Path to where the stdout will be redirected",
    )

    parser.add_argument(
        "--logfile",
        action="store_true",
        default=False,
        help="Print STDOUT to logfile",
    )

    parser.add_argument(
        "-v",
        "--verbose",
        action="count",
        default=0,
        help="Increase logging level. Defaults to logging.INFO",
    )

    return parser.parse_args(args)

def append_extras(command: str):
    for var in ENV_VARS:
        val = os.environ.get(var, None)
        if val is not None:
            command += f" -e '{var}' '{val}'"
    return command

def main():
    params.clear_all(ParamKeyType.CLEAR_ON_MANAGER_START)
    with FileLock("flowinit"):     
        cloudlog.info(f"Got services: \n {managed_processes.keys()}")

        for proc in psutil.process_iter():
            if proc.name() in managed_processes.keys():
                cloudlog.info(f"{proc.name()} already alive, restarting..")
                proc.kill()
            
        for p in managed_processes.values(): 

            # explicitly need to pass env variables to android app through extras.
            if p.name == ANDROID_APP:
                managed_processes[service].cmdline = append_extras(managed_processes[service].cmdline)

            if p.nowait:
                p.start()

        default_params = [
                        ("CompletedTrainingVersion", "0"),
                        ("DisengageOnAccelerator", "1"),
                        ("HasAcceptedTerms", "0"),
                        ("OpenpilotEnabledToggle", "1"),
                         ]

        if params.get_bool("RecordFrontLock"):
            params.put_bool("RecordFront", True)

        if not params.get_bool("DisableRadar_Allow"):
            params.delete("DisableRadar")

        for k, v in default_params:
            if params.get(k) is None:
                params.put(k, v)
        
        # is this dashcam?
        if os.getenv("PASSIVE") is not None:
            params.put_bool("Passive", bool(int(os.getenv("PASSIVE", "0"))))

        if params.get("Passive") is None:
            raise Exception("Passive must be set to continue")

        # set version params
        params.put("Version", get_version())
        params.put("TermsVersion", terms_version)
        params.put("TrainingVersion", training_version)
        params.put("GitCommit", get_commit(default=""))
        params.put("GitBranch", get_short_branch(default=""))
        params.put("GitRemote", get_origin(default=""))

        if not is_dirty():
            os.environ['CLEAN'] = '1'

        cloudlog.bind_global(dongle_id="", version=get_version(), dirty=is_dirty(), # TODO
                            device="todo")

        # Get a daemon instance
        daemon = Daemon()

        # Get the green flag from the FlowPilot app to start the services
        wait_for_start_signal(daemon)

        # on android without root, we cannot monitor external processes
        # flowpilot_pid = int.from_bytes(params.get("FlowpilotPID"), "little")
        # if psutil.pid_exists(flowpilot_pid):
        #     flowpilot_process = Service("modeld camerad ui sensord", pid=flowpilot_pid,
        #                                 monitor_only=True)
        #     services.append(flowpilot_process)
        # else:
        #     cloudlog.warning("Unable to monitor flowpilot app (modeld camerad ui sensord)")

        try:
            # Start all services
            for service in managed_processes.values():
                service.start()

            pm = messaging.PubMaster(["procLog", "deviceState", "managerState"])
            params.put_bool("FlowinitReady", True)

            # Event loop
            while True:            

                running = ' '.join("%s%s\u001b[0m" % ("\u001b[32m" if p.proc.is_alive() else "\u001b[31m", p.name)
                       for p in managed_processes.values())

                print(running)
                cloudlog.debug(running)

                # send managerState
                manager_state_msg = messaging.new_message('managerState')
                manager_state_msg.managerState.processes = [p.get_process_state_msg() for p in managed_processes.values()]
                pm.send('managerState', manager_state_msg)

                # Generate a new procLogList Message
                proc_log_msg = messaging.new_message("procLog")
                proc_log_msg.procLog.procs = [p.get_proc_msg() for p in managed_processes.values()]
                proc_log_msg.procLog.cpuTimes = get_cpu_times()
                proc_log_msg.procLog.mem = get_memory_logs()

                device_state_msg = get_device_state_msg()

                # Publishing topics
                pm.send("procLog", proc_log_msg)
                pm.send("deviceState", device_state_msg)

                # Try not to hog all the CPU cycles
                time.sleep(Config.FREQUENCY)
               
        except Exception as e:
            print(traceback.format_exc())
        finally:
            logger.info("cleaning up..")
            params.put_bool("FlowinitReady", False)
            for p in managed_processes.values():
                p.stop()
            