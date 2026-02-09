
from indiflight_log_tools import IndiflightLog
import matplotlib.pyplot as plt
import matplotlib

plt.close('all')
matplotlib.use('tkagg') 

from pyFlightPlotter import local_rc
from indiflightPlotter import IndiflightPlotter, \
    IndiflightMotorSysIdPlotter, \
    IndiflightServoSysIdPlotter, \
    IndiflightViewport, \
    IndiflightIndividualSysIdPlotter, \
    IndiflightMoments, \
    IndiflightEffectiveness
from pyFlightPlotter import Quadrotor, Tailsitter, BlittedCursor

# set local_rc
plt.rcParams.update(local_rc)

import os
import sys

class suppress_output:
    def __enter__(self):
        self._stdout_fd = os.dup(1)
        self._stderr_fd = os.dup(2)

        self._devnull = os.open(os.devnull, os.O_WRONLY)

        os.dup2(self._devnull, 1)
        os.dup2(self._devnull, 2)

    def __exit__(self, exc_type, exc_val, exc_tb):
        os.dup2(self._stdout_fd, 1)
        os.dup2(self._stderr_fd, 2)

        os.close(self._devnull)
        os.close(self._stdout_fd)
        os.close(self._stderr_fd)

path = "/mnt/data/WorkData/BlackboxLogs/"
files = [
    "2025-12-17/LOG00250_success.BFL",
    "2025-12-17/LOG00251_omg.BFL",
    "2025-12-18/LOG00252_success_under_net.BFL",
    "2025-12-18/LOG00253_success.BFL",
    "2025-12-18/LOG00254_success.BFL",
    "2025-12-18/LOG00255_crash.BFL",
    "2025-12-18/LOG00256_success.BFL",
    "2026-01-05/LOG00259_crash_demo1_wrongMocap.BFL",
    "2026-01-05/LOG00260_crash_demo2_wrongMocap.BFL",
    "2026-01-05/LOG00263_success.BFL",
    "2026-01-13/LOG00270_crash.BFL",
    "2026-01-13/LOG00271_crash.BFL",
    "2026-01-13/LOG00272_crash_wall.BFL",
    "2026-01-26/LOG00275_crash.BFL",
]

for file in files:
    with suppress_output():
        log = IndiflightLog(path + file, logId=1, resetTime=True)

    print()
    print(f"- Filename {file}")
    print(f"    - Firmware Revision `{log.parameters['Firmware revision']}`")
    print("```")
    aplt = IndiflightIndividualSysIdPlotter(log.data, Nr=2, Ns=2, true=None, name=f"{file} -- Onboard Individual Analysis")
    print("```")

    plt.close('all')

