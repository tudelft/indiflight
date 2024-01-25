#!/usr/bin/env python3
"""Convert quaternion into ZYX Euler Angles"""

from scipy.spatial.transform import Rotation as R
from argparse import ArgumentParser

parser = ArgumentParser(description=__doc__)
parser.add_argument("--enu", action='store_true', help="use enu and rfu")
parser.add_argument("quaternion", help="scalar-first", nargs='+')

args = parser.parse_args()
quat = [args.quaternion[1], args.quaternion[2], args.quaternion[3], args.quaternion[0]]

if args.enu:
    e = R.from_quat(quat).as_euler('ZXY', degrees=True)
    print(f"Yaw  : {e[0]:8.3f}deg\nPitch: {e[1]:8.3f}deg\nRoll : {e[2]:8.3f}deg")
else:
    e = R.from_quat(quat).as_euler('ZYX', degrees=True)
    print(f"Yaw  : {e[0]:8.3f}deg\nPitch: {e[1]:8.3f}deg\nRoll : {e[2]:8.3f}deg")

