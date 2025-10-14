# Load and cache indiflight logs into SI units, also provide simple plotting
#
# Copyright 2024 Till Blaha (Delft University of Technology)
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <https://www.gnu.org/licenses/>.

import pandas as pd

import logging
import os
import shutil
import glob
import re
import json
import numpy as np
from matplotlib import pyplot as plt

import ctypes
from platformdirs import user_cache_dir

logger = logging.getLogger(__name__)

local_rc = plt.rcParams.copy()
local_rc.update({
#    "text.usetex": True,
#    "font.family": "Helvetica",
    "font.family": "sans-serif",
    "font.size": 12,
    "axes.grid": True,
    "axes.grid.which": 'both',
    "grid.linestyle": '--',
    "grid.alpha": 0.7,
    "axes.labelsize": 12,
    "axes.titlesize": 16,
    "xtick.labelsize": 12,
    "ytick.labelsize": 12,
    "legend.loc": 'best',
    "legend.fontsize": 9,
    'figure.subplot.bottom': 0.05,
    'figure.subplot.left': 0.075,
    'figure.subplot.right': 0.95,
    'figure.subplot.top': 0.925,
    'figure.subplot.hspace': 0.2,
    'figure.subplot.wspace': 0.15,
    'figure.titlesize': 'large',
})

class IndiflightLog(object):
    UNIT_FLOAT_TO_UNSIGNED16VB = ((127 << 7) - 1)
    UNIT_FLOAT_TO_SIGNED16VB = ((127 << 6) - 1)
    RADIANS_TO_DEGREES = 57.2957796
    RADIANS_TO_DECADEGREES = 1e-1 * RADIANS_TO_DEGREES
    RADIANS_TO_DECIDEGREES = 10 * RADIANS_TO_DEGREES
    RADIANS_TO_HUNDRESOFRADIANS = 0.01
    METER_TO_MM = 1000.
    METER_TO_CM = 100.
    RAD_TO_MRAD = 1000.
    ONE_G = 9.80665
    PERCENT = 100.
    DSHOT_MIN = 158.
    DSHOT_MAX = 2048.
    CACHE_NAME = "indiflight_logs"
    LIBRARY_SO = os.path.join(os.path.dirname(__file__), "blackbox_decode.cpython-312-x86_64-linux-gnu.so")

    @staticmethod
    def modeToText(bits):
        # rc_modes.h:boxId_e, but only the first 32 bits i guess, because
        # blackbox.c:1200 only memcpys 4 bytes
        def single(bit):
            match(bit):
                case 0: return "ARM"
                case 1: return "ANGLE"
                case 2: return "HORIZON"
                case 3: return "MAG"
                case 4: return "HEADFREE"
                case 5: return "PASSTHRU"
                case 6: return "FAILSAFE"
                case 7: return "GPSRESCUE"
                case 8: return "VELCTL"
                case 9: return "POSCTL"
                case 10: return "CATAPULT"
                case 11: return "LEARNER"
                case 12: return "PIDCTL"
                case 13: return "NNCTL"
                case 14: return "ANTIGRAVITY"
                case 15: return "HEADADJ"
                case 16: return "CAMSTAB"
                case 17: return "BEEPERON"
                case 18: return "LEDLOW"
                case 19: return "CALIB"
                case 20: return "OSD"
                case 21: return "TELEMETRY"
                case 22: return "SERVO1"
                case 23: return "SERVO2"
                case 24: return "SERVO3"
                case 25: return "BLACKBOX"
                case 26: return "AIRMODE"
                case 27: return "3D"
                case 28: return "FPVANGLEMIX"
                case 29: return "BLACKBOXERASE"
                case 30: return "RESETHOME"
                case 31: return "CAMERA1"
                case 32: return "CAMERA2"
                case 33: return "CAMERA3"
                case 34: return "FLIPOVERAFTERCRASH"
                case 35: return "PREARM"
                case 36: return "THROWTOARM"
                case 37: return "BEEPGPSCOUNT"
                case 38: return "VTXPITMODE"
                case 39: return "PARALYZE"
                case 40: return "USER1"
                case 41: return "USER2"
                case 42: return "USER3"
                case 43: return "USER4"
                case 44: return "PIDAUDIO"
                case 45: return "ACROTRAINER"
                case 46: return "VTXCONTROLDISABLE"
                case 47: return "LAUNCHCONTROL"
                case 48: return "MSPOVERRIDE"
                case 49: return "STICKCOMMANDDISABLE"
                case 50: return "BEEPERMUTE"
                case 51: return "READY"

        try:
            return [single(bit) for bit in bits]
        except TypeError:
            return single(bits)

    def __init__(self, filename, logId=1, timeRange=None, resetTime=True):
        # check inputs
        if logId < 1 or logId > 99:
            raise ValueError(f"logId must be between 1 and 99. Was {logId}")

        self.logIdStr = str(logId).zfill(2)

        # setup libary
        lib = ctypes.CDLL(self.LIBRARY_SO)
        lib.main.argtypes = (ctypes.c_int, ctypes.POINTER(ctypes.c_char_p))
        lib.main.restype = ctypes.c_int

        # setup paths
        cachedir = user_cache_dir(self.CACHE_NAME, self.CACHE_NAME)
        if not os.path.exists(cachedir):
            os.makedirs(cachedir)
        else:
            files = glob.glob(os.path.join(cachedir, "log*"))
            for f in files:
                os.remove(f)

        self.cache_bfl = os.path.join(cachedir, "log.bfl")
        self.cache_csv = os.path.join(cachedir, f"log.{self.logIdStr}.csv")
        self.cache_events = os.path.join(cachedir, f"log.{self.logIdStr}.event")
        self.cache_headers_csv = os.path.join(cachedir, f"log.{self.logIdStr}.headers.csv")

        # copy to cache folder
        self.filename = filename
        shutil.copy(filename, self.cache_bfl)

        # convert to csv
        logger.info("Parsing logfile")
        argv = [b"blackbox_decode", self.cache_bfl.encode("ascii")]
        argc = 2
        argv_ctypes = (ctypes.c_char_p * argc)(*argv)
        _ = lib.main(argc, argv_ctypes)

        # dump data rows into pandas frame. # TODO: only import until range?
        logger.info("Importing into dataframe")
        self.raw = pd.read_csv(self.cache_csv, skipinitialspace=True)
        self.raw.set_index('loopIteration', inplace=True)

        # get parameters
        try:
            par_frame = pd.read_csv(
                self.cache_headers_csv,
                skiprows=13,
                header=None,
                usecols=[0, 1],
                names=["parameter", "value"])

            self.parameters = par_frame.set_index("parameter")["value"].to_dict()

            # Attempt to convert each value in the dictionary to an integer if possible
            for key, value in self.parameters.items():
                try:
                    self.parameters[key] = int(value)
                except ValueError:
                    pass  # Leave the value as a string if it can't be converted
        except:
            logger.error(f"No header file for log {self.logIdStr} in {self.filename}. Likely empty or corrupt log. Continuing")
            open(self.cache_headers_csv, 'a').close() # create empty file
            self.parameters = {}

        # get events
        try:
            with open(self.cache_events, 'r') as file:
                self.events = [json.loads(line) for line in file]

        except FileNotFoundError:
            logger.error(f"No event file for log {self.logIdStr} in {self.filename}. Likely empty or corrupt log. Continuing")
            open(self.cache_events, 'a').close() # create empty file
            self.events = []


        self.num_learner_vars = sum([
            re.match(r'^fx_p_rls_x\[[0-9]+\]$', c) is not None 
            for c in self.raw.columns])

        # crop to time range and apply scaling
        logger.info("Apply scaling and crop to range")
        self.data = self._processData(timeRange, resetTime=resetTime)

        # parse rc box mode change events (use LogData.modeToText to decode)
        logger.info("Convert flight modes to events")
        self.flags = self._convertModeFlagsToEvents()
        logger.info("Done")

    def outputCsv(self, units: str, path=None):
        # copy csv back to folder if required
        fileStem = os.path.splitext(os.path.basename(self.filename))[0]
        outputPath = path if path is not None else os.path.dirname(self.filename)

        if not os.path.exists(outputPath):
            raise ValueError(f"CSV output path {outputPath} does not exist...")

        logger.warning(f"Outputting converted csv to {outputPath}/")

        # always just copy headers and events
        shutil.copy(self.cache_headers_csv,
            os.path.join(outputPath, fileStem+f".{self.logIdStr}.headers.csv"))
        shutil.copy(self.cache_events,
            os.path.join(outputPath, fileStem+f".{self.logIdStr}.event"))

        if units.lower() == "raw":
            shutil.copy(self.cache_csv, 
                os.path.join(outputPath, fileStem+f".{self.logIdStr}.raw.csv"))
        elif units.lower() == "si":
            self.data.to_csv(
                os.path.join(outputPath, fileStem+f".{self.logIdStr}.si.csv"),
                #float_format="%.8e", # output with float precision, not double. this made the file even more huge
                index=True
                )
        else:
            raise ValueError("outputCsv: units must be either 'raw' or 'si'")
        
    def outputRosbag(self):
        from sensor_msgs.msg import Imu, NavSatFix
        from geometry_msgs.msg import PoseStamped
        import rosbag2_py
        from rclpy.serialization import serialize_message
        from builtin_interfaces.msg import Time

        def to_ros_time(timestamp):
            """Helper to convert a timestamp to ROS 2 Time message"""
            sec = int(timestamp)
            nanosec = int((timestamp - sec) * 1e9)
            ros_time = Time()
            ros_time.sec = sec
            ros_time.nanosec = nanosec
            return ros_time

        # Initialize rosbag2 writer
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri="output.mcap", storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", output_serialization_format="cdr"
            ),
        )

        # Create topics and their types
        writer.create_topic( rosbag2_py.TopicMetadata( name="/imu", type="sensor_msgs/Imu", serialization_format="cdr") )
        writer.create_topic( rosbag2_py.TopicMetadata( name="/pose", type="geometry_msgs/PoseStamped", serialization_format="cdr") )
        #writer.create_topic({'name': '/gps', 'type': 'sensor_msgs/NavSatFix', 'serialization_format': 'cdr'})

        # Iterate over each row in the DataFrame
        for index, row in self.data.iterrows():
            # Convert timestamp to ROS Time
            timestamp = to_ros_time(row['timeS'])

            # Populate IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = timestamp
            # Example data for IMU, replace with your actual columns
            imu_msg.angular_velocity.x = row['gyroADCafterRpm[0]']
            imu_msg.angular_velocity.y = row['gyroADCafterRpm[1]']
            imu_msg.angular_velocity.z = row['gyroADCafterRpm[2]']
            writer.write('/imu', serialize_message(imu_msg), timestamp.sec * 1_000_000_000 + timestamp.nanosec)

            pose_msg = PoseStamped()
            pose_msg.header.stamp = timestamp
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = row['ekf_pos[0]']
            pose_msg.pose.position.y = row['ekf_pos[1]']
            pose_msg.pose.position.z = row['ekf_pos[2]']
            pose_msg.pose.orientation.w = row['ekf_quat[0]']
            pose_msg.pose.orientation.x = row['ekf_quat[1]']
            pose_msg.pose.orientation.y = row['ekf_quat[2]']
            pose_msg.pose.orientation.z = row['ekf_quat[3]']
            writer.write('/pose', serialize_message(pose_msg), timestamp.sec * 1_000_000_000 + timestamp.nanosec)

        print("Finished writing to ROS 2 bag file.")
        del writer

    def addToRerun(self, name: str, clockOffsetSeconds=0.):
        import rerun as rr

        # define elements to log
        point3 = [ # in 3D view: (column, showTraceAsWell, markerSize)
            ("ekf_pos", True, 0.005),
            ("extPos", True, 0.01),
            ("posSp", True, 0.01),
        ]
        arrow3 = [ # vectors: (direction, origin, lengthScaler)
            ("ekf_vel", "ekf_pos", 0.3),
            ("velSp", "posSp", 0.3),
        ]
        pose = [ # show a triad to indicate pose (origin, rotationQuaternion)
            ("ekf_pos", "ekf_quat"),
            ("extPos", "extQuat"),
            ("posSp", "quatSp"),
        ]
        timeseries = [ # just time series: (column, indices_or_minus_one)
            ("rcCommand", range(4)),
            ("vbatLatest", -1),
            ("amperageLatest", -1),
            ("gyroSp", range(3)),
            ("gyroADCafterRpm", range(3)),
            ("gyroADC", range(3)),
            ("alphaSp", range(3)),
            ("alpha", range(3)),
            ("spfSp", range(3)),
            ("accADCafterRpm", range(3)),
            ("accSmooth", range(3)),
            ("dv", range(6)),
            ("u", range(4)),
            ("u_state", range(4)),
            ("motor", range(4)),
            ("omega", range(4)),
            ("omegaUnfiltered", range(4)),
            ("omega_dot", range(4)),
            ("extTime", -1),
            ("posSp", range(3)),
            ("pos", range(3)),
            ("velSp", range(3)),
            ("vel", range(3)),
            ("accSp", range(3)),
            ("extPos", range(3)),
            ("extVel", range(3)),
            ("extQuat", range(4)),
            ("ekf_pos", range(3)),
            ("ekf_vel", range(3)),
            ("ekf_quat", range(4)),
            ("ekf_acc_b", range(3)),
            ("ekf_gyro_b", range(3)),
            ("flightModeFlags", -1),
        ]

        # prepare arrays and lambdas
        timeS = self.data["timeS"].to_numpy() + clockOffsetSeconds
        getValues = lambda s, ir: self.data[[f"{s}[{i}]" for i in ir]].to_numpy().flatten()
        getPart = lambda i: [i for _ in range(self.N)]
        getManyOf = lambda x: np.ones(self.N)*x

        # get axis system correct
        rr.log(name, rr.ViewCoordinates.FRD, static=True)

        # log positions
        for p, trace, r in point3:
            rr.send_columns(
                f"{name}/{p}",
                times=[rr.TimeSecondsColumn("time", timeS)],
                components=[ rr.Points3D.indicator(),
                             rr.components.Position3DBatch( getValues(p, range(3)) ).partition(getPart(1)),
                             rr.components.RadiusBatch( getManyOf(0.075) ) ]
                )
            if trace:
                rr.log(
                    f"{name}/{p}/trace",
                    rr.Points3D( getValues(p, range(3)), radii=r ),
                    timeless=True
                )

        # log vectors
        for vec, orig, scale in arrow3:
            rr.send_columns(
                f"{name}/{vec}",
                times=[rr.TimeSecondsColumn("time", timeS)],
                components=[ rr.Arrows3D.indicator(),
                             rr.components.Vector3DBatch( scale*getValues(vec, range(3)) ).partition(getPart(1)),
                             rr.components.Position3DBatch( getValues(orig, range(3)) ).partition(getPart(1)),
                             rr.components.RadiusBatch( getManyOf(0.025) ) ]
                )

        # log axes triads
        for p, q in pose:
            rr.send_columns(
                f"{name}/{q}",
                times=[rr.TimeSecondsColumn("time", timeS)],
                components=[ rr.Transform3D.indicator(),
                             rr.components.Translation3DBatch( getValues(p, range(3)) ).partition(getPart(1)),
                             rr.components.RotationQuatBatch( getValues(q, [1,2,3,0]) ).partition(getPart(1)), # reverse quaternin component order
                             rr.components.AxisLengthBatch( getManyOf(0.5) ) ]
            )

        # log scalar timeseries
        for s, ir in timeseries:
            try:
                for i in ir:
                    rr.send_columns(
                        f"{name}/timeseries/{s}/{i}",
                        times=[rr.TimeSecondsColumn("time", timeS)],
                        components=[ rr.components.ScalarBatch( getValues(s, [i]) ) ],
                    )
            except TypeError:
                rr.send_columns(
                    f"{name}/timeseries/{s}",
                    times=[rr.TimeSecondsColumn("time", timeS)],
                    components=[ rr.components.ScalarBatch( self.data[s] ) ],
                )

        # log flight mode changes (must be last)
        for _, row in self.flags.iterrows():
            time = row["timeUs"] * 1e-6 + clockOffsetSeconds
            enable = IndiflightLog.modeToText(row["enable"])
            disable = IndiflightLog.modeToText(row["disable"])

            rr.set_time_seconds("time", time)
            rr.log(
                f"{name}/events",
                rr.TextLog(f"[IndiflightLog] FC Time {time:.3f}: Enabled flight modes {enable}, Disabled flight modes {disable}",
                           level=rr.TextLogLevel.INFO)
            )

        # log other events
        for event in self.events:
            time = event['time'] * 1e-6 + clockOffsetSeconds
            msg = event['name']

            rr.set_time_seconds("time", time)
            rr.log(
                f"{name}/events",
                rr.TextLog(f"[IndiflightLog] FC Time {time:.3f}: Received event: {msg}",
                           level=rr.TextLogLevel.WARN)
            )

    def crop(self, start, stop):
        timeS = self.data['timeS']
        boolarr = (timeS >= start) & (timeS <= stop)
        return self.data[boolarr], timeS[boolarr].to_numpy()

    def resample(self, t_sample):
        # strip duplicates
        self.data = self.data[~self.data.index.duplicated(keep='first')]

        self.data.set_index(self.data['timeUs'], inplace=True)

        # Generate the new index based on the desired interval
        new_index = np.arange(self.data.index.min(),
                              self.data.index.max() + t_sample,
                              t_sample)

        self.data = pd.concat([self.data, pd.DataFrame(index=new_index)])
        self.data = self.data.loc[~self.data.index.duplicated(keep='first')]
        self.data = self.data.sort_index().interpolate(method="index")
        self.data = self.data.loc[new_index]
        self.data = self.data.reset_index() # defragment
        self.data = self.data.drop('index', axis=1) # remove "index" column

    def _processData(self, timeRange, resetTime=True):
        t0 = self.raw['time'].iloc[0] if resetTime else 0

        # crop relevant time range out of raw, and adjust time
        if timeRange is not None:
            data = self.raw[ ( (self.raw['time'] - t0) > timeRange[0]*1e3 )
                        & ( (self.raw['time'] - t0) <= timeRange[1]*1e3) ].copy(deep=True)
        else:
            data = self.raw.copy(deep=True)

        self.N = len(data)

        # manage time in s, ms and us
        if resetTime:
            data['time'] -= self.raw['time'].iloc[0]

        data['timeUs'] = data['time'].copy(deep=True)
        timeUs = data['timeUs'].to_numpy()
        data['timeMs'] = 1e-3 * timeUs
        data['timeS'] = 1e-6 * timeUs

        # adjust column units
        if "blackbox_high_resolution" in self.parameters.keys():
            highRes = 10. if self.parameters['blackbox_high_resolution'] else 1.
        else:
            highRes = 1.

        for col in data.columns:
            if col == 'loopIteration':
                data[col] = data[col].astype(int)
            elif col == 'rcCommand[3]':
                data[col] -= 1000. * highRes
                data[col] /= 1000. * highRes
            elif col.startswith('rcCommand'):
                data[col] /= 500. * highRes
            elif col.startswith('gyro'):
                data[col] /= self.RADIANS_TO_DEGREES * highRes
            elif col.startswith('accSp'):
                data[col] /= self.METER_TO_CM
            elif col.startswith('acc'):
                data[col] *= self.ONE_G / self.parameters['acc_1G']
            elif re.match(r'^motor\[[0-9]+\]$', col):
                data[col] -= self.DSHOT_MIN
                data[col] /= (self.DSHOT_MAX - self.DSHOT_MIN)
            elif col.startswith('quat') or col.startswith('extQuat') or col.startswith('ekf_quat') or col.startswith('localQuat'):
                data[col] /= self.UNIT_FLOAT_TO_SIGNED16VB
            elif col.startswith('alpha'):
                data[col] /= self.RADIANS_TO_DECADEGREES
            elif col.startswith('spfSp'):
                data[col] /= self.METER_TO_CM
            elif col.startswith('dv'):
                data[col] /= 10.
            elif col.startswith('u['):
                data[col] /= self.UNIT_FLOAT_TO_SIGNED16VB
            elif col.startswith('u_state'):
                data[col] /= self.UNIT_FLOAT_TO_SIGNED16VB
            elif col.startswith('omega_dot'):
                data[col] *= 100
            elif col.startswith('omega'):
                data[col] /= 1.
            elif col.startswith('pos') or col.startswith('extPos') or col.startswith('ekf_pos') or col.startswith('localPos'):
                data[col] /= self.METER_TO_MM
            elif col.startswith('vel') or col.startswith('extVel') or col.startswith('ekf_vel') or col.startswith('localVel'):
                data[col] /= self.METER_TO_CM
            elif col.startswith('extAtt') or col.startswith('ekf_att'):
                data[col] /= 1000.
            elif col.startswith('ekf_acc_b'):
                data[col] /= 1000.
            elif col.startswith('ekf_gyro_b'):
                data[col] /= self.RADIANS_TO_DEGREES
            elif (match := re.match(r'^motor_[0-9]+_rls_x\[([0-9]+)\]$', col)):
                bbscaler = 1000.
                yscaler = 0.001
                if match.group(1) in ['0', '1', '2']:
                    # a, b, and w0
                    ascaler = 1.
                elif match.group(1) in ['3']:
                    # time constant
                    ascaler = 0.0001
                else:
                    raise NotImplementedError(f"Regressor {match.group(1)} not expected")

                data[col] /= bbscaler * yscaler / ascaler
            elif (match := re.match(r'^fx_([xyzpqr])_rls_x\[([0-9]+)\]$', col)):
                bbscaler = 1000.
                if match.group(1) in ['x', 'y', 'z']:
                    # forces. All 4 regressors have scale 1e-5. Output has scaler 10.
                    yscaler = 10.
                    ascaler = 1e-5
                elif match.group(1) in ['p', 'q', 'r']:
                    # rotations. First 4 regressors have scale 1e-5. Last 4 scale 1e-3 Output has scaler 1.
                    yscaler = 1.
                    num_vars = 8 if self.num_learner_vars > 8 else 4
                    w2vars = [str(i) for i in range(num_vars)]
                    wdotvars = [str(i) for i in range(num_vars, num_vars*2)]
                    if match.group(2) in w2vars:
                        ascaler = 1e-5
                    elif match.group(2) in wdotvars:
                        ascaler = 1e-3
                    else:
                        raise NotImplementedError(f"Regressor {match.group(2)} not expected")
                else:
                    raise NotImplementedError(f"Output {match.group(1)} not expected")

                data[col] /= bbscaler * yscaler / ascaler
            elif (match := re.match(r'^imu_rls_x\[([0-9]+)\]$', col)):
                if match.group(1) not in ['0', '1', '2']:
                    raise NotImplementedError(f"Output {match.group(1)} not expected")
                    # forces. All 4 regressors have scale 1e-5. Output has scaler 10.
                bbscaler = 1000.
                yscaler = 1.
                ascaler = 1e-2
                data[col] /= bbscaler * yscaler / ascaler
            elif (col.startswith("learnerGains")):
                data[col] /= 10.
            elif (col.startswith("hoverAttitude")):
                data[col] /= self.UNIT_FLOAT_TO_SIGNED16VB
            elif (match := re.match(r'^.*_lambda$', col)):
                data[col] /= self.UNIT_FLOAT_TO_UNSIGNED16VB
            elif (match := re.match(r'^.*_e_var$', col)):
                data[col] /= 0.1 * ((1 << 16) - 1)
            elif (col == "flightModeFlags") or (col == "stateFlags")\
                    or (col == "failsafePhase") or (col == "rxSignalReceived")\
                    or (col == "rxFlightChannelValid"):
                with pd.option_context("future.no_silent_downcasting", True):
                    data.replace({col: {"": 0}}, inplace=True)
                data[col] = data[col].astype(int)

        return data

    def _convertModeFlagsToEvents(self):
        # parse changes in flightModeFlags column into its own dataframe
        flags = []
        lastFlags = 0
        for index, row in self.data.iterrows():
            # for some reason, flightModeFlags in the logs doesnt correspond to
            # flightModeFlags in betaflight, but to rcModeActivationMask...
            currentFlags = int(row['flightModeFlags'])
            e, d = IndiflightLog._getModeChanges(currentFlags, lastFlags)
            lastFlags = currentFlags

            if (len(e) + len(d)) > 0:
                flags.append({'loopIteration': index, 
                                'timeUs': int(row['timeUs']),
                                "enable": e, 
                                "disable": d})

        if (len(flags) == 0):
            df = pd.DataFrame(columns=["loopIteration", "timeUs", "enable", "disable"])
            return df

        df = pd.DataFrame(flags)
        df.set_index('loopIteration', inplace=True)
        return df

    @staticmethod
    def _getModeChanges(new, old=0):
        # find if bits have been turned on (enabled) or turned off (enabled) 
        # between two int32
        enabled = []
        disabled = []
        for i in range(32):
            bitSel = (1 << i)
            if (new & bitSel) and not (old & bitSel):
                enabled.append(i)
            elif not (new & bitSel) and (old & bitSel):
                disabled.append(i)

        return enabled, disabled

    def resetTime(self):
        self.data['timeS'] -= self.data['timeS'].iloc[0]
        self.data['timeMs'] -= self.data['timeMs'].iloc[0]
        self.data['timeUs'] -= self.data['timeUs'].iloc[0]

    def plot(self):
        with plt.rc_context(rc=local_rc):
            # plot some overview stuff
            f, axs = plt.subplots(4, 4, figsize=(12,9), sharex='all', sharey='row')
            axs[2, 3].axis('off')
            axs[3, 3].axis('off')

            for i in range(4):
                line1, = axs[0, i].plot(self.data['timeMs'], self.data[f'omega[{i}]'], label=f'onboard rpm omega[{i}]')

                yyax = axs[0, i].twinx()
                line2, = yyax.plot(self.data['timeMs'], self.data[f'u[{i}]'], label=f'command u[{i}]', color='orange')
                yyax.tick_params('y', colors='orange')
                yyax.set_ylim(bottom=-0.1, top=1.1)

                lines = [line1, line2]
                labels = [line.get_label() for line in lines]

                axs[0, i].legend(lines, labels)
                axs[0, i].set_ylim(bottom=-0.1, top=1.1)
                if (i==0):
                    axs[0, i].set_ylabel("Motor command/output [-], [rad/s]")

            for i in range(4):
                axs[1, i].plot(self.data['timeMs'], self.data[f'omega_dot[{i}]'], label=f'onboard drpm omega_dot[{i}]')
                axs[1, i].legend()
                if (i==0):
                    axs[1, i].set_ylabel("Motor acceleration [rad/s/s]")
                if (i==3):
                    axs[1, i].set_xlabel("Time [ms]")


            for i in range(3):
                axs[2, i].plot(self.data['timeMs'], self.data[f'alpha[{i}]'], label=f'onboard angular accel alpha[{i}]')
                axs[2, i].legend()
                if (i==0):
                    axs[2, i].set_ylabel("Angular acceleration [rad/s/s]")

            for i in range(3):
                axs[3, i].plot(self.data['timeMs'], self.data[f'accSmooth[{i}]'], label=f'onboard linear accSmooth[{i}]')
                axs[3, i].legend()
                axs[3, i].set_xlabel("Time [ms]")
                if (i==0):
                    axs[3, i].set_ylabel("Specific force [N/kg]")

            # Maximize the window on Linux
            mgr = plt.get_current_fig_manager()
            mgr.resize(1920, 1080)

            f.show()
            return f

    def compare(self, other, other_offset=0, self_name='A', other_name='B'):
        a = self_name
        b = other_name

        with plt.rc_context(rc=local_rc):
            f, axs = plt.subplots(4, 4, figsize=(12,9), sharex='all', sharey='row')
            f.suptitle(f"Log comparison -- {a} vs {b}")
            axs[2, 3].axis('off')
            axs[3, 3].axis('off')

            otherTimeMs = other.data['timeMs'] - other_offset;

            for i in range(4):
                line1, = axs[0, i].plot(self.data['timeMs'], self.data[f'omega[{i}]'], label=f'{a}: onboard rpm omega[{i}]')
                line1b, = axs[0, i].plot(otherTimeMs, other.data[f'omega[{i}]'], linestyle='--', label=f'{b}: onboard rpm omega[{i}]')

                yyax = axs[0, i].twinx()
                line2, = yyax.plot(self.data['timeMs'], self.data[f'u[{i}]'], label=f'{a}: command u[{i}]', color='green')
                line2b, = yyax.plot(otherTimeMs, other.data[f'u[{i}]'], linestyle='--', label=f'{b}: command u[{i}]', color='black')
                yyax.tick_params('y', colors='green')
                yyax.set_ylim(bottom=0)

                lines = [line1, line1b, line2, line2b]
                labels = [line.get_label() for line in lines]

                axs[0, i].legend(lines, labels)
                axs[0, i].set_ylim(bottom=0)
                if (i==0):
                    axs[0, i].set_ylabel("Motor command/output [-], [rad/s]")

            for i in range(4):
                axs[1, i].plot(self.data['timeMs'], self.data[f'omega_dot[{i}]'], label=f'{a}: onboard drpm omega_dot[{i}]')
                axs[1, i].plot(otherTimeMs, other.data[f'omega_dot[{i}]'], linestyle='--', label=f'{b}: onboard drpm omega_dot[{i}]')
                axs[1, i].legend()
                if (i==0):
                    axs[1, i].set_ylabel("Motor acceleration [rad/s/s]")
                if (i==3):
                    axs[1, i].set_xlabel("Time [ms]")

            for i in range(3):
                axs[2, i].plot(self.data['timeMs'], self.data[f'alpha[{i}]'], label=f'{a}: onboard angular accel alpha[{i}]')
                axs[2, i].plot(otherTimeMs, other.data[f'alpha[{i}]'], linestyle='--', label=f'{b}: onboard angular accel alpha[{i}]')
                axs[2, i].legend()
                if (i==0):
                    axs[2, i].set_ylabel("Angular acceleration [rad/s/s]")

            for i in range(3):
                axs[3, i].plot(self.data['timeMs'], self.data[f'accSmooth[{i}]'], label=f'{a}: onboard linear accSmooth[{i}]')
                axs[3, i].plot(otherTimeMs, other.data[f'accSmooth[{i}]'], linestyle='--', label=f'{b}: onboard linear accSmooth[{i}]')
                axs[3, i].legend()
                axs[3, i].set_xlabel("Time [ms]")
                if (i==0):
                    axs[3, i].set_ylabel("Specific force [N/kg]")

            # Maximize the window on Linux
            mgr = plt.get_current_fig_manager()
            mgr.resize(1920, 1080)

            f.show()
            return f
