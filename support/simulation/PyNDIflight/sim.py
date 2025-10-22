# Main simulation loop
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

from .interfaces import visData

class Sim():
    def __init__(self, uav, imu=None, mocap=None, hil=None, sil=None):
        self.uav = uav
        self.imu = imu
        self.mocap = mocap
        self.hil = hil
        self.sil = sil
        self.t = 0.
        self.i = 0

        # spawn craft
        visData.spawn(self.uav)

    def tick(self, dt):
        if not (self.i % 2): # 4kHz
            self.sil.receive() if self.sil else None
        if not (self.i % 16): # 0.5kHz
            self.hil.receive() if self.hil else None

        if not (self.i % 4):
            self.uav.tick(dt*4)
            self.imu.update() if self.imu else None

        if self.sil:
            self.sil.sendImuAndMotor()
            self.sil.tick() # ticks the indiflight controller

        if not (self.i % 16): # 500 Hz
            self.hil.send() if self.hil else None
        if not (self.i % 80): # 100 Hz
            visData.update()
        if not (self.i % 800): # 10 Hz
            self.mocap.update() if self.mocap else None
            self.sil.sendMocap() if self.sil else None

        self.t += dt
        self.i += 1