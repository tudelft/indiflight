# Prepare an image to launch a PyNDIflight Hardware-in-the-loop simulation
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

FROM ubuntu:22.04

RUN apt-get update && \
    apt-get --no-install-recommends install -y \
        build-essential \
        gcc-11 \
        python3 \
        python3-pip \
        python3-dev \
        git \
    && apt-get clean

RUN pip install --upgrade pip

# do requirements first, so rebuilding caches this even if Simulation code changes
COPY Simulation/requirements.txt /requirements.txt
RUN pip install -r /requirements.txt

# copy sim code
COPY Simulation/PyNDIflight /PyNDIflight

EXPOSE 5000

WORKDIR /
ENTRYPOINT [ "python3", "sim.py" ]
