# Image to launch a PyNDIflight simulation from a local indiflight clone
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

FROM indiflight-builder

ENV DEBIAN_FRONTEND=noninteractive \
    TZ="Europe/Amsterdam"

RUN apt-get update \
    && apt-get --no-install-recommends install -y \
        gdb \
        gdbserver \
        locales \
        software-properties-common \
        curl \
        python3.10-venv \
    && rm -rf /usr/var/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN export LANG=en_us.UTF-8

# do requirements first, so rebuilding caches this even if Simulation code changes
COPY requirements.txt /requirements.txt
RUN python3 -m venv /python-venv \
    && /python-venv/bin/pip install -r /requirements.txt

EXPOSE 5000
EXPOSE 3333

#ENV EXTRA="-Wno-double-promotion -Wno-misleading-indentation"
ADD --chmod=755 entrypoint.sh /entrypoint.sh

WORKDIR /indiflight
ENTRYPOINT [ "/entrypoint.sh" ]
