FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    TZ="Europe/Amsterdam"

RUN apt-get update && \
    apt-get --no-install-recommends install -y \
        build-essential \
        python3 \
        python3-pip \
        python3-dev \
        git \
        curl \
        dfu-util \
        ssh \
        rsync \
        sshpass \
        udev \
    && rm -rf /usr/var/apt/lists/*

# install cross compiler
COPY make/tools.mk tools.mk
RUN mkdir -p /downloads && \
    mkdir -p /tools && \
    make -f tools.mk OSFAMILY=linux DL_DIR=/downloads TOOLS_DIR=/tools arm_sdk_install && \
    rm tools.mk

RUN cp -r /tools/*/* /usr/


# install python utilities
RUN pip install --upgrade pip

COPY lib/main/pi-protocol/python/requirements.txt requirements.txt
RUN pip install -r requirements.txt

COPY lib/main/ekf_c_code_generation/requirements.txt requirements.txt
RUN pip install -r requirements.txt

RUN pip install intelhex pyserial tqdm

RUN git config --global --add safe.directory /indiflight
COPY --chmod=755 entrypoint.sh /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
