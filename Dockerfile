FROM ubuntu:22.04

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
    && apt-get clean

# install python utilities
RUN pip install --upgrade pip
COPY lib/main/pi-protocol/python/requirements.txt requirements.txt
RUN pip install -r requirements.txt && rm requirements.txt
RUN pip install intelhex

# install cross compiler
COPY make/tools.mk tools.mk
RUN mkdir -p /downloads && \
    mkdir -p /tools && \
    make -f tools.mk OSFAMILY=linux DL_DIR=/downloads TOOLS_DIR=/tools arm_sdk_install && \
    rm tools.mk

RUN cp -r /tools/*/* /usr/

# git config
RUN git config --global --add safe.directory /indiflight

RUN echo "#!/bin/bash" > /entrypoint.sh && \
    echo "cd /indiflight" >> /entrypoint.sh && \
    echo 'make -j "$@"' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
