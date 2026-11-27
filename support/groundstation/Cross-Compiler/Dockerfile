# Dockerfile.cross
FROM debian:bookworm

# CROSS compile config and packages
ENV CROSS_ARCH=aarch64
ENV GNU_HOST=$CROSS_ARCH-linux-gnu
ENV C_COMPILER_ARM_LINUX=$GNU_HOST-gcc
ENV CXX_COMPILER_ARM_LINUX=$GNU_HOST-g++

RUN apt-get update \
    && apt-get --no-install-recommends install -y autoconf \
        automake \
        build-essential \
        ca-certificates \
        curl \
        # C/C++ cross compilers
        gcc-$GNU_HOST \
        g++-$GNU_HOST \
        git \
        gnupg \
        libssl-dev \
        openssh-client \
        pkg-config \
        software-properties-common \
        wget \
        cmake \
        rsync \
        sshpass \
    && rm -rf /var/lib/apt/lists/*

# system config
ENV SYSROOT=/rootfs
ENV PACKAGE=/package

# cmake config
#ENV CROSS_INSTALL_PREFIX=/usr/$GNU_HOST
ENV CROSS_INSTALL_PREFIX=$SYSROOT
ENV CROSS_TOOLCHAIN=/cmake/$CROSS_ARCH.toolchain.cmake

# pkg-config confuguration (what a shit way of configuring this...)
ENV PKG_CONFIG_PATH="$(pkg-config --variable pc_path pkg-config):/$SYSROOT/usr/lib/$GNU_HOST/pkgconfig"

# https://cmake.org/cmake/help/v3.16/manual/cmake-toolchains.7.html#cross-compiling-for-linux
# https://cmake.org/cmake/help/v2.8.11/cmake.html#variable%3aCMAKE_PREFIX_PATH
RUN mkdir -p /cmake
RUN echo "set(CMAKE_SYSTEM_NAME Linux)" >> $CROSS_TOOLCHAIN && \
  echo "set(CMAKE_SYSTEM_PROCESSOR $CROSS_ARCH)" >> $CROSS_TOOLCHAIN && \
  echo "set(CMAKE_CROSSCOMPILING TRUE)" >> $CROSS_TOOLCHAIN && \
  echo "set(CMAKE_C_COMPILER /usr/bin/$C_COMPILER_ARM_LINUX)" >> $CROSS_TOOLCHAIN && \
  echo "set(CMAKE_C_COMPILER_WORKS 1)" >> $CROSS_TOOLCHAIN && \
  echo "set(CMAKE_C_LIBRARY_ARCHITECTURE $GNU_HOST)" >> $CROSS_TOOLCHAIN && \
  echo "set(CMAKE_CXX_COMPILER /usr/bin/$CXX_COMPILER_ARM_LINUX)" >> $CROSS_TOOLCHAIN && \
  echo "set(CMAKE_CXX_COMPILER_WORKS 1)" >> $CROSS_TOOLCHAIN && \
  echo "set(CMAKE_CXX_LIBRARY_ARCHITECTURE $GNU_HOST)" >> $CROSS_TOOLCHAIN && \
  echo "set(CMAKE_PREFIX_PATH $CROSS_INSTALL_PREFIX)" >> $CROSS_TOOLCHAIN && \
  echo "set(CMAKE_SYSROOT $SYSROOT)" >> $CROSS_TOOLCHAIN && \
#  echo "set(CMAKE_FIND_ROOT_PATH $CROSS_INSTALL_PREFIX)" >> $CROSS_TOOLCHAIN && \
  echo "set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)" >> $CROSS_TOOLCHAIN && \
  echo "set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)" >> $CROSS_TOOLCHAIN && \
  echo "set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)" >> $CROSS_TOOLCHAIN && \
  echo "set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)" >> $CROSS_TOOLCHAIN
# the *_COMPILER_WORKS lines are maximaal beun, but in cmakes built-in tests,
# CMAKE_PREFIX_PATH seems not to be taken into account, so we'll skip them in
# good faith.
# https://stackoverflow.com/questions/10599038/can-i-skip-cmake-compiler-tests-or-avoid-error-unrecognized-option-rdynamic#comment86573469_30642130


################
# install dependencies needed in the container
RUN apt-get update \
    && apt-get --no-install-recommends install -y \
        python3-pip \
        # more-packages \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --break-system-packages PyYAML semver Jinja2 \
    && pip cache purge
################


# entry point config
ADD --chmod=755 entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
