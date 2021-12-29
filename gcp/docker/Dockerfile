FROM ubuntu:20.04
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        locales \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# https://gitlab.com/nvidia/driver/blob/master/ubuntu16.04/Dockerfile
RUN dpkg --add-architecture i386 \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       apt-utils \
       build-essential \
       ca-certificates \
       dirmngr \
       gpg-agent \
       gnupg2 \
       kmod \
       libc6:i386 \
       libelf-dev \
       software-properties-common \
       wget \
    && rm -rf /var/lib/apt/lists/*

# NOTE: Install nvidia drivers from nvidia apt repository to match the version of host machines
# xserver-xorg-video-nvidia-XXX is what we actually want to install
ENV NVIDIA_DRIVER_MAJOR_VERSION=460
ENV NVIDIA_DRIVER_PACKAGE_VERSION=460.91.03-0ubuntu1
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin \
    && mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600 \
    && apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub \
    && add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/ /" \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
            libnvidia-cfg1-${NVIDIA_DRIVER_MAJOR_VERSION}=${NVIDIA_DRIVER_PACKAGE_VERSION} \
            libnvidia-common-${NVIDIA_DRIVER_MAJOR_VERSION}=${NVIDIA_DRIVER_PACKAGE_VERSION} \
            libnvidia-compute-${NVIDIA_DRIVER_MAJOR_VERSION}=${NVIDIA_DRIVER_PACKAGE_VERSION} \
            libnvidia-decode-${NVIDIA_DRIVER_MAJOR_VERSION}=${NVIDIA_DRIVER_PACKAGE_VERSION} \
            libnvidia-encode-${NVIDIA_DRIVER_MAJOR_VERSION}=${NVIDIA_DRIVER_PACKAGE_VERSION} \
            libnvidia-fbc1-${NVIDIA_DRIVER_MAJOR_VERSION}=${NVIDIA_DRIVER_PACKAGE_VERSION} \
            libnvidia-gl-${NVIDIA_DRIVER_MAJOR_VERSION}=${NVIDIA_DRIVER_PACKAGE_VERSION} \
            libnvidia-ifr1-${NVIDIA_DRIVER_MAJOR_VERSION}=${NVIDIA_DRIVER_PACKAGE_VERSION} \
            libnvidia-extra-${NVIDIA_DRIVER_MAJOR_VERSION}=${NVIDIA_DRIVER_PACKAGE_VERSION} \
            nvidia-compute-utils-${NVIDIA_DRIVER_MAJOR_VERSION}=${NVIDIA_DRIVER_PACKAGE_VERSION} \
            nvidia-dkms-${NVIDIA_DRIVER_MAJOR_VERSION}=${NVIDIA_DRIVER_PACKAGE_VERSION} \
            nvidia-driver-${NVIDIA_DRIVER_MAJOR_VERSION}=${NVIDIA_DRIVER_PACKAGE_VERSION} \
            nvidia-kernel-common-${NVIDIA_DRIVER_MAJOR_VERSION}=${NVIDIA_DRIVER_PACKAGE_VERSION} \
            nvidia-kernel-source-${NVIDIA_DRIVER_MAJOR_VERSION}=${NVIDIA_DRIVER_PACKAGE_VERSION} \
            nvidia-modprobe=${NVIDIA_DRIVER_PACKAGE_VERSION} \
            nvidia-utils-${NVIDIA_DRIVER_MAJOR_VERSION}=${NVIDIA_DRIVER_PACKAGE_VERSION} \
            nvidia-settings=${NVIDIA_DRIVER_PACKAGE_VERSION} \
            xserver-xorg-video-nvidia-${NVIDIA_DRIVER_MAJOR_VERSION}=${NVIDIA_DRIVER_PACKAGE_VERSION} \
            libxnvctrl0=${NVIDIA_DRIVER_PACKAGE_VERSION} \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        alsa \
        dbus-x11 \
        git \
        libdbus-c++-1-0v5 \
        libgtk2.0-0 \
        libvulkan1 \
        libvulkan-dev \
        mesa-utils \
        pulseaudio \
        unzip \
        vulkan-utils \
        xinit \
        x11-apps \
        zip \
    && rm -rf /var/lib/apt/lists/*

# x11vnc
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        automake autoconf libssl-dev xorg-dev libvncserver-dev \
    && rm -rf /var/lib/apt/lists/* \
    && git clone https://github.com/LibVNC/x11vnc.git \
    && cd x11vnc \
    && ./autogen.sh \
    && make \
    && cp src/x11vnc /usr/bin/x11vnc

# novnc
RUN wget https://github.com/novnc/noVNC/archive/v1.1.0.zip \
    && unzip -q v1.1.0.zip \
    && rm -rf v1.1.0.zip \
    && git clone https://github.com/novnc/websockify /noVNC-1.1.0/utils/websockify

# utility
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      gettext-base \
      openbox \
      tint2 \
      pciutils \
      xterm \
    && rm -rf /var/lib/apt/lists/

# install ROS
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       curl gnupg2 lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && bash -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list' \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       ros-foxy-desktop \
    && rm -rf /var/lib/apt/lists/*

COPY xorg.conf.temp /etc/X11/
COPY run.sh /run.sh
COPY nvidia_icd.json /etc/vulkan/icd.d/
COPY Xwrapper.config /etc/X11/Xwrapper.config
RUN install -m 666 /dev/null /etc/X11/xorg.conf

ENV DISPLAY=:0

CMD ["bash", "/run.sh"]