# Based on https://hub.docker.com/r/luxonis/depthai-library
FROM ubuntu:20.04

ENV DEBIAN_FRONTEND noninteractive
RUN apt update && \
    apt install -y --no-install-recommends \
        apt-transport-https apt-utils autoconf automake autopoint bash bash-completion bison \
        build-essential ca-certificates cmake coreutils cpio curl ffmpeg \
        gir1.2-gst-rtsp-server-1.0 git gstreamer1.0-plugins-bad less libavcodec-dev \
        libavformat-dev libswscale-dev libdc1394-22-dev libgl1-mesa-glx libgtk2.0-dev libjpeg-dev \
        libpng-dev libsm6 libtbb2 libtbb-dev libtiff-dev libtool libxext6 pkg-config python3-dev \
        python3-gst-1.0 software-properties-common ssh sudo tzdata udev unzip vim wget zip && \
    wget https://github.com/libusb/libusb/releases/download/v1.0.24/libusb-1.0.24.tar.bz2 && \
    tar xf libusb-1.0.24.tar.bz2 && \
    cd libusb-1.0.24 && \
    ./configure --disable-udev && \
    make -j && sudo make install && \
    cd .. && rm -rf libusb-1.0.24* && \
    ln -sf /usr/share/zoneinfo/US/Pacific-New /etc/localtime && \
    echo America/Los_Angeles > /etc/timezone && \
    dpkg-reconfigure --frontend noninteractive tzdata && \
    wget https://bootstrap.pypa.io/get-pip.py && \
    python3 get-pip.py && rm -f get-pip.py && \
    pip install absl-py numpy opencv-python && \
    ln -s python3 /usr/bin/python

ARG NAME
ARG UID
ARG GID
RUN groupadd -g $GID $NAME && \
    useradd -u $UID -g $NAME --groups plugdev,sudo --shell /bin/bash $NAME && \
    echo "$NAME ALL = NOPASSWD: ALL" > /etc/sudoers.d/$NAME && \
    mkdir /home/$NAME && chown -R $NAME:$NAME /home/$NAME

USER $NAME
WORKDIR /home/$NAME
RUN git clone https://github.com/luxonis/depthai-python -b v2.10.0.0 && \
    cd depthai-python && git submodule update --init --recursive && \
    mkdir -p build && cd build && cmake .. && make -j && \
    sudo cp depthai.cpython-38-x86_64-linux-gnu.so /usr/local/lib/python3.8/dist-packages/
