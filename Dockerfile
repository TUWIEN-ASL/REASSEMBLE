FROM osrf/ros:noetic-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    git \
    portaudio19-dev python3-all-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio \
    build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev \
    lsb-release curl \
    && apt install -y software-properties-common 

# Installing libfranka ("*libfranka*" must be removed first to avoid conflict)
RUN apt-get remove "*libfranka*" \
    && git clone --recurse-submodules https://github.com/frankaemika/libfranka.git && cd libfranka && git checkout 0.13.4 && git submodule update \
    && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. \
    && make -j$(nproc) && make install \
    && rm -rf /var/lib/apt/lists/* 
WORKDIR /

# Get iir filter library
RUN apt-get update && apt install -y ros-noetic-soem ros-noetic-ethercat-grant
RUN add-apt-repository ppa:berndporr/dsp && \
    apt install iir1 iir1-dev

# Setup rosdep
RUN rosdep update

# Create workspace directory and copy the corresponding source files
WORKDIR /root/catkin_reas
COPY ./catkin_reas/src ./src

# Install ROS package dependencies
RUN apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka

# Audio ROS package
RUN apt-get update && apt-get install -y \
    ros-noetic-audio-common ros-noetic-audio-common-msgs ros-noetic-audio-capture ros-noetic-audio-play \
    && rm -rf /var/lib/apt/lists/*

# Copy dependencies into the docker container
COPY catkin_reas/src/haptic_ros/lib/libdhd.so /asl_libs/haptic/lib/libdhd.so.3

# Omega haptic device SDK
WORKDIR /asl_libs/omega-sdk-3.17.6
RUN curl -L -o sdk.tar.gz https://downloads.forcedimension.com/sdk/sdk-3.17.6-linux-x86_64-gcc.tar.gz \
    && tar -xzf sdk.tar.gz --strip-components=1 \
    && rm sdk.tar.gz
RUN pip3 install pytransform3d
ENV PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH

# ECI USB driver installation
WORKDIR /asl_libs
RUN curl -L -o eci-linux.zip https://hmsnetworks.blob.core.windows.net/nlw/docs/default-source/products/ixxat/monitored/pc-interface-cards/eci-linux.zip \
    && python3 -c "import zipfile; zipfile.ZipFile('eci-linux.zip').extractall()" \
    && rm eci-linux.zip
COPY /asl_libs/OsEci.h /asl_libs/EciLinux_amd64/inc/OsEci.h
# RUN apt-get update && apt-get install -y \
#     udev
RUN mkdir -p /etc/udev/rules.d \
    && cd /asl_libs/EciLinux_amd64/src/KernelModule \
    && make install-usb
# RUN cp /asl_libs/EciLinux_amd64/lib/libeci113DriverLinux-usb-1.0.so.1 /usr/local/lib/ \
#     && ldconfig

# NatNet
# RUN apt-get update && apt install -y ros-$ROS_DISTRO-tf2* wget
# RUN apt-get upgrade

# Build the workspace
WORKDIR /root/catkin_reas
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build" 
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build -DCMAKE_VERBOSE_MAKEFILE=ON -j1 VERBOSE=1" 

# Source the workspace automatically
RUN echo "source /root/catkin_reas/devel/setup.bash" >> ~/.bashrc

# # Install python packages for recording
# COPY ./requirements_recording.txt /requirements_recording.txt
# RUN python3 -m pip install -r /requirements_recording.txt

# Set default working directory
WORKDIR /root/catkin_reas

CMD ["/bin/bash"]
