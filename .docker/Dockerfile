# Base image: ROS noetic Ubuntu 20.04
FROM osrf/ros:noetic-desktop-full
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt install software-properties-common -y && \
    apt-get install -y python3-pip python3-rosdep python3-catkin-tools \
    build-essential cmake git curl \
    libglfw3-dev libpoco-dev libeigen3-dev libfmt-dev lsb-release \
    portaudio19-dev python3-all-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio

# Newer Boost version must be installed for building the event camera workspace
WORKDIR /asl_libs
RUN curl -L https://archives.boost.io/release/1.86.0/source/boost_1_86_0.tar.gz | tar xz
RUN cd boost_1_86_0 && ./bootstrap.sh --prefix=/asl_libs/boost_1_86_0/install && ./b2 install --prefix=/asl_libs/boost_1_86_0/install
RUN echo "/asl_libs/boost_1_86_0/install/lib" > /etc/ld.so.conf.d/boost.conf && \
    ldconfig

# Installing libfranka ("*libfranka*" must be removed first to avoid conflict)
WORKDIR /asl_libs
RUN apt-get remove "*libfranka*" \
    && git clone --recurse-submodules https://github.com/frankaemika/libfranka.git && cd libfranka && git checkout 0.13.4 && git submodule update \
    && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. \
    && make -j$(nproc) && make install \
    && rm -rf /var/lib/apt/lists/* 
WORKDIR /

# Get iir filter library for the haptic device
RUN apt-get update && apt install -y ros-noetic-soem ros-noetic-ethercat-grant
RUN add-apt-repository ppa:berndporr/dsp && \
    apt-get install iir1 iir1-dev

# Setup rosdep
RUN rosdep update

# Audio ROS package for recording audio signals
RUN apt-get update && apt-get install -y \
    ros-noetic-audio-common ros-noetic-audio-common-msgs ros-noetic-audio-capture ros-noetic-audio-play \
    && rm -rf /var/lib/apt/lists/*

# Omega haptic device SDK
WORKDIR /asl_libs/omega-sdk
RUN curl -L -o sdk.tar.gz https://downloads.forcedimension.com/sdk/sdk-3.17.6-linux-x86_64-gcc.tar.gz \
    && tar -xzf sdk.tar.gz --strip-components=1 \
    && rm sdk.tar.gz \
    && make
RUN export LD_LIBRARY_PATH=/asl_libs/omega-sdk/lib/release/lin-x86_64-gcc:$LD_LIBRARY_PATH && ldconfig

# ECI USB driver installation for the aidin ROS package
WORKDIR /asl_libs
RUN curl -L -o eci-linux.zip https://hmsnetworks.blob.core.windows.net/nlw/docs/default-source/products/ixxat/monitored/pc-interface-cards/eci-linux.zip \
    && python3 -c "import zipfile; zipfile.ZipFile('eci-linux.zip').extractall()" \
    && rm eci-linux.zip
COPY /asl_libs/OsEci.h ./EciLinux_amd64/inc/OsEci.h
RUN apt-get update && apt-get install -y udev \
    && mkdir -p /etc/udev/rules.d \
    && cd ./EciLinux_amd64/src/KernelModule \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && make install-usb" \
    && ldconfig    

# RealSense2 camera ROS package
RUN apt-get update && apt-get install -y ros-noetic-realsense2-camera

# Event camera (DV ROS)
RUN add-apt-repository ppa:inivation-ppa/inivation && \
    apt update && \
    apt install -y python3-catkin python3-catkin-tools ros-noetic-catkin ros-noetic-camera-info-manager && \
    apt install -y dv-processing dv-runtime-dev gcc-10 g++-10
WORKDIR /asl/libs

# Create workspace directory and copy the corresponding source files
WORKDIR /root/catkin_reas
COPY catkin_reas/src src

# Copy customm joint limits
COPY asl_libs/joint_limits.yaml src/panda_moveit_config/config/

# Install ROS package dependencies
WORKDIR /root/catkin_reas
RUN apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka

ENV BOOST_ROOT=/asl_libs/boost_1_86_0/install
ENV BOOST_INCLUDEDIR=/asl_libs/boost_1_86_0/install/include
ENV BOOST_LIBRARYDIR=/asl_libs/boost_1_86_0/install/lib

# Build the workspace (build natnet_ros_cpp first)
WORKDIR /root/catkin_reas
RUN touch src/dv-ros/CATKIN_IGNORE && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build natnet_ros_cpp"
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build"
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Debug -DFranka_DIR:PATH=/libfranka/build -DCMAKE_VERBOSE_MAKEFILE=ON -j1 VERBOSE=1"

# Build event camera worksapce separately
WORKDIR /root/catkin_reas
ENV CC=/usr/bin/gcc-10
ENV CXX=/usr/bin/g++-10
RUN rm src/dv-ros/CATKIN_IGNORE && \
    # /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build --cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10" && \
    # /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Debug -DFranka_DIR:PATH=/libfranka/build -DCMAKE_VERBOSE_MAKEFILE=ON -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10 -j1 VERBOSE=1" && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build dv_ros_accumulation -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/asl_libs/libfranka/build --cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10" && \
    # /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build dv_ros_aedat4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/asl_libs/libfranka/build --cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10" && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build dv_ros_capture -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/asl_libs/libfranka/build --cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10" && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build dv_ros_imu_bias -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/asl_libs/libfranka/build --cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10" && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build dv_ros_messaging -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/asl_libs/libfranka/build --cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10" && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build dv_ros_msgs -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/asl_libs/libfranka/build --cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10" && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build dv_ros_runtime_modules -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/asl_libs/libfranka/build --cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10" && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build dv_ros_tracker -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/asl_libs/libfranka/build --cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10" && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build dv_ros_visualization -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/asl_libs/libfranka/build --cmake-args -DCMAKE_C_COMPILER=gcc-10 -DCMAKE_CXX_COMPILER=g++-10" && \
    touch src/dv-ros/CATKIN_IGNORE

# Copy haptic driver library file
# RUN cp /asl_libs/omega-sdk/lib/release/lin-x86_64-gcc/libdhd.so.3.17.6 src/haptic_ros/lib/libdhd.so.3
COPY asl_libs/libdhd.so.3 /root/catkin_reas/src/haptic_ros/lib/libdhd.so.3

# Source the workspace automatically
RUN echo "source /root/catkin_reas/devel/setup.bash" >> ~/.bashrc

# Python environent for recording script
COPY requirements_docker.txt .
RUN pip3 install -r requirements_docker.txt

# Set default working directory
WORKDIR /root/catkin_reas
CMD ["/bin/bash"]

# Copy previous calibration if needed
# COPY asl_libs/franka_ft_calib_data.yaml /root/.ros/ft_calib/franka_ft_calib_data.yaml
