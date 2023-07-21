# STEP 1 - Imports ros:noetic base image, already with ROS and OpenCV 4 --------
FROM ros:noetic-perception

ARG WORK_FOLDER=/workspaces/odometry

# STEP 2 - Install dependencies ------------------------------------------------
RUN cd && apt-get update \
    && apt-get -y install \
    cmake make git nano openssh-client \
    clang g++ \
    wget unzip \
    curl lsb-release gnupg\
    libeigen3-dev libmetis5 \
    libsuitesparse-dev \
    python3-catkin-tools \
    python3-pip \
    libavcodec-dev libavformat-dev libavutil-dev \
    libglew-dev libgtk2.0-dev libgtk-3-dev libjpeg-dev libpng-dev libpostproc-dev \
    libswscale-dev libtiff5-dev libv4l-dev libxvidcore-dev \
    libx264-dev qt5-default zlib1g-dev libgl1 libglvnd-dev pkg-config \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev mesa-utils \
    python2.7-dev python3-dev \
    yasm gfortran libtbb-dev libatlas-base-dev  \
    libprotobuf-dev protobuf-compiler libgoogle-glog-dev libgflags-dev \
    libgphoto2-dev libhdf5-dev doxygen libgmp3-dev liblapack-dev libopenblas-dev \
    metis libceres-dev autoconf automake  \
    libboost-all-dev  \
    doxygen libopencv-dev \
    libpoco-dev libblas-dev \
    python3-scipy \
    python3-matplotlib ipython3 python3-tk python3-igraph python3-pyx

# # STEP ? - Install GKlib and METIS ------------------------------------------------ WARN - NOT WORKING
# RUN git clone https://github.com/KarypisLab/GKlib \
#     && cd GKlib \
#     && make config prefix=/usr/local\
#     && make \
#     && make install \
#     && cd .. 

# RUN git clone https://github.com/KarypisLab/METIS.git && cd METIS \
#     && apt-get install -y cmake-qt-gui build-essential \
#     && make config share=1 cc=gcc prefix=/usr/local \
#     && make \
#     && make install \
#     && cd .. 

# STEP ? - SuiteSparse 4.5.6  -------------------------------------------------------- WARN - NOT WORKING
# RUN git clone https://github.com/DrTimothyAldenDavis/SuiteSparse.git \
#     && cd SuiteSparse \
#     && git checkout 75753f9 \
#     && make -j8 \
#     && make install \
#     && cd

# STEP 3 - Install Ceres Solver 2.1.0 -------------------------------------------------- 
RUN curl http://ceres-solver.org/ceres-solver-2.1.0.tar.gz -o ceres-solver.tar.gz \
    && tar zxf ceres-solver.tar.gz \
    && mkdir ceres-bin \
    && cd ceres-bin \
    && cmake ../ceres-solver-2.1.0 \
    && make -j8 \                           
    && make install \
    && cd

# STEP 4 - Change default shell to bash ----------------------------------------
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# STEP 5 - Source ROS and create catkin workspace ------------------------------
RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc \
    && source ~/.bashrc \
    && source /opt/ros/noetic/setup.bash \
    && mkdir -p ~/catkin_ws/src \
    && cd ~/catkin_ws/ \
    && catkin_make \
    && cd 

# STEP 6 - Prepare ROS workspace for VINS-Fusion and mavros --------------------
RUN apt-get update -y \
    && apt-get install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-rosbag \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-mavros-extras \
    ros-${ROS_DISTRO}-mavros-msgs \
    && wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && bash ./install_geographiclib_datasets.sh  

# STEP 7 - Install mavlink -----------------------------------------------------------
RUN git clone https://github.com/mavlink/mavlink.git --recursive \ 
    && cd mavlink \
    && python3 -m pip install -r pymavlink/requirements.txt \
    && python3 -m pip install -e pymavlink \
    && python3 -m pip install matplotlib numpy pygame lxml \
    && cd


# STEP 8 - Install QGroundControl ----------------------------------------------------
ENV DEBIAN_FRONTEND=noninteractive      

RUN apt-get remove modemmanager -y \
    && apt-get install -y gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    libqt5gui5 libfuse2 

# STEP 9 - Install MAVProxy ----------------------------------------------------------
RUN apt-get install -y python3-wxgtk4.0 \
    && python3 -m pip install PyYAML mavproxy --user \
    && echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc

# STEP 10 - Install Gazebo -----------------------------------------------------------
RUN curl -sSL http://get.gazebosim.org | sh

# STEP 11 - 18 - ArduPilot ----------------------------------------------------------------
RUN apt-get install sudo && adduser --disabled-password --gecos '' weakuser \
    && adduser weakuser sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER weakuser

WORKDIR /home/weakuser

RUN git clone https://github.com/ArduPilot/ardupilot \
    && cd ardupilot \
    && git submodule update --init --recursive 

ENV USER weakuser
RUN cd ardupilot && ./Tools/environment_install/install-prereqs-ubuntu.sh -y

# STEP ? - Not Working, must run inside Docker ---------------------------------------
# RUN . ~/.profile && pwd \
#     && cd \
#     && sim_vehicle.py -w \
#     && echo 'source ${PWD}/ardupilot/Tools/completion/completion.bash' >> ~/.bashrc \
#     && cd

USER root
WORKDIR /root
ENV USER root

RUN git clone https://github.com/khancyr/ardupilot_gazebo \
    && cd ardupilot_gazebo \
    && mkdir build && cd build \
    && cmake .. \
    && make -j8 \
    && make install \
    && echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc \
    && echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc \
    && cd

RUN cd && git clone https://github.com/dronekit/dronekit-python \
    && cd dronekit-python \
    && python setup.py install \
    && cd

# STEP 19 - Install VINS-Fusion ------------------------------------------------------
# OBS: This is a modified version of VINS-Fusion, with a few changes to make it work with OpenCV 4 and Ceres 2.1.0

RUN cd ~/catkin_ws/src \
    && apt-get install -y libmetis-dev libmetis-edf-dev\
	&& git clone https://github.com/guisoares9/VINS-Fusion.git \
    && source ~/.bashrc \
    && source /opt/ros/noetic/setup.bash \
	&& cd ../ \
	&& catkin_make > /vins_log.log \
	&& source ~/catkin_ws/devel/setup.bash \
    && cd

# STEP 20 - Install Kalibr -----------------------------------------------------------
RUN mkdir ~/kalibr_workspace && cd ~/kalibr_workspace \
    && mkdir src && cd src \
    && git clone https://github.com/ethz-asl/kalibr.git \
    && cd ~/kalibr_workspace/ \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && catkin build -DCMAKE_BUILD_TYPE=Release -j8
