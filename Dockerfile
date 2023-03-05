FROM osrf/ros:noetic-desktop-focal

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y apt-utils curl wget git bash-completion build-essential sudo && rm -rf /var/lib/apt/lists/*

# Now create the same user as the host itself
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} daniel
RUN adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} daniel
RUN usermod -a -G dialout daniel
RUN mkdir config && echo "ros ALL=(ALL) NOPASSWD: ALL" > config/99_aptget
RUN cp config/99_aptget /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

# Change HOME environment variable
ENV HOME /home/daniel
RUN mkdir -p ${HOME}/ros_ws/src

# Initialize the workspace
RUN cd ${HOME}/ros_ws/src && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin_init_workspace"
RUN cd ${HOME}/ros_ws /bin/bash -c "source source /opt/ros/${ROS_DISTRO}/setup.bash; catkin_make"

# set up environment
COPY ./update_bashrc /sbin/update_bashrc
RUN sudo chmod +x /sbin/update_bashrc ; sudo chown ros /sbin/update_bashrc ; sync ; /bin/bash -c /sbin/update_bashrc ; sudo rm /sbin/update_bashrc

# Install pip
RUN apt-get install -y curl
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
RUN python3.8 get-pip.py

# Install rospkg & empy
RUN pip3 install --target=/opt/ros/noetic/lib/python3/dist-packages rospkg
RUN pip3 install empy

# Compile cv_bridge
RUN cd ${HOME}/ros_ws/src/ & git clone -b melodic https://github.com/ros-perception/vision_opencv.git
RUN cd ${HOME}/ros_ws/ /bin/bash -c "source source /opt/ros/${ROS_DISTRO}/setup.bash; catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.8m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8m.so"

# Put python3 as python
RUN apt update
RUN apt install python-is-python3

# Install pytorch
RUN pip install torch==1.10.0+cu111 torchvision==0.11.0+cu111 torchaudio==0.10.0 -f https://download.pytorch.org/whl/torch_stable.html

# Install pybullet, gym and graphic interface
RUN pip3 install pybullet
RUN pip3 install gym
RUN apt-get update
RUN apt-get install libgl1-mesa-glx

ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update
RUN apt-get -y install \
    libcanberra-gtk-module \
    libcanberra-gtk3-module
RUN apt-get clean

# Install opencv-python
RUN pip3 install opencv-python

# Install tkinter
RUN apt install -y python3-tk

# Install open3d
RUN pip install --ignore-installed PyYAML
RUN pip3 install open3d

# Copy dataset to pybullet
COPY ./pybullet_URDF_objects/ /usr/local/lib/python3.8/dist-packages/pybullet_data/yale_dataset/

# Install attrdict
RUN pip install attrdict

# Give permissions
RUN chown -R 1000:1000 ./../home/daniel/ros_ws/

RUN export CUDA_VISIBLE_DEVICES=[0]

# Allow use of NVIDIA card
# ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
# ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# For the other tutorial
#COPY environment.py /workspace/
COPY ur5e /opt/conda/lib/python3.8/site-packages/pybullet_data/ur5e/
COPY blue_ur5e_2fgripper /opt/conda/lib/python3.8/site-packages/pybullet_data/blue_ur5e_2fgripper/


# Install gymnasium
RUN pip3 install gymnasium

# Install Robotic Toolbox
RUN pip3 install roboticstoolbox-python