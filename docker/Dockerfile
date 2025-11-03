FROM osrf/ros:jazzy-desktop AS base

ARG USERNAME=orca5
ARG USER_UID=1001
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y

# Install a few handy tools
RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    bash-completion \
    build-essential \
    git \
    gnupg \
    iputils-ping \
    lsb-release \
    software-properties-common \
    sudo \
    wget \
    vim \
  && rm -rf /var/lib/apt/lists/*

# Install Gazebo Harmonic for use with ROS2 Jazzy
RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install ros-jazzy-ros-gz \
  && rm -rf /var/lib/apt/lists/*

# Install NVIDIA software
RUN apt-get update \
 && apt-get -y --quiet --no-install-recommends install \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
  && rm -rf /var/lib/apt/lists/* \
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

# Install some ardupilot and ardupilot_gazebo prereqs
RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    python3-pip \
    python3-wxgtk4.0 \
    rapidjson-dev \
    xterm \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
  && rm -rf /var/lib/apt/lists/*

# Add packages.osrfoundation.org, required by ardupilot_gazebo
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Add OSRF's rosdep rules so that rosdep will find Gazebo Harmonic keys
# Ref https://github.com/ArduPilot/ardupilot_gazebo#Rosdep
RUN wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list

# Create a non-root user
# Required for ardupilot install, but generally a good idea
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && echo "\n# Added by orca5 Dockerfile:" >> /home/$USERNAME/.bashrc \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

# Switch to our new user
USER $USERNAME
ENV USER=$USERNAME

# Clone ArduSub code
WORKDIR /home/$USERNAME
RUN git clone https://github.com/clydemcqueen/ardupilot.git --recurse-submodules \
  && cd ardupilot \
  && git checkout 9eb5dc4

# Install ArduSub prereqs (this also appends to .bashrc)
WORKDIR /home/$USERNAME/ardupilot
ENV SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1
RUN Tools/environment_install/install-prereqs-ubuntu.sh -y

# Build ArduSub
# Note: waf will capture all of the environment variables in ardupilot/.lock-waf_linux_build.
# Any change to enviroment variables will cause a re-build.
# To avoid this call sim_vehicle.py with the "--no-rebuild" option.
WORKDIR /home/$USERNAME/ardupilot
RUN modules/waf/waf-light configure --board sitl \
  && modules/waf/waf-light build --target bin/ardusub

# Build for Gazebo Harmonic, define this before building ardupilot_gazebo
ENV GZ_VERSION=harmonic

# Create colcon workspace and the mount point for orca5
WORKDIR /home/$USERNAME
RUN mkdir -p colcon_ws/src/orca5

# Get workspace repos
WORKDIR /home/$USERNAME/colcon_ws/src
COPY --chown=$USER_UID:$USER_GID workspace.repos orca5
RUN vcs import --recursive < orca5/workspace.repos

# Run rosdep over workspace repos
# Fun fact: rosdep will automatically elevate permissions, but apt-get will not, so it requires sudo
WORKDIR /home/$USERNAME/colcon_ws
RUN sudo apt-get update \
  && rosdep update \
  && rosdep install -y --from-paths . --ignore-src

# Build everything so far
WORKDIR /home/$USERNAME/colcon_ws
RUN [ "/bin/bash" , "-c" , "\
  source /opt/ros/jazzy/setup.bash \
  && colcon build --event-handlers console_direct+" ]

# Copy orca5 package.xml files
COPY --chown=$USER_UID:$USER_GID orca_bridge/package.xml src/orca5/orca_bridge/
COPY --chown=$USER_UID:$USER_GID orca_bringup/package.xml src/orca5/orca_bringup/
COPY --chown=$USER_UID:$USER_GID orca_msgs/package.xml src/orca5/orca_msgs/

# Run rosdep over orca5 package.xml files
WORKDIR /home/$USERNAME/colcon_ws
RUN sudo apt-get update \
  && rosdep update \
  && rosdep install -y --from-paths . --ignore-src

# Copy orca5 requirements.txt file
COPY --chown=$USER_UID:$USER_GID requirements.txt src/orca5/

# Set up the Python environment
RUN python3 -m venv venv --system-site-packages \
  && touch venv/COLCON_IGNORE \
  && . venv/bin/activate \
  && pip install -r src/orca5/requirements.txt

# Configure the shell environment
RUN echo "export PATH=\"/home/$USERNAME/ardupilot/build/sitl/bin:\$PATH\"" >> /home/$USERNAME/.bashrc \
  && echo "source /home/$USERNAME/colcon_ws/install/setup.bash" >> /home/$USERNAME/.bashrc

# Create a mount point for orca5 source, this will shadow the files we copied
VOLUME /home/$USERNAME/colcon_ws/src/orca5

# Use bash as the default shell
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash"]
