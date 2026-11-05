###############################
# base stage, common to all targets
###############################

FROM osrf/ros:jazzy-desktop AS base

ARG USERNAME=orca5
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG RENDER_GID=992
ARG DEBIAN_FRONTEND=noninteractive

# Use bash as the default shell
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/bin/bash"]

RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get -y --quiet --no-install-recommends install \
    bash-completion \
    build-essential \
    git \
    gnupg \
    iputils-ping \
    lsb-release \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    python3-venv \
    software-properties-common \
    sudo \
    wget \
    vim \
  && rm -rf /var/lib/apt/lists/*

# Create a non-root user (remove default 'ubuntu' user at UID 1000 first)
# Also ensure render GID exists to avoid 'groups: cannot find name for group ID ...'
RUN (userdel -r ubuntu 2>/dev/null || true) \
  && (groupmod -g $RENDER_GID render 2>/dev/null || groupadd -f -g $RENDER_GID render) \
  && groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && echo -e "\n# Added by orca5 Dockerfile:" >> /home/$USERNAME/.bashrc \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

# Switch to our new user
USER $USERNAME
ENV USER=$USERNAME

# Create colcon workspace and the mount point for orca5
WORKDIR /home/$USERNAME/colcon_ws/src
RUN mkdir -p orca5

# Get common repos and copy orca5 package.xml files for rosdep
COPY --chown=$USER_UID:$USER_GID common.repos /tmp/common.repos
COPY --chown=$USER_UID:$USER_GID orca_bridge/package.xml orca5/orca_bridge/
COPY --chown=$USER_UID:$USER_GID orca_bringup/package.xml orca5/orca_bringup/
COPY --chown=$USER_UID:$USER_GID orca_msgs/package.xml orca5/orca_msgs/
RUN vcs import --recursive < /tmp/common.repos \
  && rm /tmp/common.repos

# Run rosdep over all repos
# Fun fact: rosdep will automatically elevate permissions, but apt-get will not, so it requires sudo
WORKDIR /home/$USERNAME/colcon_ws
RUN sudo apt-get update \
  && rosdep update \
  && rosdep install -y --from-paths . --ignore-src \
  && sudo rm -rf /var/lib/apt/lists/*

# Build everything so far (ignore orca5 until mounted at runtime)
RUN touch src/orca5/COLCON_IGNORE \
  && source /opt/ros/jazzy/setup.bash \
  && colcon build --event-handlers console_direct+

# Set up the Python environment
COPY --chown=$USER_UID:$USER_GID requirements.txt /tmp/requirements.txt
RUN python3 -m venv venv --system-site-packages \
  && touch venv/COLCON_IGNORE \
  && source venv/bin/activate \
  && pip install --no-cache-dir -r /tmp/requirements.txt \
  && rm /tmp/requirements.txt

ENV VIRTUAL_ENV="/home/$USERNAME/colcon_ws/venv"
ENV PATH="$VIRTUAL_ENV/bin:${PATH}"

# Configure the shell environment
RUN echo "source /home/$USERNAME/colcon_ws/install/setup.bash" >> /home/$USERNAME/.bashrc \
  && echo "source /home/$USERNAME/colcon_ws/venv/bin/activate" >> /home/$USERNAME/.bashrc

###############################
# sim target
###############################

FROM base AS sim

# Switch to root to install stuff
USER root

# Add packages.osrfoundation.org and OSRF's rosdep rules for Gazebo Harmonic
# Ref https://github.com/ArduPilot/ardupilot_gazebo#Rosdep
RUN wget -qO /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg https://packages.osrfoundation.org/gazebo.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
  && wget -qO /etc/ros/rosdep/sources.list.d/00-gazebo.list https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list

# Install Gazebo Harmonic, ardupilot_sitl, and ardupilot_gazebo prereqs
# Micro-XRCE-DDS-Gen needs openjdk-17, but jazzy-desktop has openjdk-21
RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    ros-jazzy-ros-gz \
    openjdk-17-jdk \
    python3-pexpect \
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
  && update-java-alternatives -s "java-1.17.0-openjdk-$(dpkg --print-architecture)" \
  && rm -rf /var/lib/apt/lists/*

# Install Micro-XRCE-DDS-Gen, required by ArduPilot AP_DDS
RUN git clone --depth 1 --recurse-submodules --branch v4.7.0 https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git /opt/Micro-XRCE-DDS-Gen \
  && cd /opt/Micro-XRCE-DDS-Gen \
  && ./gradlew assemble \
  && chmod -R a+rX /opt/Micro-XRCE-DDS-Gen \
  && ln -s /opt/Micro-XRCE-DDS-Gen/scripts/microxrceddsgen /usr/local/bin/microxrceddsgen \
  && rm -rf ~/.gradle
ENV PATH="/opt/Micro-XRCE-DDS-Gen/scripts:${PATH}"

# Switch back to new user
USER $USERNAME

# Build for Gazebo Harmonic, define this before building ardupilot_gazebo
ENV GZ_VERSION=harmonic

# Get sim repos and copy sim-only orca5 package.xml files for rosdep
WORKDIR /home/$USERNAME/colcon_ws/src
COPY --chown=$USER_UID:$USER_GID sim.repos /tmp/sim.repos
COPY --chown=$USER_UID:$USER_GID orca_gz/package.xml orca5/orca_gz/
COPY --chown=$USER_UID:$USER_GID orca_test/package.xml orca5/orca_test/
RUN vcs import --recursive < /tmp/sim.repos \
  && rm /tmp/sim.repos \
  && cd ardupilot \
  && git submodule update --init --recursive

# Run rosdep over sim repos
WORKDIR /home/$USERNAME/colcon_ws
RUN rm -f src/orca5/COLCON_IGNORE \
  && sudo apt-get update \
  && rosdep update \
  && rosdep install -y --from-paths . --ignore-src --skip-keys microxrcedds_agent \
  && sudo rm -rf /var/lib/apt/lists/* \
  && touch src/orca5/COLCON_IGNORE

# Build everything so far
# Subsequent ardupilot_sitl builds will re-link all binaries, touch COLCON_IGNORE to avoid this
RUN source /opt/ros/jazzy/setup.bash \
  && colcon build --event-handlers console_direct+ \
  && touch src/ardupilot/COLCON_IGNORE

# Configure the shell environment
ENV PATH="/home/$USERNAME/colcon_ws/install/ardupilot_sitl/bin:${PATH}"

###############################
# hw target (assumes CPU)
###############################

FROM base AS hw

# Switch to root to install stuff
USER root

# Install gscam2 prereqs
RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x \
  && rm -rf /var/lib/apt/lists/*

# Switch back to new user
USER $USERNAME

# Get hw repos
WORKDIR /home/$USERNAME/colcon_ws/src
COPY --chown=$USER_UID:$USER_GID hw.repos /tmp/hw.repos
RUN vcs import --recursive < /tmp/hw.repos \
  && rm /tmp/hw.repos

# Run rosdep over hw repos
WORKDIR /home/$USERNAME/colcon_ws
RUN sudo apt-get update \
  && rosdep install -y --from-paths . --ignore-src \
  && sudo rm -rf /var/lib/apt/lists/*

# Build everything so far
RUN source /opt/ros/jazzy/setup.bash \
  && colcon build --event-handlers console_direct+
