FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=venky
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN if id -u $USER_UID >/dev/null 2>&1; then \
        existing_user=$(id -nu $USER_UID); \
        usermod -l $USERNAME $existing_user; \
        groupmod -n $USERNAME $(id -ng $USER_UID); \
        usermod -d /home/$USERNAME -m $USERNAME; \
    else \
        groupadd --gid $USER_GID $USERNAME && \
        useradd --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME



RUN apt-get update && apt-get install -y \
    build-essential \
    python3-pip \
    python3-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-urdf-tutorial \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-controller-manager \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2controlcli \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --break-system-packages \
    qpsolvers \
    osqp \
    cvxopt \
    scs

USER $USERNAME
WORKDIR /robot_dynamics

RUN rosdep update

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
