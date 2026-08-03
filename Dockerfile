FROM ros:humble

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# -----------------------------------------------------------------------------
# Gazebo (Ignition Fortress) apt repository
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl \
        gnupg \
        lsb-release \
 && curl https://packages.osrfoundation.org/gazebo.gpg \
        --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
 && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# 模擬器與 ROS 橋接
#
# ros_gz_sim 以前不在映像裡，是靠 `make init` 執行 rosdep install 補進
# **執行中的容器**的。因為只有 sim_ws 是 volume，容器一 recreate
# （compose down / up、換機器）這些套件就消失，launch 會直接以
# 「找不到 ros_gz_sim」失敗 —— 「映像 build 成功」不等於「系統跑得起來」。
#
# sim_ws/src 的 package.xml 宣告的相依一律裝進映像。新增相依時請同步更新
# 這裡，不要只依賴 rosdep。
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ignition-fortress \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-interfaces \
        ros-humble-ros-gz-sim \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /root/
