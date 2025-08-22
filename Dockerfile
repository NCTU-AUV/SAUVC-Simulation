FROM ros:humble

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

RUN apt-get update
RUN apt-get install lsb-release gnupg -y

RUN apt-get install curl -y
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get install ignition-fortress -y

WORKDIR /root/
