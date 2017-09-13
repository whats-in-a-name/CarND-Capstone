FROM osrf/ros:kinetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -yq --no-install-recommends apt-utils ros-kinetic-controller-manager ros-kinetic-gazebo-ros-control ros-kinetic-joint-state-controller ros-kinetic-effort-controllers
RUN apt-get install -yq --no-install-recommends sudo wget python-pip
RUN pip install --upgrade pip
RUN pip install wheel
RUN pip install attrdict eventlet flask-socketio
RUN wget https://bitbucket.org/DataspeedInc/dbw_mkz_ros/raw/default/dbw_mkz/scripts/sdk_install.bash
RUN yes | bash sdk_install.bash
RUN apt-get -o Dpkg::Options::="--force-confmiss" install -yq --reinstall netbase
