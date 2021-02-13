FROM ros:foxy-ros-base

RUN apt-get update \
    apt-get install -y vim byobu git python3-pip
RUN pip3 install asciimatics
