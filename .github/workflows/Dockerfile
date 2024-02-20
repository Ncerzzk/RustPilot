FROM ubuntu:22.04

RUN apt-get install wget
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' 
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - 
RUN apt-get update

RUN apt-get install libgz-sim8-dev

