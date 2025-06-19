FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ROS 2 keys & sources
RUN apt update && apt install -y curl gnupg lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/ros2.list

RUN apt update && apt install -y ros-humble-desktop ros-dev-tools

# Sprache und Build Tools
RUN apt install -y build-essential cmake git python3-pip python3-colcon-common-extensions
RUN apt install -y python3-flask
RUN apt install -y libomp-dev
RUN apt install -y git

#COPY ./flask_web_interface/ /home/ws/src/flask_web_interface/
#COPY ./fs_backend/ /home/ws/src/fs_backend/

WORKDIR /tmp/
RUN git clone https://github.com/DominikSlomma/FocusStacking

RUN mkdir -p /home/ws/src/flask_web_interface/
RUN mkdir -p /home/ws/src/fs_backend/
RUN mkdir -p /home/ws/

RUN cp -r /tmp/FocusStacking/flask_web_interface/* /home/ws/src/flask_web_interface/
RUN cp -r /tmp/FocusStacking/fs_backend/* /home/ws/src/fs_backend/

RUN cp /tmp/FocusStacking/update.sh /home/ws/


RUN rm -r  /tmp/FocusStacking

WORKDIR /home/ws

# Workspace bauen
RUN source /opt/ros/humble/setup.bash && colcon build

#RUN install/setup.bash


# ROS Setup in Bashrc eintragen (für interaktive shells)
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /home/ws/install/setup.bash" >> ~/.bashrc

# Start-Shell mit Workspace Setup (optional)
#CMD ["/bin/bash", "-c", "source /home/ws/install/setup.bash && exec bash"]

# Container-Startbefehl: Environment sourcen, dann dein Befehl ausführen
CMD source /opt/ros/humble/setup.bash && source /home/ws/install/setup.bash && ros2 run flask_web_interface web_server
