FROM ros:iron

COPY . ./src/
WORKDIR src

RUN apt update
RUN apt install -y ros-iron-ros2-control
RUN apt install -y vim
RUN apt install -y ros-iron-ament-cmake
RUN . /opt/ros/${ROS_DISTRO}/setup.sh
RUN rosdep install -i -y --from-paths .; return 0
RUN rosdep install -i -y --from-paths .; return 0
#RUN export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/ros/iron
RUN /bin/bash -c "source /opt/ros/iron/setup.bash && \
    export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/ros/iron && \
    colcon build"
#RUN colcon build
#RUN . /opt/ros/${ROS_DISTRO}/setup.sh && . install/setup.sh
RUN touch /root/.bashrc
RUN echo "source /src/install/setup.bash" >> /root/.bashrc
RUN echo "source /opt/ros/iron/setup.bash" >> /root/.bashrc
