ARG BASE_IMAGE=lcas.lincoln.ac.uk/lcas/ros:jammy-humble

FROM ${BASE_IMAGE} AS base

# making the standard global variables available for target-specific builds
USER root
ENV DEBIAN_FRONTEND=noninteractive

# Copy the list of APT packages to be installed from the local directory to the container
COPY .docker/apt-packages.lst /tmp/apt-packages.lst

# Update the package list, upgrade installed packages, install the packages listed in apt-packages.lst,
# remove unnecessary packages, clean up the APT cache, and remove the package list to reduce image size
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -qq -y --no-install-recommends \
        `cat /tmp/apt-packages.lst` && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# The vendor_base stage sets up the base image and includes additional Dockerfiles
# for various dependencies that are not ROS packages. 
# This stage is used to build a foundation with all
# necessary libraries and tools required.
#
# - FROM base AS vendor_base: Uses the 'base' image as the starting point and names this stage 'vendor_base'.
# - INCLUDE .docker/ydlidar.dockerfile: Adds the YDLidar library setup from the specified Dockerfile.
# - INCLUDE .docker/glog.dockerfile: Adds the Google Logging library setup from the specified Dockerfile.
# - INCLUDE .docker/magic_enum.dockerfile: Adds the Magic Enum library setup from the specified Dockerfile.
# - INCLUDE .docker/uvc.dockerfile: Adds the UVC library setup from the specified Dockerfile.
FROM base AS vendor_base

# # setup glog (google log)
# RUN mkdir -p /tmp/vendor && cd /tmp/vendor && wget -c https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz  -O glog-0.6.0.tar.gz &&\
#     tar -xzvf glog-0.6.0.tar.gz &&\
#     cd glog-0.6.0 &&\
#     mkdir build && cd build &&\
#     cmake .. && make -j4 &&\
#     sudo make install &&\
#     sudo ldconfig &&\
#     cd ../.. && rm -r glog-*

# setup magic_enum
RUN mkdir -p /tmp/vendor && cd /tmp/vendor && wget -c https://github.com/Neargye/magic_enum/archive/refs/tags/v0.8.0.tar.gz -O  magic_enum-0.8.0.tar.gz &&\
    tar -xzvf magic_enum-0.8.0.tar.gz &&\
    cd magic_enum-0.8.0 &&\
    mkdir build && cd build &&\
    cmake .. && make -j4 &&\
    sudo make install &&\
    sudo ldconfig &&\
    cd ../.. && rm -r magic_enum*   

# This stage is named 'sourcefilter' and is based on the 'base' image.
# It performs the following actions:
# 1. Creates a directory /tmp/src/ to store source files.
# 2. Copies all .repos files from the ./.docker directory to /tmp/.docker/.
# 3. Changes the working directory to /tmp/src and imports repositories listed in the .repos files using vcs.
# 4. Pulls the latest changes for all imported repositories.
FROM base AS sourcefilter
RUN mkdir -p /tmp/src/
COPY ./.docker/*.repos* /tmp/.docker/
RUN cd /tmp/src && for r in /tmp/.docker/*.repos; do vcs import < $r ; done
RUN cd /tmp/src && vcs pull
# remove everything that isn't package.xml
RUN find /tmp/src -type f \! -name "package.xml" -print | xargs rm -rf

# This Dockerfile is designed to build a ROS (Robot Operating System) workspace.
# It consists of multiple stages to optimize the build process and reduce the final image size.
# 
# The stages are as follows:
# 1. depinstaller: Installs the necessary dependencies for the ROS workspace by copying a reduced source tree and using rosdep.
# 2. depbuilder: Copies additional repository and script files, imports the source tree, and builds the workspace.
# 
# The output of depbuilder is a Docker image with the ROS workspace built and ready for use.
FROM vendor_base AS depinstaller
# copy the reduced source tree (only package.xml) from previous stage
COPY --from=sourcefilter /tmp/src /tmp/src
RUN rosdep update --rosdistro=${ROS_DISTRO} && apt-get update
RUN rosdep install --from-paths /tmp/src --ignore-src -r -y && rm -rf /tmp/src && apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/src

FROM depinstaller AS depbuilder
COPY .docker/*.repos* .docker/*.sh /tmp/.docker/

RUN mkdir -p /opt/ros/lcas/src && \
    cd /opt/ros/lcas/src && \
    for r in /tmp/.docker/*.repos; do vcs import < $r ; done

RUN . /opt/ros/humble/setup.sh && \
    apt update && \
    rosdep --rosdistro=${ROS_DISTRO} update && \
    cd /opt/ros/lcas/src && \
    vcs pull && \
    rosdep install --from-paths . -i -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# build the workspace but only until limo_gazebosim to avoid building the hardware specific packages
RUN cd /opt/ros/lcas; colcon build && \
    rm -rf /opt/ros/lcas/src/ /opt/ros/lcas/build/ /opt/ros/lcas/log/

# now also copy in all sources and build and install them
FROM depbuilder AS compiled

RUN echo "source /opt/ros/lcas/install/setup.bash" >> /etc/bash.bashrc

RUN . /opt/ros/lcas/install/setup.sh && \
    apt update && \
    rosdep --rosdistro=${ROS_DISTRO} update && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# rosdep install --from-paths /opt/ros/lcas/src/local-code/src --ignore-src -y && \
    
RUN cd /opt/ros/lcas && colcon build && \
    rm -rf /opt/ros/lcas/src/ /opt/ros/lcas/build/ /opt/ros/lcas/log/

USER ros
WORKDIR /home/ros
ENV SHELL=/bin/bash