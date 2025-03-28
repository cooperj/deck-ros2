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

# Create a ros user that we can run inside the container instead of root.
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME &&\
    useradd -m -d /home/$USERNAME -s /bin/bash --uid $USER_UID --gid $USER_GID $USERNAME
# own the home directory, configure perms
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME  &&\
    chmod 700 /home/$USERNAME
# Add sudo support for the non-root user
RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME  &&\
    chmod 0440 /etc/sudoers.d/$USERNAME 

# Install VirtualGL
RUN curl -L -O https://github.com/VirtualGL/virtualgl/releases/download/3.1.1/virtualgl_3.1.1_amd64.deb && \
    apt-get update && \
    apt-get -y install ./virtualgl_3.1.1_amd64.deb && \
    rm virtualgl_3.1.1_amd64.deb && rm -rf /var/lib/apt/lists/* 

RUN groupadd --gid 1002 vglusers 

ENV VGL_ISACTIVE=1
ENV VGL_FPS=25
ENV VGL_COMPRESS=0
ENV VGL_DISPLAY=egl
ENV VGL_WM=1
ENV VGL_PROBEGLX=0
ENV LD_PRELOAD=/usr/lib/libdlfaker.so:/usr/lib/libvglfaker.so

# Add user groups
RUN usermod -a -G dialout $USERNAME &&\
    usermod -a -G video $USERNAME &&\
    usermod -a -G audio $USERNAME &&\
    usermod -a -G plugdev $USERNAME &&\
    usermod -a -G staff $USERNAME &&\
    usermod -a -G sudo $USERNAME &&\
    usermod -a -G input $USERNAME &&\
    usermod -a -G vglusers $USERNAME

# The vendor_base stage sets up the base image and includes additional Dockerfiles
# for various dependencies that are not ROS packages. 
# This stage is used to build a foundation with all
# necessary libraries and tools required.
FROM base AS vendor_base

# setup glog (google log): Adds the Google Logging library setup from the specified Dockerfile.
RUN mkdir -p /tmp/vendor && cd /tmp/vendor && wget -c https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz  -O glog-0.6.0.tar.gz &&\
    tar -xzvf glog-0.6.0.tar.gz &&\
    cd glog-0.6.0 &&\
    mkdir build && cd build &&\
    cmake .. && make -j4 &&\
    sudo make install &&\
    sudo ldconfig &&\
    cd ../.. && rm -r glog-*

# setup magic_enum: Adds the Magic Enum library setup from the specified Dockerfile.
RUN mkdir -p /tmp/vendor && cd /tmp/vendor && wget -c https://github.com/Neargye/magic_enum/archive/refs/tags/v0.8.0.tar.gz -O  magic_enum-0.8.0.tar.gz &&\
    tar -xzvf magic_enum-0.8.0.tar.gz &&\
    cd magic_enum-0.8.0 &&\
    mkdir build && cd build &&\
    cmake .. && make -j4 &&\
    sudo make install &&\
    sudo ldconfig &&\
    cd ../.. && rm -r magic_enum*   

# Setup Zenoh bridge
ENV ZENOH_BRIDGE_VERSION=1.2.1
RUN cd /tmp; \
    curl -L -O https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/${ZENOH_BRIDGE_VERSION}/zenoh-plugin-ros2dds-${ZENOH_BRIDGE_VERSION}-x86_64-unknown-linux-gnu-standalone.zip; \
    unzip zenoh-plugin-ros2dds-*.zip && \
    mv zenoh-bridge-ros2dds /usr/local/bin/ && \
    chmod +x /usr/local/bin/zenoh-bridge-ros2dds && \
    ldconfig && \
    rm -rf zenoh-*

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


# install src dependencies
RUN mkdir -p /opt/ros/coops/src
COPY ./src /opt/ros/coops/src

RUN . /opt/ros/humble/setup.sh && \
    apt update && \
    rosdep --rosdistro=${ROS_DISTRO} update && \
    cd /opt/ros/coops/src && \
    vcs pull && \
    rosdep install --from-paths . -i -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN cd /opt/ros/coops; colcon build && \
    rm -rf /opt/ros/coops/src/ /opt/ros/coops/build/ /opt/ros/coops/log/

# now also copy in all sources and build and install them
FROM depbuilder AS compiled

# Switch to the ros user and then configure the environment
USER ros

# Add a custom prompt, tmux configuration and source ros install
RUN echo "export PS1='\[\e[0;33m\]deck-ros2 ➜ \[\e[0;32m\]\u@\h\[\e[0;34m\]:\w\[\e[0;37m\]\$ '" >> ~/.bashrc

# setup tmule 
RUN pip3 install tmule
RUN echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
RUN echo 'export PATH="/usr/games:$PATH"' >> ~/.bashrc

# sort out dotfiles
COPY ./.docker/tmux.conf /home/ros/.tmux.conf
RUN echo "alias cls=clear" >> ~/.bashrc
RUN echo "alias spheres=/opt/VirtualGL/bin/glxspheres64" >> ~/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/coops/install/setup.bash" >> ~/.bashrc

WORKDIR /home/ros/ws
ENV SHELL=/bin/bash