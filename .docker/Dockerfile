ARG ROS_DISTRO=jazzy
ARG GENERATE_PYTHON_STUBS=ON

# Preserve rosdep installs separate from source. Maintains caches as long as rosdeps do not change.
FROM alpine:latest AS package-manifests

COPY . /roboplan_ros
RUN find /roboplan_ros -type f ! -name "package.xml" ! -name "COLCON_IGNORE" -delete && \
    find /roboplan_ros -type d -empty -delete

FROM ros:${ROS_DISTRO} AS roboplan_ros

ENV PIP_BREAK_SYSTEM_PACKAGES=1
SHELL ["/bin/bash", "-c"]

# Install core dependencies.
RUN apt update -y \
    && apt upgrade -y \
    && apt install -y libgmock-dev python3-pip

# Install Python dependencies needed for bindings.
# Note that since Pinocchio in ROS is built with Numpy 1.x on Ubuntu 24.04 and older,
# we must preserve this by pinning a bunch of other versions that vary per distro.
COPY .docker/ros_python_requirements_${ROS_DISTRO}.txt /tmp/ros_python_requirements.txt
RUN pip3 install -r /tmp/ros_python_requirements.txt

# Create a ROS 2 workspace and copy in the source code.
RUN mkdir -p /workspace/roboplan_ws/src/roboplan_ros
WORKDIR /workspace/roboplan_ws

# Copy package manifests for installing rosdeps
COPY --from=package-manifests /roboplan_ros ./src/roboplan_ros
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt update -y \
    && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y

# Copy in remaining source and build
COPY . src/roboplan_ros
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --cmake-args -DBUILD_TESTING=ON -DGENERATE_PYTHON_STUBS=${GENERATE_PYTHON_STUBS}

# Set up entrypoint and working folder.
WORKDIR /workspace/roboplan_ws
COPY ./.docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
RUN echo "source /entrypoint.sh" >> ~/.bashrc
