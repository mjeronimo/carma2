# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# A Docker configuration script to build the CARMA2 image.
#
# The script provides the following build arguments:
#
#   BUILD_DATE  - The time and date that the docker image was created (no default value).
#   VCS_REF     - The git revision of the CARMA2 source code (no default value).
#   VERSION     - The version of CARMA2 (default: "rolling")
#   CARMA_WS    - The location for the CARMA2 source code in the container (default: /usr/local/src/carma2)

FROM ros:rolling

ARG BUILD_DATE
ARG VCS_REF
ARG VERSION="rolling"
ARG CARMA_WS=/usr/local/src/carma2_ws

# LABEL the image
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="CARMA2"
LABEL org.label-schema.description="Prototype CARMA2 platform"
LABEL org.label-schema.vendor="Open Robotics"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://openrobotics.org"
LABEL org.label-schema.vcs-url="https://github.com/mjeronimo/carma2"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

# Disable prompting during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Update the Ubuntu software repository
RUN apt-get update

# Get the CARMA2 source code
RUN mkdir -p ${CARMA_WS}/src
WORKDIR ${CARMA_WS}/src
RUN git clone https://github.com/mjeronimo/carma2
RUN vcs import < carma2/carma2.repos

# Install dependencies
RUN rosdep update
RUN rosdep install -r --from-paths . --ignore-src --rosdistro rolling -y

# Install additional tools
RUN apt-get install -y vim

# Build the CARMA2 code in the workspace
WORKDIR ${CARMA_WS}
RUN /bin/bash -c "source /opt/ros/rolling/setup.bash ; colcon build --symlink-install"

# Set up the entrypoint
COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
