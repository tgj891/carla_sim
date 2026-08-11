#!/usr/bin/env bash

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

ARCH=$(uname -m)

addgroup --gid "$DOCKER_GRP_ID" "$DOCKER_GRP"
adduser --disabled-password --force-badname --gecos '' "$DOCKER_USER" \
    --uid "$DOCKER_USER_ID" --gid "$DOCKER_GRP_ID" 2>/dev/null
usermod -aG sudo "$DOCKER_USER"
gpasswd -a ${DOCKER_USER} root

echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
cp -r /etc/skel/. /home/${DOCKER_USER}

if [ "$ARCH" == 'aarch64' ]; then
  echo "
export PATH=\$PATH:/usr/lib/java/bin:/usr/local/miniconda2/bin/:/usr/local/cuda/bin
export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib:/usr/lib/aarch64-linux-gnu/tegra:/usr/local/ipopt/lib:/usr/local/cuda/lib64/stubs:/opt/MVS/lib/aarch64
export NVBLAS_CONFIG_FILE=/usr/local/cuda
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
source /opt/ros/melodic/setup.bash
ulimit -c unlimited
cd /opt/v2x/catkin_ws
source devel/setup.bash" >> /home/${DOCKER_USER}/.bashrc
  source /home/${DOCKER_USER}/.bashrc
else
  echo '
export PATH=${PATH}:/usr/lib/java/bin:/home/tgj/miniconda3/bin
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib:/usr/local/ipopt/lib:/usr/local/cuda/lib64
export NVBLAS_CONFIG_FILE=/usr/local/cuda
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
export QT_X11_NO_MITSHM=1
source /opt/ros/melodic/setup.sh
source /opt/carla_ws/ros_bridge_ws/devel/setup.bash
ulimit -c unlimited
cd /opt/carla_ws' >> /home/${DOCKER_USER}/.bashrc
  source /home/${DOCKER_USER}/.bashrc
fi
 echo '
genhtml_branch_coverage = 1
lcov_branch_coverage = 1
' > "/home/${DOCKER_USER}/.lcovrc"

#chown -R ${DOCKER_USER}:${DOCKER_GRP} "/home/${DOCKER_USER}"

# check /dev/nv*
if [ "$ARCH" == 'aarch64' ]; then
  chmod a+rw /dev/nv*

  if [ -e "/usr/lib/aarch64-linux-gnu/tegra/libGL.so" ]; then
    rm /usr/lib/aarch64-linux-gnu/libGL.so
    ln -s /usr/lib/aarch64-linux-gnu/tegra/libGL.so /usr/lib/aarch64-linux-gnu/libGL.so
  elif [ -e "/usr/lib/aarch64-linux-gnu/tegra/libGLX_nvidia.so.0" ]; then
    rm /usr/lib/aarch64-linux-gnu/libGL.so
    ln -s /usr/lib/aarch64-linux-gnu/tegra/libGLX_nvidia.so.0 /usr/lib/aarch64-linux-gnu/libGL.so
  fi

  if [ -e "/usr/lib/aarch64-linux-gnu/tegra/libGLX_nvidia.so.0" ]; then
    rm /usr/lib/aarch64-linux-gnu/libGL.so.1
    ln -s /usr/lib/aarch64-linux-gnu/tegra/libGLX_nvidia.so.0 /usr/lib/aarch64-linux-gnu/libGL.so.1
  fi
fi
