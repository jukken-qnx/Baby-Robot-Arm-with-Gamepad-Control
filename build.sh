#!/bin/bash
#
# Copyright (c) 2026, BlackBerry Limited. All rights reserved.
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
#

set -e
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG

# --- ROS2 Installation Discovery ---
if [ -f "/opt/ros/jazzy/local_setup.bash" ]; then
    ROS2_HOST_INSTALLATION_PATH=/opt/ros/jazzy
    echo "Found ROS2 Installation in $ROS2_HOST_INSTALLATION_PATH"
else
    echo "Failed to find ROS2 in expected locations, please run 'sudo apk add ros2-jazzy'"
    exit 1
fi

# Source the ROS2 environment for cross-compilation
. "${ROS2_HOST_INSTALLATION_PATH}/local_setup.bash"

echo "--- ROS Environment Variables ---"
printenv | grep "ROS"
echo "-------------------------------"

# --- Build Execution ---
colcon build --merge-install --cmake-force-configure

rc=$?
if [ $rc -eq 0 ]; then
    echo "Success"
else
    echo "Error: $rc"
    exit $rc
fi

echo " "
