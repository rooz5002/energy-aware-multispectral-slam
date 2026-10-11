# Source this before running anything in the simulation/recording stack.
source /opt/ros/lyrical/setup.bash
# ROS C++ nodes that also load gz-transport (ros_gz_bridge) deadlock at start-up
# on this host while registering with the running lttng-sessiond; do not wait for it.
export LTTNG_UST_REGISTER_TIMEOUT=0
# Keep gz-transport and DDS discovery on this machine only (several VPN interfaces present).
export GZ_IP=127.0.0.1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GZ_SIM_RESOURCE_PATH="$REPO_ROOT/sim/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
