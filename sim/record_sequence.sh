#!/bin/bash
# Record one sequence to an MCAP rosbag (guide section 4.5).
#   sim/record_sequence.sh <sequence>      e.g. office_loop_01
# Gazebo starts paused; it is un-paused only once the bridge and the recorder are up,
# so the bag starts at sim time 0.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/env.sh"
set -u
seq=$1
world="$REPO/sim/worlds/$seq.sdf"; route="$REPO/sim/routes/$seq.yaml"
out="$REPO/data/sequences/$seq"
[ -f "$world" ] && [ -f "$route" ] || { echo "unknown sequence $seq"; exit 1; }
[ -e "$out/bag" ] && { echo "$out/bag exists - remove it first to re-record"; exit 1; }
mkdir -p "$out/logs"
wname=$(grep -o '<world name="[^"]*"' "$world" | cut -d'"' -f2)

pids=()
cleanup() { for p in "${pids[@]}"; do kill -TERM "$p" 2>/dev/null; done; sleep 3
            for p in "${pids[@]}"; do pkill -9 -P "$p" 2>/dev/null; kill -9 "$p" 2>/dev/null; done; }
trap cleanup EXIT

gz sim -s --headless-rendering "$world" > "$out/logs/gz.log" 2>&1 &
pids+=($!)
/opt/ros/lyrical/lib/ros_gz_bridge/parameter_bridge --ros-args -p config_file:="$REPO/sim/launch/bridge.yaml" \
    > "$out/logs/bridge.log" 2>&1 &
pids+=($!)
sleep 8
ros2 bag record -s mcap -o "$out/bag" --max-cache-size 2000000000 --topics \
    /clock /ms/rgb/image /ms/depth/image /ms/thermal/image \
    /ms/rgb/camera_info /ms/depth/camera_info /ms/thermal/camera_info \
    /ms/ground_truth /odom /cmd_vel > "$out/logs/record.log" 2>&1 &
rec=$!
sleep 5
kill -0 $rec 2>/dev/null || { echo "recorder failed:"; cat "$out/logs/record.log"; exit 1; }
gz service -s "/world/$wname/control" --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
    --timeout 5000 --req 'pause: false' > /dev/null
python3 "$REPO/sim/routes/drive_route.py" "$route" 2>&1 | tee "$out/logs/driver.log"
kill -TERM $rec; wait $rec 2>/dev/null
cp "$world" "$route" "$REPO/sim/models/tb3_waffle_ms/sensors.yaml" "$out/"
{ echo "sequence: $seq"; echo "recorded: $(date -Is)"; echo "gz_sim: $(gz sim --versions | head -1)"
  echo "ros_distro: $ROS_DISTRO"; echo "repo_commit: $(git -C "$REPO" rev-parse HEAD 2>/dev/null || echo uncommitted)"
  echo "repo_dirty: $(git -C "$REPO" status --porcelain 2>/dev/null | wc -l)"
  echo "tb3_models_commit: $(git -C "$REPO/third_party/turtlebot3_simulations" rev-parse HEAD)"
  echo "world_sha256: $(sha256sum "$world" | cut -d' ' -f1)"; } > "$out/recording.yaml"
ros2 bag info "$out/bag" | tee "$out/logs/bag_info.txt"
