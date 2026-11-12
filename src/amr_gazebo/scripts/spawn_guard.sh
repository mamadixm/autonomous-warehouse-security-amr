#!/usr/bin/env bash
# ==============================================================================
# Helper Script: spawn_guard.sh
# Spawns an Authorized Security Officer (High-Vis Green Vest) into Gazebo Harmonic
# to test AUTHORIZED_SECURITY classification and RFID badge integration.
#
# Usage:
#   ./spawn_guard.sh [X] [Y] [Z]
#   ./spawn_guard.sh delete
#
# Examples:
#   ./spawn_guard.sh 1.5 0.0 0.35      # Spawns 1.5m in front of robot
#   ./spawn_guard.sh -1.5 0.0 0.35     # Spawns 1.5m behind robot (rear camera test)
#   ./spawn_guard.sh delete            # Removes the guard
# ==============================================================================

TARGET_NAME="authorized_guard_target"
WORLD_NAME="gazebo_world"

if [ "$1" == "delete" ] || [ "$1" == "remove" ]; then
    echo "[INFO] Removing authorized guard '$TARGET_NAME' from Gazebo world '$WORLD_NAME'..."
    gz service -s /world/${WORLD_NAME}/remove \
        --reqtype gz.msgs.Entity \
        --reptype gz.msgs.Boolean \
        --timeout 3000 \
        --req "name: \"${TARGET_NAME}\", type: MODEL"
    exit 0
fi

X="${1:-1.5}"
Y="${2:-0.0}"
Z="${3:-0.35}"

echo "[INFO] Spawning authorized security officer (High-Vis Green Vest) at X=$X, Y=$Y, Z=$Z..."

GUARD_SDF="<?xml version='1.0'?>
<sdf version='1.8'>
  <model name='${TARGET_NAME}'>
    <static>false</static>
    <link name='body'>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.2</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.2</iyy><iyz>0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <cylinder>
            <radius>0.20</radius>
            <length>0.70</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>0.20</radius>
            <length>0.70</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.10 0.95 0.20 1.0</ambient>
          <diffuse>0.10 0.95 0.20 1.0</diffuse>
          <specular>0.20 0.80 0.20 1.0</specular>
          <emissive>0.05 0.30 0.05 1.0</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>"

# Spawn via ros_gz_sim create tool
ros2 run ros_gz_sim create \
    -name "${TARGET_NAME}" \
    -string "${GUARD_SDF}" \
    -x "${X}" \
    -y "${Y}" \
    -z "${Z}"

# Publish simulated RFID badge to whitelist the officer (timeout/wait 0 so it does not block)
ros2 topic pub --once -w 0 /security/authorized_beacon std_msgs/msg/String "{data: '{\"officer_id\": \"SEC-042\", \"name\": \"Officer Davis\", \"status\": \"AUTHORIZED\"}'}" > /dev/null 2>&1 &

echo "[SUCCESS] Authorized Security Officer spawned at ($X, $Y, $Z) with active RFID badge SEC-042."
echo "[TIP] To remove: $0 delete"
