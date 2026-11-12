#!/usr/bin/env bash
# ==============================================================================
# Helper Script: spawn_intruder.sh
# Spawns a high-visibility red anomaly/intruder target into Gazebo Harmonic
# to test and record the DETECT -> ALERT/INTERCEPT FSM pipeline.
#
# Usage:
#   ./spawn_intruder.sh [X] [Y] [Z]
#   ./spawn_intruder.sh delete
#
# Examples:
#   ./spawn_intruder.sh 1.5 0.0 0.35      # Spawns 1.5m in front of base
#   ./spawn_intruder.sh delete            # Removes the intruder
# ==============================================================================

TARGET_NAME="red_intruder_target"
WORLD_NAME="gazebo_world"

if [ "$1" == "delete" ] || [ "$1" == "remove" ]; then
    echo "[INFO] Removing intruder '$TARGET_NAME' from Gazebo world '$WORLD_NAME'..."
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

echo "[INFO] Spawning bright red intruder target at X=$X, Y=$Y, Z=$Z..."

INTRUDER_SDF="<?xml version='1.0'?>
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
          <ambient>0.95 0.05 0.05 1.0</ambient>
          <diffuse>0.95 0.05 0.05 1.0</diffuse>
          <specular>0.8 0.1 0.1 1.0</specular>
          <emissive>0.25 0.0 0.0 1.0</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>"

# Spawn via ros_gz_sim create tool
ros2 run ros_gz_sim create \
    -name "${TARGET_NAME}" \
    -string "${INTRUDER_SDF}" \
    -x "${X}" \
    -y "${Y}" \
    -z "${Z}"

echo "[SUCCESS] Red intruder spawned successfully at ($X, $Y, $Z)."
echo "[TIP] To remove: $0 delete"
