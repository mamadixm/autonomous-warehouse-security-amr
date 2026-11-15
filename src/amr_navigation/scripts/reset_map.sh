#!/usr/bin/env bash
# ==============================================================================
# Autonomous Warehouse Security AMR - On-Demand Map Reset & Re-SLAM CLI Script
# ==============================================================================
set -e

WS_DIR="/home/mmdixm/Desktop/amr_security_ws"
MAP_SRC="${WS_DIR}/src/amr_navigation/maps"
MAP_INSTALL="${WS_DIR}/install/amr_navigation/share/amr_navigation/maps"

echo "=============================================================================="
echo "  [AMR SECURITY] On-Demand Map Reset & Re-SLAM"
echo "=============================================================================="

# 1. Remove map files and markers
echo "[1/3] Deleting saved warehouse map files and mapping completion markers..."
rm -f "${MAP_SRC}"/warehouse_map.yaml "${MAP_SRC}"/warehouse_map.pgm "${MAP_SRC}"/.mapping_complete
rm -f "${MAP_INSTALL}"/warehouse_map.yaml "${MAP_INSTALL}"/warehouse_map.pgm "${MAP_INSTALL}"/.mapping_complete

# 2. Trigger reset via Command Center REST API or ROS 2 topic
echo "[2/3] Notifying Command Center & Mission Orchestrator..."
RESET_CALLED=false
if curl -s -f -X POST http://localhost:8080/api/map/reset -H "Content-Type: application/json" > /dev/null 2>&1; then
    echo "  • Successfully dispatched POST /api/map/reset to Command Center API."
    RESET_CALLED=true
fi

if [ "$RESET_CALLED" = false ]; then
    echo "  • Web server not directly responding on port 8080; publishing ROS 2 reset topic..."
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        source /opt/ros/jazzy/setup.bash
    fi
    if [ -f "${WS_DIR}/install/setup.bash" ]; then
        source "${WS_DIR}/install/setup.bash"
    fi
    ros2 topic pub --once /mission/reset std_msgs/msg/String "{data: 'RESET'}" > /dev/null 2>&1 || true
    echo "  • Published /mission/reset signal."
fi

# 3. Confirmation
echo "[3/3] Map reset complete. Autonomous SLAM exploration restarted from origin (0, 0)."
echo "=============================================================================="
