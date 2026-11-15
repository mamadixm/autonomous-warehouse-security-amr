#!/usr/bin/env bash
# ==============================================================================
# LinkedIn Cinematic Demo Orchestrator
# Automates: Patrol -> Timed Threat Spawn -> Standoff & Snapshot -> Auto-Cleanup -> Resume Patrol
# ==============================================================================
set -e

WS_DIR="$HOME/Desktop/amr_security_ws"
SPAWN_SCRIPT="$WS_DIR/src/amr_gazebo/scripts/spawn_intruder.sh"

echo "======================================================================"
echo "🎬 AMR Security System - Automated LinkedIn Demo Sequence"
echo "======================================================================"
echo "💡 Layout suggestion:"
echo "   - Left half of screen : Web Browser (http://localhost:8080)"
echo "   - Right half of screen: Gazebo GUI (Warehouse 3D View)"
echo "======================================================================"

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

# Step 1: Ensure clean environment
echo "[1/5] Cleaning up old test entities..."
$SPAWN_SCRIPT delete > /dev/null 2>&1 || true

# Step 2: Wait for AMR Patrol state
echo "[2/5] Waiting for Security AMR to enter active PATROL mode..."
until ros2 topic echo /security/status --once 2>/dev/null | grep -q "PATROL"; do
    sleep 1
done
echo "✅ Security AMR is actively patrolling waypoints!"

# Step 3: Let robot patrol for 7 seconds (great for video intro)
echo "[3/5] Patrolling cleanly (7 seconds)... Start your recording now!"
for i in {7..1}; do
    echo "      Spawning threat in ${i}s..."
    sleep 1
done

# Step 4: Spawn unauthorized red intruder target in front aisle
echo "[4/5] 🚨 INTRUDER DETECTED! Spawning red intruder target at (1.8, 1.2)..."
$SPAWN_SCRIPT 1.8 1.2 0.35

echo "      Waiting 6 seconds in STANDOFF mode (Forensic snapshot captured & alert raised)..."
sleep 6

# Step 5: Operator confirmation & threat removal
echo "[5/5] 🛡️ Operator Action: Clearing threat & resuming patrol..."
$SPAWN_SCRIPT delete
sleep 1

# Send operator resume command to AMR
ros2 topic pub --once /security/operator_command std_msgs/msg/String '{data: "{\"action\": \"RESUME_PATROL\"}"}' > /dev/null 2>&1

echo "======================================================================"
echo "🎉 DEMO COMPLETE! Threat neutralized, robot resumed perimeter patrol."
echo "======================================================================"
