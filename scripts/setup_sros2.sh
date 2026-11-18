#!/usr/bin/env bash
# ==============================================================================
# SROS2 Keystore & Enclaves Generator for Autonomous Warehouse Security AMR
# Generates cryptographic keys and certificates for DDS security (authentication,
# access control, and encryption) across all nodes in the security stack.
# ==============================================================================
set -e

WS_DIR="${HOME}/Desktop/amr_security_ws"
KEYSTORE_DIR="${WS_DIR}/keystores/depot_keystore"
ENV_SCRIPT="${WS_DIR}/scripts/sros2_env.sh"

echo "======================================================================"
echo "🔐 SROS2 Cryptographic Keystore & Enclave Setup"
echo "======================================================================"

source /opt/ros/jazzy/setup.bash

# Ensure output directory exists
mkdir -p "${WS_DIR}/keystores"

# Step 1: Create or initialize SROS2 Root Keystore
if [ -d "${KEYSTORE_DIR}" ]; then
    echo "[1/4] Keystore already exists at: ${KEYSTORE_DIR}"
else
    echo "[1/4] Creating SROS2 root keystore at: ${KEYSTORE_DIR}..."
    ros2 security create_keystore "${KEYSTORE_DIR}"
fi

# Step 2: Define and create enclaves for all security and navigation nodes
ENCLAVES=(
    "/amr_command_center_node"
    "/patrol_interceptor_node"
    "/auto_explorer_node"
    "/mission_orchestrator_node"
    "/amcl"
    "/controller_server"
    "/planner_server"
    "/bt_navigator"
    "/behavior_server"
    "/ros_gz_bridge"
    "/robot_state_publisher"
)

echo "[2/4] Generating cryptographic enclaves for security nodes..."
for enclave in "${ENCLAVES[@]}"; do
    echo "      Creating enclave: ${enclave}"
    ros2 security create_enclave "${KEYSTORE_DIR}" "${enclave}" > /dev/null 2>&1 || true
done

# Step 3: Create helper environment script for enabling SROS2 in terminals
echo "[3/4] Creating SROS2 environment activation script: ${ENV_SCRIPT}..."
cat << 'EOF' > "${ENV_SCRIPT}"
#!/usr/bin/env bash
# Source this file to enforce SROS2 DDS encryption and authentication
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_KEYSTORE="${HOME}/Desktop/amr_security_ws/keystores/depot_keystore"
echo "🔐 SROS2 Enforced: DDS topic encryption & node authentication active."
EOF

chmod +x "${ENV_SCRIPT}"

# Step 4: Summary
echo "[4/4] Verifying generated enclaves..."
ros2 security list_enclaves "${KEYSTORE_DIR}"

echo "======================================================================"
echo "✅ SROS2 Keystore setup completed successfully!"
echo "   Keystore Directory: ${KEYSTORE_DIR}"
echo "   To activate DDS encryption: source ${ENV_SCRIPT}"
echo "======================================================================"
