#!/usr/bin/env bash
# Source this file to enforce SROS2 DDS encryption and authentication
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
export ROS_SECURITY_KEYSTORE="${HOME}/Desktop/amr_security_ws/keystores/depot_keystore"
echo "🔐 SROS2 Enforced: DDS topic encryption & node authentication active."
