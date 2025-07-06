#!/usr/bin/env bash

# This script is used to control a PX4 drone using MAVLink commands.
ROOT_DIR="/home/redwan/CppDev/droneController"
BIN_DIR="$ROOT_DIR/build"
EXECUTABLE="$BIN_DIR/quadrotor_teleoperation"

# Function to start the PX4 controller
start_px4_controller() {
    echo "Starting PX4 controller..."
    $EXECUTABLE $ROOT_DIR/config/teleop_keyboard.xml
    echo "PX4 controller terminated."
}

start_px4_controller