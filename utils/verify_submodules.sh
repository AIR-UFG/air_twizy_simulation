#!/bin/bash
# utils/verify_submodules.sh

echo "Verifying submodules..."

# Deinitialize all submodules
echo "Deinitializing all submodules..."
git submodule deinit -f .

# Initialize and update all submodules recursively
echo "Initializing and updating all submodules recursively..."
git submodule update --init --recursive

# Verify first-layer submodules
if [ ! -d "ros_packages/vehicle_interface_packages" ]; then
    echo "First-layer submodules are not correctly initialized. Please check the .gitmodules file."
    exit 1
fi

# Verify second-layer submodules within vehicle_interface_packages
if [ ! -d "ros_packages/vehicle_interface_packages/ros2_socketcan" ] || [ ! -d "ros_packages/vehicle_interface_packages/SD-VehicleInterface" ]; then
    echo "Second-layer submodules within vehicle_interface_packages are not correctly initialized. Please check the .gitmodules file."
    exit 1
fi

echo "All submodules are correctly initialized and updated."
