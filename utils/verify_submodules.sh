#!/bin/bash
# utils/verify_submodules.sh

echo "Verifying submodules..."

# Deinitialize all submodules
echo "Deinitializing all submodules..."
git submodule deinit -f .

# Temporary function to change submodule URLs to HTTPS
change_submodule_url_to_https() {
    local submodule_path=$1
    local https_url=$2

    if [ -d "$submodule_path" ]; then
        git config -f .gitmodules submodule."$submodule_path".url "$https_url"
        git submodule sync "$submodule_path"
    fi
}

# Convert URLs in .gitmodules to HTTPS
change_submodule_url_to_https "ros_packages/vehicle_interface_packages" "https://github.com/AIR-UFG/vehicle_interface_packages.git"

# Initialize and update all submodules recursively
echo "Initializing and updating all submodules recursively..."
git submodule update --init --recursive

# Verify first-layer submodules
if [ ! -d "ros_packages/vehicle_interface_packages" ]; then
    echo "First-layer submodules are not correctly initialized. Please check the .gitmodules file."
    exit 1
fi

# Convert second-layer submodule URLs to HTTPS
cd ros_packages/vehicle_interface_packages
change_submodule_url_to_https "SD-VehicleInterface" "https://github.com/AIR-UFG/SD-VehicleInterface.git"
change_submodule_url_to_https "ros2_socketcan" "https://github.com/AIR-UFG/ros2_socketcan.git"
cd -

# Initialize and update second-layer submodules
echo "Initializing and updating second-layer submodules recursively..."
git submodule update --init --recursive

# Verify second-layer submodules within vehicle_interface_packages
if [ ! -d "ros_packages/vehicle_interface_packages/ros2_socketcan" ] || [ ! -d "ros_packages/vehicle_interface_packages/SD-VehicleInterface" ]; then
    echo "Second-layer submodules within vehicle_interface_packages are not correctly initialized. Please check the .gitmodules file."
    exit 1
fi

echo "All submodules are correctly initialized and updated."
