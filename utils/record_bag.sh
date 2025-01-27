#!/bin/bash

# Define the directory to save bag files
ROOT_DIR=$(dirname "$(dirname "$(realpath "$0")")")
SHARED_FOLDER="$ROOT_DIR/shared_folder"

# Create the shared folder if it doesn't exist
mkdir -p "$SHARED_FOLDER"

# Function to get the next available bag file name
get_next_bag_name() {
    PREFIX=$1
    INDEX=1
    while [ -e "$SHARED_FOLDER/${PREFIX}_bag${INDEX}.bag" ]; do
        INDEX=$((INDEX + 1))
    done
    echo "${PREFIX}_bag${INDEX}.bag"
}

# Function to start recording
record_bag() {
    BAG_NAME=$(get_next_bag_name $1)
    TOPICS=$2
    echo "Recording bag file: $SHARED_FOLDER/$BAG_NAME"
    echo "Topics: $TOPICS"
    ros2 bag record $TOPICS -o "$SHARED_FOLDER/$BAG_NAME"
}

# Main script
if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <bag_prefix> <all|specific> [<topic>...]"
    exit 1
fi

PREFIX=$1
MODE=$2

if [ "$MODE" = "all" ]; then
    record_bag $PREFIX "-a"
elif [ "$MODE" = "specific" ]; then
    if [ "$#" -lt 3 ]; then
        echo "Usage: $0 <bag_prefix> specific <topic>..."
        exit 1
    fi
    shift 2
    TOPICS="$@"
    record_bag $PREFIX "$TOPICS"
else
    echo "Invalid mode: $MODE. Use 'all' to record all topics or 'specific' to record specific topics."
    exit 1
fi
