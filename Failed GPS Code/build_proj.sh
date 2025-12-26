#!/bin/bash

# Stop the script immediately if any command fails
set -e

echo "Starting Build Process..."

# Step 1: Clean previous builds
echo "[1/3] Cleaning project..."
make clean

# Step 2: Build Ground Station
echo "[2/3] Building Ground Station..."
make mav_ground

# Step 3: Cross-compile Drone software
echo "[3/3] Building Drone (ARM64)..."
make mav_drone CC=aarch64-linux-gnu-gcc

echo "-----------------------------------"
echo "Build Success! All targets compiled."