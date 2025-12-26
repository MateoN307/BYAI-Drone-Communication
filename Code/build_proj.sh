#!/bin/bash

# Stop the script if any command fails
set -e

echo "Starting Build Process..."

# Step 1: Clean
echo "[1/3] Cleaning project..."
make clean

# Step 2: Build Ground file
echo "[2/3] Building Ground Station..."
make ground_mav

# Step 3: Cross-compile Drone file
echo "[3/3] Building Drone (ARM64)..."
make drone_mav CC=aarch64-linux-gnu-gcc
    
echo "-----------------------------------"
echo "Build Success! All targets compiled."