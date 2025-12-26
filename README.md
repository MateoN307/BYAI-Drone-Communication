# BeagleY-AI Drone Flight Controller & Ground Station
**by Mateo Noriega, An Phan, Hunter Will and Bennet Wilson**

This project implements a custom flight control system for a BeagleY-AI based drone. It consists of two main components:
1. **Onboard Flight Computer (`mav_drone`):** Runs on the BeagleY-AI, interfaces with the Flight Controller (ArduPilot) via MAVLink, and executes safety failsafes.
2. **Ground Station (`mav_ground`):** Runs on a Linux laptop/VM, reads a PS4 controller, and sends telemetry/commands over UDP.

## ⚠️ Important Note: Missing MAVLink Library

**The `c_library_v2` folder is NOT included in this zip file.**

**To compile this project, you must:**
1. Download the [MAVLink v2 C library](https://github.com/mavlink/c_library_v2.git).
2. Place the `c_library_v2` folder in the root directory of this project (same level as `makefile`).

## Build Instructions

We have provided a shell script to automate the cross-compilation process for the drone and the local build for the ground station.

### 1. Make the script executable
Run the following command in your terminal:
```bash
chmod +x build_proj.sh
```

## Failed GPS implementation
The original project idea was to implement an autonomous drone controlled by missions containing different checkpoints defined by a set of 3D coordinates. 

Unfortunately this was not possible as we realized too late that the GPS sensor choosen didn't include a compass. This issue was discovered at a very late time in the project and with the course's time constraints we didn't have time to find a substitue part or fix for this issue.