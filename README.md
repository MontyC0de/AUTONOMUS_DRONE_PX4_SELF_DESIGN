# Design and Control of an Autonomous UAV and its Digital Twin

**Final Degree Project (TFG)**, completed in September 2026

<table>
<tr><td><b>Author</b></td><td>Antonio Montero Díaz</td></tr>
<tr><td><b>Degree</b></td><td>BSc in Electronic, Robotics and Mechatronics Engineering</td></tr>
<tr><td><b>Specialisation</b></td><td>Control</td></tr>
<tr><td><b>University</b></td><td>University of Málaga</td></tr>
<tr><td><b>Supervisor</b></td><td>Ricardo Vázquez</td></tr>
<tr><td><b>Status</b></td><td>Completed (September 2026)</td></tr>
</table>

---

This project implements an autonomous drone control system using the PX4 Autopilot flight stack and MAVSDK Python SDK. It demonstrates programmatic drone control including takeoff, landing, and offboard velocity-based flight maneuvers.

![Render of the custom drone frame](Design/Untitled_2025-Dec-15_08-25-52PM-000_CustomizedView4066260842.png)

## Table of Contents

- [Project Overview](#project-overview)
- [System Architecture](#system-architecture)
- [Digital Twin Parameters](#digital-twin-parameters)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Project Structure](#project-structure)
- [Usage](#usage)
- [Flight Scripts](#flight-scripts)
- [References](#references)

## Project Overview

This project explores autonomous drone navigation through software-in-the-loop (SITL) simulation. The system integrates:

- **PX4 Autopilot**: Open-source flight control software providing low-level flight control, state estimation, and safety features
- **MAVSDK**: High-level Python SDK for drone communication via MAVLink protocol
- **QGroundControl**: Ground control station for real-time monitoring and configuration

The implemented flight scripts demonstrate:
- Autonomous takeoff and landing sequences
- Offboard control mode with velocity-based navigation
- Real-time telemetry monitoring (altitude, flight mode, position)
- Asynchronous task management for concurrent operations

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Ground Control Station                       │
│                      (QGroundControl)                           │
└─────────────────────────────────────────────────────────────────┘
                              │ MAVLink
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                      PX4 Autopilot (SITL)                       │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │  Commander  │  │    EKF2     │  │  Position Controller    │  │
│  │  (State)    │  │ (Estimator) │  │  (mc_pos_control)       │  │
│  └─────────────┘  └─────────────┘  └─────────────────────────┘  │
│                         uORB Messaging                          │
└─────────────────────────────────────────────────────────────────┘
                              │ MAVLink (UDP:14540)
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                   MAVSDK Python Application                     │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │   Action    │  │  Telemetry  │  │       Offboard          │  │
│  │ (arm/land)  │  │ (position)  │  │  (velocity control)     │  │
│  └─────────────┘  └─────────────┘  └─────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### Communication Protocols

| Interface | Protocol | Port | Purpose |
|-----------|----------|------|---------|
| MAVSDK ↔ PX4 | MAVLink over UDP | 14540 | Flight commands and telemetry |
| QGC ↔ PX4 | MAVLink over UDP | 14550 | Ground station monitoring |
| PX4 Internal | uORB | N/A | Inter-module publish-subscribe |

## Digital Twin Parameters

The Gazebo model `mi_drone` (in [`simulation/models/`](simulation/models)) uses the parameters of the real drone, not the stock PX4 x500:

| Parameter | Value | Source |
|-----------|-------|--------|
| Body mass | 0.54 kg | Fusion 360 assembly |
| Inertia Ixx / Iyy / Izz | 0.0087 / 0.0087 / 0.0170 kg·m² | Fusion 360 assembly (Izz ≤ Ixx + Iyy enforced) |
| Motor positions | (±122.8, ±122.8) mm, 347 mm diagonal | CAD mesh, matches drawings (125 mm plates, 117 mm arms) |
| Propeller | DALPROP Cyclone T5050C, 5" tri-blade, 6 g | Component spec; inertia Izz = mR²/3 |
| Motor | Racerstar BR2205 2300KV, 4S | Component spec |
| Max thrust per motor | ~950 g (9.32 N) | Manufacturer data (5045 prop, 14.8 V) |
| Max motor speed | 2700 rad/s | 2300 KV × 14.8 V, ~76 % under load |
| Motor constant | 1.278e-6 N/(rad/s)² | T_max / ω_max² |
| Hover throttle | 0.28 (predicted) / 0.278 (SITL estimate) | m·g/4 = k·ω² |

| PX4 x500 (previous) | mi_drone (now) |
|---|---|
| 2.0 kg, 13" props, motors at ±174 mm | 0.54 kg, 5" props, motors at ±122.8 mm |

## Prerequisites

### System Requirements

- **Operating System**: Ubuntu 24.04 LTS (native or WSL2 on Windows 11)
- **Simulator**: Gazebo Harmonic (gz-sim 8), installed by the PX4 setup script
- **Python**: 3.12 (Ubuntu 24.04 default)
- **RAM**: 8 GB minimum (16 GB recommended for Gazebo simulation)
- **Disk Space**: ~5 GB for PX4 and dependencies

### Required Software

- Git
- Python 3 with pip
- Build tools (gcc, g++, make, cmake)

## Installation

### 1. Clone this Repository

```bash
git clone https://github.com/MontyC0de/AUTONOMUS_DRONE_PX4_SELF_DESIGN.git
cd AUTONOMUS_DRONE_PX4_SELF_DESIGN
```

### 2. Initialize PX4-Autopilot Submodule

The PX4 flight stack is included as a git submodule. After cloning this repository, initialize and update the submodule:

```bash
git submodule update --init --recursive
```

**About submodules:** PX4-Autopilot is tracked at a specific commit of the [MontyC0de/PX4-Autopilot](https://github.com/MontyC0de/PX4-Autopilot) fork, ensuring reproducibility. To update it to the latest commit of the fork's `dev` branch:

```bash
git submodule update --remote PX4-Autopilot
git add PX4-Autopilot
git commit -m "chore: update px4-autopilot submodule"
```

### 3. Install PX4 Dependencies

```bash
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

This script installs all necessary dependencies including:
- ARM GCC toolchain
- Gazebo Harmonic simulator
- Required Python packages

### 4. Set Up Python Virtual Environment

```bash
cd ..  # Return to the repository root
python3 -m venv mavsdk-env
source mavsdk-env/bin/activate
pip install mavsdk
```

### 5. Link the Drone Model into PX4

PX4 loads Gazebo models from `PX4-Autopilot/Tools/simulation/gz/models`, which belongs to the official `PX4-gazebo-models` repository. The `mi_drone` models live in this repo under `simulation/models` and are symlinked in:

```bash
./scripts/setup_sim.sh
```

The airframe `4022_gz_mi_drone` is part of the [MontyC0de/PX4-Autopilot](https://github.com/MontyC0de/PX4-Autopilot) fork (`dev` branch).

### 6. Build PX4 SITL

```bash
cd PX4-Autopilot
make px4_sitl_default
```

## Project Structure

```
AUTONOMUS_DRONE_PX4_SELF_DESIGN/
├── README.md                 # This file
├── .gitignore                # Git ignore rules
├── .gitmodules               # Git submodule configuration
│
├── PX4-Autopilot/            # PX4 flight stack (git submodule)
│   ├── src/modules/          # Flight control modules
│   ├── src/drivers/          # Hardware drivers
│   ├── ROMFS/                # Runtime filesystem
│   └── Tools/                # Build and setup tools
│
├── simulation/
│   ├── models/mi_drone/      # Motor plugins (BR2205 thrust model)
│   └── models/mi_drone_base/ # Mass, inertia, collisions, sensors, CAD meshes
│
├── scripts/
│   └── setup_sim.sh          # Links the models into the PX4 tree
│
├── mavsdk_commands/          # Python flight control scripts
│   ├── vuelo_basico.py       # Basic takeoff/landing demo
│   └── vuelo_01.py           # Offboard velocity control demo
│
├── Design/                   # CAD renders and frame assembly video
│
├── mavsdk-env/               # Python virtual environment (not tracked)
│
└── QGroundControl-x86_64.AppImage  # Ground control station (not tracked)
```

## Usage

### Starting the Simulation Environment

**Terminal 1 - Start PX4 SITL with Gazebo and the drone's digital model:**

```bash
cd PX4-Autopilot
make px4_sitl gz_mi_drone
```

Wait until you see `INFO  [commander] Ready for takeoff!` in the console.

**Terminal 2 - Run Flight Script:**

```bash
source mavsdk-env/bin/activate
python mavsdk_commands/vuelo_basico.py
```

**Optional - Ground Control Station:**

```bash
./QGroundControl-x86_64.AppImage
```

### Simulation Without Graphics

For faster testing without the Gazebo GUI:

```bash
HEADLESS=1 make px4_sitl gz_mi_drone
```

## Flight Scripts

### vuelo_basico.py - Basic Flight Demo

Demonstrates fundamental autonomous flight operations:

1. Connects to PX4 SITL via UDP
2. Waits for GPS lock and home position
3. Arms the vehicle
4. Takes off to 10 meters altitude
5. Hovers for 10 seconds
6. Lands autonomously

**Features:**
- Concurrent telemetry monitoring (altitude, flight mode)
- Graceful task termination on landing
- Connection state verification

### vuelo_01.py - Offboard Velocity Control

Demonstrates offboard mode with velocity setpoints:

1. Connects, arms and takes off to 5 m
2. Enters offboard control mode
3. Executes a 5 m square using NED velocity commands at 1 m/s:
   - North (forward) for 5 seconds
   - East (right) for 5 seconds
   - South (backward) for 5 seconds
   - West (left) for 5 seconds
4. Stops, lands and waits until on the ground

**Key Concepts:**
- `VelocityNedYaw`: Velocity setpoint in North-East-Down frame
- Offboard mode requires continuous setpoint streaming
- Initial setpoint must be sent before starting offboard mode

## Development

### PX4 Build Commands

```bash
# SITL simulation build
make px4_sitl_default

# Build and run with Gazebo and mi_drone
make px4_sitl gz_mi_drone

# Clean build artifacts
make clean

# Run unit tests
make tests

# Code style check
make check_format
```

### Creating New Flight Scripts

```python
import asyncio
from mavsdk import System

async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    # Wait for connection
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    # Your flight logic here
    await drone.action.arm()
    await drone.action.takeoff()
    # ...

if __name__ == "__main__":
    asyncio.run(run())
```

## References

### Documentation

- [PX4 User Guide](https://docs.px4.io/main/en/)
- [PX4 Developer Guide](https://docs.px4.io/main/en/development/development.html)
- [MAVSDK Python Documentation](https://mavsdk.mavlink.io/main/en/python/)
- [MAVLink Protocol](https://mavlink.io/en/)

### Related Projects

- [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) - Flight control stack
- [MAVSDK](https://github.com/mavlink/MAVSDK) - MAVLink SDK
- [QGroundControl](http://qgroundcontrol.com/) - Ground control station

---

## About

This project was developed as a Final Degree Project (TFG) at the University of Málaga (September 2026), demonstrating autonomous drone control systems using open-source flight control software.

---

**University of Málaga** | School of Industrial Engineering

*Antonio Montero Díaz*
