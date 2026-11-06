# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an autonomous drone control system developed as a Final Degree Project (TFG) at Universidad de Málaga. The project integrates PX4 Autopilot flight stack with MAVSDK Python SDK to demonstrate programmatic drone control through software-in-the-loop (SITL) simulation.

## Key Architecture Components

### PX4-Autopilot Submodule
- **Important**: PX4-Autopilot is tracked as a git submodule pointing to a forked repository
- Repository: `git@github.com:MontyC0de/PX4-Autopilot.git` (branch: `dev`)
- After cloning this repository, initialize submodules with: `git submodule update --init --recursive`
- The submodule is tracked at specific commits for reproducibility
- To update PX4: `cd PX4-Autopilot && git pull origin dev && cd .. && git add PX4-Autopilot && git commit -m "Update PX4-Autopilot submodule"`

### Communication Stack
The system uses a layered communication architecture:
1. **MAVLink Protocol** (UDP): Communication protocol between components
   - MAVSDK ↔ PX4: Port 14540 (flight commands and telemetry)
   - QGroundControl ↔ PX4: Port 14550 (monitoring)
2. **uORB**: PX4's internal publish-subscribe messaging (Commander, EKF2, Position Controller)
3. **MAVSDK Python**: High-level API wrapper for MAVLink communication

### Flight Control Scripts
Located in `mavsdk_commands/`:
- `vuelo_basico.py`: Basic autonomous flight demo (takeoff, hover, land) with concurrent telemetry monitoring
- `vuelo_01.py`: Offboard velocity control demonstration using NED (North-East-Down) frame

## Common Development Commands

### PX4 SITL Build and Simulation

```bash
# Build PX4 for SITL
cd PX4-Autopilot
make px4_sitl_default

# Run SITL with Gazebo Classic visualization
make px4_sitl gazebo-classic

# Run SITL without GUI (headless mode for faster testing)
HEADLESS=1 make px4_sitl gazebo-classic

# Clean build artifacts
make clean

# Run PX4 unit tests
make tests

# Check code formatting
make check_format
```

**Important**: Wait for `INFO  [commander] Ready for takeoff!` message before running flight scripts.

### Python Environment and Flight Scripts

```bash
# Activate virtual environment
source mavsdk-env/bin/activate

# Run flight scripts
python mavsdk_commands/vuelo_basico.py
python mavsdk_commands/vuelo_01.py

# Install/update MAVSDK
pip install mavsdk
```

### Typical Workflow

**Terminal 1** - Start PX4 SITL:
```bash
cd PX4-Autopilot
make px4_sitl gazebo-classic
```

**Terminal 2** - Run flight script:
```bash
source mavsdk-env/bin/activate
python mavsdk_commands/vuelo_basico.py
```

**Optional Terminal 3** - Ground Control Station:
```bash
./QGroundControl-x86_64.AppImage
```

## MAVSDK Python Development Patterns

### Connection Pattern
All flight scripts follow this connection pattern:
```python
drone = System()
await drone.connect(system_address="udp://:14540")  # or "udpin://0.0.0.0:14540"

# Wait for connection
async for state in drone.core.connection_state():
    if state.is_connected:
        break
```

### Telemetry Monitoring
Use concurrent tasks for real-time telemetry:
```python
# Create concurrent monitoring tasks
altitude_task = asyncio.ensure_future(print_altitude(drone))
flight_mode_task = asyncio.ensure_future(print_flight_mode(drone))

# Monitor telemetry streams
async for position in drone.telemetry.position():
    # Process position updates

async for health in drone.telemetry.health():
    if health.is_global_position_ok and health.is_home_position_ok:
        # Ready for flight
```

### Offboard Control Mode
For velocity-based control:
1. Set initial velocity setpoint BEFORE starting offboard mode
2. Start offboard mode
3. Continuously stream velocity setpoints (North-East-Down frame with Yaw)
4. Stop offboard mode before landing

```python
await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
await drone.offboard.start()

# Send velocity commands (must be continuous)
await drone.offboard.set_velocity_ned(VelocityNedYaw(1.0, 0.0, 0.0, 0.0))  # North
```

## Custom mi_drone Model

The repository includes a custom drone model registered with PX4:
- **Model files**: `PX4-Autopilot/Tools/simulation/gz/models/mi_drone/` and `mi_drone_base/`
- **Airframe config**: `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_mi_drone`
- **Build target**: `make px4_sitl gz_mi_drone`
- **Run simulation**: `make px4_sitl gz_mi_drone` (one command to build and run)

### WSL2/Windows Zone.Identifier Files
**Important**: When working in WSL2, Windows creates NTFS alternate data stream files (`:Zone.Identifier`) that can break the PX4 build. If you see CMake errors about reserved target names with "Zone.Identifier", remove them:

```bash
# Remove from model directories
find PX4-Autopilot/Tools/simulation/gz/models -name "*:Zone.Identifier" -delete

# Remove from airframe files
find PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes -name "*:Zone.Identifier" -delete

# Remove from worlds
cd PX4-Autopilot/Tools/simulation/gz/worlds && for f in *.sdf; do [ -f "$f:Zone.Identifier" ] && rm "$f:Zone.Identifier"; done

# Clear build cache and rebuild
rm -rf PX4-Autopilot/build/px4_sitl_default
make px4_sitl_default
```

## Code Organization

```
AutonomusDrone/
├── mavsdk_commands/          # Python flight control scripts
│   ├── vuelo_basico.py       # Takeoff/hover/land with telemetry
│   └── vuelo_01.py           # Offboard velocity control demo
├── PX4-Autopilot/            # Git submodule (forked PX4 flight stack)
├── Design/                   # CAD renders and design images
├── mavsdk-env/               # Python virtual environment (git-ignored)
└── QGroundControl-*.AppImage # Ground control station (git-ignored)
```

## Important Development Notes

### Git Submodules
- Always use `git submodule update --init --recursive` after cloning
- The PX4-Autopilot submodule tracks the `dev` branch of the forked repository
- Changes to PX4 configuration should be committed in the submodule first, then referenced in the parent repo

### Python Virtual Environment
- The `mavsdk-env/` directory is git-ignored
- Always activate before running flight scripts
- Only MAVSDK library is required for flight scripts

### Coordinate Frames
- **NED (North-East-Down)**: PX4 uses NED frame for position and velocity
  - North: Forward (positive X)
  - East: Right (positive Y)
  - Down: Downward (positive Z, altitude is negative)
- Yaw angle: 0° = North, 90° = East, -90° = West, 180° = South

### Flight Modes
Key PX4 flight modes in this project:
- **OFFBOARD**: External computer control via MAVLink (used in vuelo_01.py)
- **AUTO.TAKEOFF**: Autonomous takeoff to specified altitude
- **AUTO.LAND**: Autonomous landing at current position
- **HOLD**: Position hold mode

## System Requirements

- Ubuntu 20.04/22.04 LTS (or WSL2)
- Python 3.8+
- 8 GB RAM minimum (16 GB recommended for Gazebo)
- ~5 GB disk space for PX4 and dependencies

## Reference Documentation

- [PX4 User Guide](https://docs.px4.io/main/en/)
- [PX4 Developer Guide](https://docs.px4.io/main/en/development/development.html)
- [MAVSDK Python Documentation](https://mavsdk.mavlink.io/main/en/python/)
- [MAVLink Protocol](https://mavlink.io/en/)
