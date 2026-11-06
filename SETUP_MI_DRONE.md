# Setting Up Custom mi_drone Model in PX4

## Problem
Attempting to run `make px4_sitl mi_drone` resulted in error:
```
ninja: error: unknown target 'mi_drone'
make: *** [Makefile:227: px4_sitl] Error 1
```

## Root Cause
The custom `mi_drone` model files existed in the simulation directory, but PX4's build system didn't know about them. Three things were missing:
1. Airframe configuration file
2. Registration in CMake build system
3. Windows Zone.Identifier files were breaking the build

## Solution

### Step 1: Create Airframe Configuration File
Created file: `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_mi_drone`

This file contains:
- Drone type and name
- Simulator settings (Gazebo)
- Motor configuration (4 rotors)
- Motor positions from the model (±0.1228m)
- ESC mappings
- Flight parameters

Key content:
```bash
#!/bin/sh
# @name MI Drone (Custom Quadcopter)
# @type Quadrotor

. ${R}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=mi_drone}

param set-default SIM_GZ_EN 1
param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 4

# Motor positions (±0.1228, ±0.1228)
# ... rotor parameters
```

### Step 2: Register in Build System
Modified: `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`

Added line:
```cmake
4022_gz_mi_drone
```

This makes CMake automatically create the `gz_mi_drone` build target.

### Step 3: Fix Zone.Identifier Issue
Windows/WSL2 creates NTFS alternate data stream files (`:Zone.Identifier`) when downloading or creating files. These break CMake because colons are reserved characters in target names.

**Cleanup commands:**
```bash
# Remove from model directories
find PX4-Autopilot/Tools/simulation/gz/models -name "*:Zone.Identifier" -delete

# Remove from airframe files
find PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes -name "*:Zone.Identifier" -delete

# Remove from worlds
cd PX4-Autopilot/Tools/simulation/gz/worlds
for f in *.sdf; do [ -f "$f:Zone.Identifier" ] && rm "$f:Zone.Identifier"; done

# Clear build cache and rebuild
rm -rf PX4-Autopilot/build/px4_sitl_default
cd PX4-Autopilot
make px4_sitl_default
```

## Result

### Available Commands
```bash
# Build PX4 with mi_drone support
cd PX4-Autopilot
make px4_sitl_default

# Run simulation with mi_drone model
make px4_sitl gz_mi_drone
```

### File Structure
```
PX4-Autopilot/
├── Tools/simulation/gz/models/
│   ├── mi_drone/                    # Wrapper model with motor plugins
│   │   ├── model.sdf
│   │   └── model.config
│   └── mi_drone_base/               # Base model with physics & meshes
│       ├── model.sdf
│       ├── model.config
│       └── meshes/
│           ├── mi_frame_full.dae
│           ├── ccw.dae
│           ├── cw.dae
│           ├── estator.dae
│           └── rotor.dae
└── ROMFS/px4fmu_common/init.d-posix/airframes/
    ├── 4022_gz_mi_drone            # Airframe configuration
    └── CMakeLists.txt              # Build system registration
```

## How It Works

1. **CMake Discovery** (`src/modules/simulation/gz_bridge/CMakeLists.txt`):
   - Scans `ROMFS/px4fmu_common/init.d-posix/airframes/*_gz_*` for airframe files
   - Extracts model name: `4022_gz_mi_drone` → `mi_drone`
   - Creates build targets for each world: `gz_mi_drone`, `gz_mi_drone_forest`, etc.

2. **Runtime Startup**:
   - `make px4_sitl gz_mi_drone` sets `PX4_SIM_MODEL=gz_mi_drone`
   - PX4 startup script finds matching airframe: `4022_gz_mi_drone`
   - Airframe file loads model: `mi_drone` from `Tools/simulation/gz/models/`
   - Gazebo loads the model and PX4 connects via MAVLink on port 14540

## Future Note

If you encounter Zone.Identifier errors again after downloading or creating new files in WSL2, run the cleanup commands above and rebuild.

## Testing

After successful build, test with:
```bash
# Terminal 1: Start PX4 SITL with mi_drone
cd PX4-Autopilot
make px4_sitl gz_mi_drone

# Wait for: INFO  [commander] Ready for takeoff!

# Terminal 2: Run flight script
cd AutonomusDrone
source mavsdk-env/bin/activate
python mavsdk_commands/vuelo_basico.py
```

---

**Date**: December 19, 2025
**PX4 Version**: v1.17.0-alpha1-232-ge7e0f8c272
**Gazebo**: Garden (gz-sim8)
