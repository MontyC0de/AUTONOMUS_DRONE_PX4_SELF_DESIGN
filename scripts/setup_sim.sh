#!/usr/bin/env bash
# Links the mi_drone Gazebo models from this repo into the PX4 tree.
#
# PX4 only spawns models from PX4-Autopilot/Tools/simulation/gz/models, which
# is a submodule of the official PX4-gazebo-models repo we cannot push to, so
# the models live here under simulation/models and are symlinked in.
#
# Run once after cloning (and again after a fresh submodule checkout):
#   ./scripts/setup_sim.sh
# Then:
#   cd PX4-Autopilot && make px4_sitl gz_mi_drone
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4="$REPO/PX4-Autopilot"
GZ_MODELS="$PX4/Tools/simulation/gz/models"
GZ_WORLDS="$PX4/Tools/simulation/gz/worlds"

if [ ! -d "$GZ_MODELS" ]; then
	echo "PX4 submodules missing, initialising..."
	git -C "$REPO" submodule update --init --recursive
fi

for model in "$REPO"/simulation/models/*/; do
	name="$(basename "$model")"
	ln -sfn "$model" "$GZ_MODELS/$name"
	echo "model  $name -> $GZ_MODELS/$name"
done

# Keep the symlinks out of the PX4-gazebo-models submodule's `git status`
EXCLUDE="$(git -C "$GZ_MODELS" rev-parse --git-path info/exclude)"
case "$EXCLUDE" in /*) ;; *) EXCLUDE="$GZ_MODELS/$EXCLUDE" ;; esac
for model in "$REPO"/simulation/models/*/ "$REPO"/simulation/worlds/*.sdf; do
	[ -e "$model" ] || continue
	entry="/$(realpath --relative-to="$(git -C "$GZ_MODELS" rev-parse --show-toplevel)" "$GZ_MODELS")/$(basename "$model")"
	case "$model" in *.sdf) entry="${entry/\/models\//\/worlds\/}" ;; esac
	grep -qxF "$entry" "$EXCLUDE" 2>/dev/null || echo "$entry" >> "$EXCLUDE"
done

for world in "$REPO"/simulation/worlds/*.sdf; do
	[ -e "$world" ] || continue
	ln -sfn "$world" "$GZ_WORLDS/$(basename "$world")"
	echo "world  $(basename "$world") -> $GZ_WORLDS"
done

# Files copied from Windows into WSL carry NTFS "Zone.Identifier" streams that
# break PX4's CMake target generation (':' is not allowed in target names).
find "$REPO/simulation" "$PX4/ROMFS/px4fmu_common/init.d-posix/airframes" \
	"$GZ_MODELS" "$GZ_WORLDS" -name '*:Zone.Identifier' -delete 2>/dev/null || true

echo "Done. Run: cd PX4-Autopilot && make px4_sitl gz_mi_drone"
