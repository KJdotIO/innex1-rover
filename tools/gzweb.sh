#!/bin/bash
# Launch Gazebo Fortress for the moon yard arena.
#
# Usage:
#   ./tools/gzweb.sh          # GUI mode (requires display / X11 forwarding)
#   ./tools/gzweb.sh --headless  # Server-only mode (no GUI)
#
# For web-based visualization (gzweb), install the websocket plugin:
#   sudo apt install libignition-launch5-dev ignition-launch5
# Then add to moon_yard.sdf:
#   <plugin filename="ignition-launch-websocket-server-system"
#           name="ignition::launch::WebsocketServer">
#     <port>9002</port>
#   </plugin>
# And open http://localhost:9002 in a browser.

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

source /opt/ros/humble/setup.bash
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
  source "$SCRIPT_DIR/install/setup.bash"
fi

INSTALL_MODELS="$SCRIPT_DIR/install/lunabot_simulation/share/lunabot_simulation/models"
SRC_MODELS="$SCRIPT_DIR/src/lunabot_simulation/models"
MODELS_PATH="$INSTALL_MODELS"
if [ ! -d "$MODELS_PATH" ]; then
  MODELS_PATH="$SRC_MODELS"
fi
export GZ_SIM_RESOURCE_PATH="$MODELS_PATH:${GZ_SIM_RESOURCE_PATH:-}"

WORLD="$SCRIPT_DIR/install/lunabot_simulation/share/lunabot_simulation/worlds/moon_yard.sdf"
if [ ! -f "$WORLD" ]; then
  WORLD="$SCRIPT_DIR/src/lunabot_simulation/worlds/moon_yard.sdf"
fi

echo "World: $WORLD"

if [ "$1" = "--headless" ]; then
  echo "Launching Gazebo server only (headless)..."
  ign gazebo -r -s "$WORLD"
else
  echo "Launching Gazebo with GUI..."
  ign gazebo -r "$WORLD"
fi
