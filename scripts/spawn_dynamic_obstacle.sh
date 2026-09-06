#!/usr/bin/env bash
# scripts/spawn_dynamic_obstacle.sh — spawn or remove dynamic_obstacle model in Gazebo.
#
# USAGE:
#   bash scripts/spawn_dynamic_obstacle.sh            # Spawns dynamic_obstacle
#   REMOVE=1 bash scripts/spawn_dynamic_obstacle.sh    # Removes dynamic_obstacle

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

WORLD="${WORLD:-empty}"
REMOVE="${REMOVE:-0}"
MODEL_NAME="dynamic_obstacle"
SDF_FILE="${SDF_FILE:-${REPO_DIR}/ur5e_robotiq_description/models/dynamic_obstacle/model.sdf}"

GZ_CREATE="/world/${WORLD}/create"
GZ_REMOVE="/world/${WORLD}/remove"

if [[ ! -f "$SDF_FILE" ]]; then
  echo "[ERROR] Model SDF file not found: $SDF_FILE" >&2
  exit 1
fi

if [[ "$REMOVE" == "1" ]]; then
  echo "Removing model ${MODEL_NAME} from world ${WORLD}..."
  gz service -s "$GZ_REMOVE" \
    --reqtype gz.msgs.Entity \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "name: \"${MODEL_NAME}\", type: MODEL"
  echo "Removed ${MODEL_NAME}."
  exit 0
fi

echo "Spawning ${MODEL_NAME} into world ${WORLD} from ${SDF_FILE}..."

SDF_CONTENT=$(cat "$SDF_FILE")
# Flatten newlines for protobuf text-format
SDF_FLAT="${SDF_CONTENT//$'\n'/ }"
# Escape quotes
SDF_REQ="sdf: \"${SDF_FLAT//\"/\\\"}\", name: \"${MODEL_NAME}\", allow_renaming: false"

CREATE_OUT=$(gz service -s "$GZ_CREATE" \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "$SDF_REQ" 2>&1)

echo "$CREATE_OUT"

if echo "$CREATE_OUT" | grep -qi "error\|cannot cross line"; then
  echo "[ERROR] Spawn request rejected for ${MODEL_NAME}" >&2
  exit 2
fi

echo "Successfully spawned ${MODEL_NAME}."
