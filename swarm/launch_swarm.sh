#!/usr/bin/env bash
# =============================================================================
#  launch_swarm.sh — PX4 SITL multi-drone swarm launcher (gz_x500)
#
#  Usage:
#    ./swarm/launch_swarm.sh [NUM_DRONES]
#
#  Examples:
#    ./swarm/launch_swarm.sh        # launch 3 drones (default)
#    ./swarm/launch_swarm.sh 5      # launch 5 drones
#
#  Prerequisites:
#    make px4_sitl_default          # build the SITL binary first
#
#  Each drone gets:
#    - MAVLink GCS port  : 18570 + N  (connect QGC here)
#    - Offboard API port : 14580 + N  (MAVSDK / MAVROS)
#    - MAV_SYS_ID        : N + 1
#
#  Drone logs: swarm/instances/drone_N/px4.log
# =============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${PX4_ROOT}/build/px4_sitl_default"
PX4_BIN="${BUILD_DIR}/bin/px4"

NUM_DRONES="${1:-3}"
DRONE_SPACING=0.5   # metres between drones (square grid)
WORLD_NAME="swarm_x500"

# ---------------------------------------------------------------------------
# Pre-flight checks
# ---------------------------------------------------------------------------
if [ ! -f "${PX4_BIN}" ]; then
    echo "ERROR: PX4 SITL binary not found at:"
    echo "       ${PX4_BIN}"
    echo ""
    echo "Build it first:"
    echo "  cd ${PX4_ROOT} && make px4_sitl_default"
    exit 1
fi

if ! [[ "${NUM_DRONES}" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: NUM_DRONES must be a positive integer (got: '${NUM_DRONES}')"
    exit 1
fi

# ---------------------------------------------------------------------------
# Environment setup — source PX4's gz_env.sh (models, plugins, server config)
# ---------------------------------------------------------------------------
GZ_ENV="${BUILD_DIR}/rootfs/gz_env.sh"
if [ ! -f "${GZ_ENV}" ]; then
    echo "ERROR: gz_env.sh not found at ${GZ_ENV}"
    echo "       Ensure the project has been built with: make px4_sitl_default"
    exit 1
fi
# Pre-initialize variables that gz_env.sh expands unconditionally,
# so they are bound even when not set in the caller's environment.
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

# shellcheck source=/dev/null
source "${GZ_ENV}"

# px4-rc.gzsim re-sources gz_env.sh from inside PX4's working directory,
# which would overwrite PX4_GZ_WORLDS back to the build default.
# We fix this by writing a per-instance gz_env.sh (below, before launch)
# that px4-rc.gzsim finds first (./gz_env.sh has priority over ../gz_env.sh).

# ---------------------------------------------------------------------------
# Instance directories
# ---------------------------------------------------------------------------
INSTANCES_DIR="${SCRIPT_DIR}/instances"
mkdir -p "${INSTANCES_DIR}"

# ---------------------------------------------------------------------------
# Cleanup on exit
# ---------------------------------------------------------------------------
PIDS=()

cleanup() {
    echo ""
    echo "Stopping swarm..."
    for pid in "${PIDS[@]}"; do
        kill "${pid}" 2>/dev/null || true
    done
    # Give processes a moment to exit cleanly
    sleep 1
    for pid in "${PIDS[@]}"; do
        kill -9 "${pid}" 2>/dev/null || true
    done
    pkill -f "gz sim" 2>/dev/null || true
    echo "All instances stopped."
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------------------
# Launch drones
# ---------------------------------------------------------------------------
echo ""
echo "========================================================"
echo "  PX4 x500 Swarm Simulation"
echo "  World   : ${WORLD_NAME}"
echo "  Drones  : ${NUM_DRONES}"
  echo "  Spacing : ${DRONE_SPACING} m (square grid)"
echo "========================================================"
echo ""

PX4_ROOTFS="${BUILD_DIR}/rootfs"

# Compute grid columns: smallest integer >= sqrt(NUM_DRONES)
GRID_COLS=$(awk "BEGIN{c=int(sqrt(${NUM_DRONES})); while(c*c<${NUM_DRONES}) c++; print c}")

for i in $(seq 0 $((NUM_DRONES - 1))); do
    INSTANCE_DIR="${INSTANCES_DIR}/drone_${i}"
    PX4_INSTANCE_WD="${PX4_ROOTFS}/${i}"
    mkdir -p "${INSTANCE_DIR}" "${PX4_INSTANCE_WD}"

    # Write a gz_env.sh into PX4's working dir. px4-rc.gzsim sources
    # "./gz_env.sh" first, so this takes priority over rootfs/gz_env.sh
    # and ensures Gazebo finds our swarm world.
    cat > "${PX4_INSTANCE_WD}/gz_env.sh" << GZENV
#!/usr/bin/env bash
export PX4_GZ_MODELS="${PX4_GZ_MODELS}"
export PX4_GZ_WORLDS="${SCRIPT_DIR}/worlds"
export PX4_GZ_PLUGINS="${PX4_GZ_PLUGINS}"
export PX4_GZ_SERVER_CONFIG="${PX4_GZ_SERVER_CONFIG}"
export GZ_SIM_RESOURCE_PATH="\${GZ_SIM_RESOURCE_PATH:-}:\${PX4_GZ_MODELS}:\${PX4_GZ_WORLDS}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="\${GZ_SIM_SYSTEM_PLUGIN_PATH:-}:\${PX4_GZ_PLUGINS}"
export GZ_SIM_SERVER_CONFIG_PATH="\${PX4_GZ_SERVER_CONFIG}"
GZENV

    # Square grid layout: row = i / COLS, col = i % COLS
    POSE_X=$(awk "BEGIN{printf \"%.2f\", (${i} % ${GRID_COLS}) * ${DRONE_SPACING}}")
    POSE_Y=$(awk "BEGIN{printf \"%.2f\", int(${i} / ${GRID_COLS}) * ${DRONE_SPACING}}")
    POSE="${POSE_X},${POSE_Y},0,0,0,0"

    # Build per-instance env overrides
    EXTRA_ENV=()
    if [ "${i}" -gt 0 ]; then
        # Attach to already-running Gazebo instead of launching a new one
        EXTRA_ENV+=(PX4_GZ_STANDALONE=1)
    fi

    echo "  [drone ${i}]  MAV_SYS_ID=$((i+1))  pose=(${POSE_X}, ${POSE_Y}, 0)  GCS=:$((18570+i))  offboard=:$((14580+i))"

    env \
        PX4_SIM_MODEL=gz_x500 \
        PX4_GZ_WORLD="${WORLD_NAME}" \
        PX4_GZ_MODEL_POSE="${POSE}" \
        "${EXTRA_ENV[@]}" \
        "${PX4_BIN}" -i "${i}" -d \
        > "${INSTANCE_DIR}/px4.log" 2>&1 &
    ln -sf "${INSTANCE_DIR}/px4.log" "${PX4_INSTANCE_WD}/px4_stdout.log" 2>/dev/null || true

    PIDS+=($!)

    if [ "${i}" -eq 0 ]; then
        # First drone starts Gazebo — wait for it to be ready
        echo ""
        echo "  Waiting for Gazebo to initialise (drone 0)..."
        sleep 6
    else
        sleep 2
    fi
done

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
echo "========================================================"
echo "  Swarm running: ${NUM_DRONES} × gz_x500"
echo ""
printf "  %-8s %-12s %-18s %s\n" "Drone" "MAV_SYS_ID" "GCS port (UDP)" "Offboard port (UDP)"
for i in $(seq 0 $((NUM_DRONES - 1))); do
    printf "  %-8s %-12s %-18s %s\n" "${i}" "$((i+1))" "$((18570+i))" "$((14580+i))"
done
echo ""
echo "  Connect QGroundControl → Add Comm Link → UDP → port 18570"
  echo "  stdout logs : ${INSTANCES_DIR}/drone_N/px4.log"
  echo "  PX4 logs    : ${BUILD_DIR}/rootfs/N/log/"
  echo "  (WSL tip: if Gazebo renders black, re-run with: LIBGL_ALWAYS_SOFTWARE=1 ./swarm/launch_swarm.sh)"
echo ""
echo "  Press Ctrl+C to stop all drones."
echo "========================================================"
echo ""

wait
