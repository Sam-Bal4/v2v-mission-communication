#!/usr/bin/env bash
# ============================================================
#  UAV MISSION LAUNCHER
#  Place: mission_2030/uav/start_mission.sh
#
#  Usage (from ANYWHERE after git clone):
#    bash mission_2030/uav/start_mission.sh
#  OR make it executable once:
#    chmod +x mission_2030/uav/start_mission.sh
#    ./mission_2030/uav/start_mission.sh
# ============================================================

set -e   # exit on unexpected errors

# ------ Bash Guard (Forces script to run in Bash if run via 'sh') ------
if [ -z "$BASH_VERSION" ]; then
    exec bash "$0" "$@"
fi

# ------ Resolve REPO ROOT regardless of where you launch from ------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
MISSION_ROOT="$REPO_ROOT/mission_2030"
VENV_PATH="$REPO_ROOT/.venv"

# ------ Find Python Command ------
PY_CMD="python3"
if ! command -v "$PY_CMD" &>/dev/null; then
    PY_CMD="python"
fi
if ! command -v "$PY_CMD" &>/dev/null; then
    echo -e "${RED}ERROR: Neither python3 nor python found. Install it first.${NC}"
    exit 1
fi

# ------ Create/Activate Virtual Environment ------
if [ ! -d "$VENV_PATH" ]; then
    echo -e "${YELLOW}Virtual environment not found. Creating one at $VENV_PATH...${NC}"
    # Use --system-site-packages so we can access Jetson's pyzed/cv2 libs
    "$PY_CMD" -m venv --system-site-packages "$VENV_PATH"
fi

# Activate the venv (handle bin/activate vs Scripts/activate)
ACTIVATE_SCRIPT="$VENV_PATH/bin/activate"
[ ! -f "$ACTIVATE_SCRIPT" ] && ACTIVATE_SCRIPT="$VENV_PATH/Scripts/activate"

# shellcheck source=/dev/null
source "$ACTIVATE_SCRIPT"

# ------ Verify Dependencies ------
# We use a sentinel file AND a module health check (for 'past' which dronekit needs)
SENTINEL="$VENV_PATH/venv_ready"
ENV_OK=true
[ ! -f "$SENTINEL" ] && ENV_OK=false
if [ "$ENV_OK" = true ]; then
    if ! python -c "import past, numpy; assert int(numpy.__version__.split('.')[0]) < 2" &>/dev/null; then
        echo -e "${YELLOW}Environment health check failed (NumPy >= 2.x detected or missing modules). Repairing...${NC}"
        ENV_OK=false
    fi
fi

if [ "$ENV_OK" = false ]; then
    echo -e "${YELLOW}Environment incomplete or corrupted. Installing dependencies...${NC}"
    python -m pip install --upgrade pip
    python -m pip install -e "$REPO_ROOT"
    touch "$SENTINEL"
    echo -e "${GREEN}Environment ready!${NC}"
else
    echo -e "${CYAN}Using existing virtual environment at $VENV_PATH${NC}"
fi

# ------ Export PYTHONPATH ------
export PYTHONPATH="$REPO_ROOT"

# ------ Main menu ------
echo -e "${BOLD}Select what to run:${NC}"
echo ""
echo -e "  ${YELLOW}--- COMPETITION MISSIONS ---${NC}"
echo "  [1]  Mission 1 – Takeoff, land on moving UGV, 30s ride"
echo "  [2]  Mission 2 – Scan field, send coords to UGV, land on UGV"
echo "  [3]  Mission 3 – Mission 2 + obstacle avoidance relay"
echo ""
echo -e "  ${YELLOW}--- HARDWARE TESTS (run in order) ---${NC}"
echo "  [t1]  Test 01 – Basic flight: arm / hover / land"
echo "  [t2]  Test 02 – ZED camera + ArUco detection"
echo "  [t3]  Test 03 – ESP32 V2V link: send command to UGV"
echo "  [t4]  Test 04 – ZED point cloud to UGV destination"
echo "  [t5]  Test 05 – Forward pacing with UGV"
echo "  [t6]  Test 06 – Left sway pacing with UGV"
echo "  [t7]  Test 07 – Right sway pacing with UGV"
echo "  [t8]  Test 08 – Proportional centering hover (high, 1.3m)"
echo "  [t9]  Test 09 – Proportional centering hover (low, 0.5m)"
echo "  [t10] Test 10 – Full precision landing (LANDING_TARGET stream)"
echo "  [t11] Test 11 – Radio commander: arm UGV & obstacle avoidance (no drone flight)"
echo ""
echo "  [q]  Quit"
echo ""
read -rp "  Your choice: " CHOICE

run_script() {
    local SCRIPT="$1"
    echo ""
    echo -e "${GREEN}Running: python $SCRIPT${NC}"
    echo -e "${YELLOW}>>> Press Ctrl+C at any time — drone will land safely <<<${NC}"
    echo ""
    python "$SCRIPT"
    echo ""
    echo -e "${GREEN}Script finished.${NC}"
}

case "$CHOICE" in
    1)  run_script "$MISSION_ROOT/uav/mission1_runner.py" ;;
    2)  run_script "$MISSION_ROOT/uav/mission2_runner.py" ;;
    3)  run_script "$MISSION_ROOT/uav/mission3_runner.py" ;;
    t1) run_script "$MISSION_ROOT/dennis_test/drone/01_basic_flight.py" ;;
    t2) run_script "$MISSION_ROOT/dennis_test/drone/02_aruco_land.py" ;;
    t3) run_script "$MISSION_ROOT/dennis_test/drone/03_move_ugv_5ft.py" ;;
    t4) run_script "$MISSION_ROOT/dennis_test/drone/04_ugv_to_aruco.py" ;;
    t5) run_script "$MISSION_ROOT/dennis_test/drone/05_fly_side_by_side_forward.py" ;;
    t6) run_script "$MISSION_ROOT/dennis_test/drone/06_fly_side_by_side_left.py" ;;
    t7) run_script "$MISSION_ROOT/dennis_test/drone/07_fly_side_by_side_right.py" ;;
    t8) run_script "$MISSION_ROOT/dennis_test/drone/08_center_hover_high.py" ;;
    t9) run_script "$MISSION_ROOT/dennis_test/drone/09_center_hover_low.py" ;;
    t10) run_script "$MISSION_ROOT/dennis_test/drone/10_precision_land.py" ;;
    t11) run_script "$MISSION_ROOT/dennis_test/drone/11_arm_ugv_avoid.py" ;;
    q|Q) echo "Exiting."; exit 0 ;;
    *) echo -e "${RED}Invalid choice. Exiting.${NC}"; exit 1 ;;
esac
