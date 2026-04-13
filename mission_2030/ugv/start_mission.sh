#!/usr/bin/env bash
# ============================================================
#  UGV MISSION LAUNCHER
#  Place: mission_2030/ugv/start_mission.sh
#
#  Usage (from ANYWHERE after git clone):
#    bash mission_2030/ugv/start_mission.sh
#  OR make executable once:
#    chmod +x mission_2030/ugv/start_mission.sh
#    ./mission_2030/ugv/start_mission.sh
# ============================================================

set -e

# ------ Resolve REPO ROOT regardless of where you launch from ------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
MISSION_ROOT="$REPO_ROOT/mission_2030"

# ------ Export PYTHONPATH ------
export PYTHONPATH="$REPO_ROOT"

# ------ Colours ------
RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'
YELLOW='\033[1;33m'; BOLD='\033[1m'; NC='\033[0m'

clear
echo -e "${BOLD}${CYAN}"
echo "  ╔═══════════════════════════════════════╗"
echo "  ║    OPERATION TOUCHDOWN — UGV LAUNCHER ║"
echo "  ║       Pixhawk Rover / Jetson / Pi     ║"
echo "  ╚═══════════════════════════════════════╝"
echo -e "${NC}"
echo -e "  Repo root : ${GREEN}$REPO_ROOT${NC}"
echo -e "  PYTHONPATH: ${GREEN}$PYTHONPATH${NC}"
echo ""

if ! command -v python3 &>/dev/null; then
    echo -e "${RED}ERROR: python3 not found.${NC}"
    exit 1
fi

echo -e "${BOLD}Select what to run:${NC}"
echo ""
echo -e "  ${YELLOW}--- COMPETITION MISSION ---${NC}"
echo "  [1]  UGV Runner – waits for UAV destination, drives, avoids obstacles"
echo ""
echo -e "  ${YELLOW}--- HARDWARE TESTS (pair with matching drone test) ---${NC}"
echo "  [t3]  Test 03 – Receive UAV command → drive 5 ft"
echo "  [t4]  Test 04 – Receive UAV destination vector → drive to ArUco"
echo "  [t5]  Test 05 – Forward drive (pair with drone test 05)"
echo "  [t6]  Test 06 – Turn left  (pair with drone test 06)"
echo "  [t7]  Test 07 – Turn right (pair with drone test 07)"
echo ""
echo "  [q]  Quit"
echo ""
read -rp "  Your choice: " CHOICE

run_script() {
    local SCRIPT="$1"
    echo ""
    echo -e "${GREEN}Running: python3 $SCRIPT${NC}"
    echo -e "${YELLOW}>>> Press Ctrl+C to stop safely <<<${NC}"
    echo ""
    python3 "$SCRIPT"
    echo ""
    echo -e "${GREEN}Script finished.${NC}"
}

case "$CHOICE" in
    1)  run_script "$MISSION_ROOT/ugv/ugv_runner.py" ;;
    t3) run_script "$MISSION_ROOT/dennis_test/groundvehicle/03_move_ugv_5ft.py" ;;
    t4) run_script "$MISSION_ROOT/dennis_test/groundvehicle/04_ugv_to_aruco.py" ;;
    t5) run_script "$MISSION_ROOT/dennis_test/groundvehicle/05_fly_side_by_side_forward.py" ;;
    t6) run_script "$MISSION_ROOT/dennis_test/groundvehicle/06_fly_side_by_side_left.py" ;;
    t7) run_script "$MISSION_ROOT/dennis_test/groundvehicle/07_fly_side_by_side_right.py" ;;
    q|Q) echo "Exiting."; exit 0 ;;
    *) echo -e "${RED}Invalid choice.${NC}"; exit 1 ;;
esac
