#!/bin/bash
#
# G1 Simulation Test Quick Start
#
# Usage:
#   ./start_sim_test.sh keyboard   # 키보드 제어 테스트
#   ./start_sim_test.sh quest      # Meta Quest 텔레오퍼레이션 테스트
#

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=============================================="
echo "  G1 Simulation Test"
echo "=============================================="

# Check mode
MODE=${1:-keyboard}

case $MODE in
    keyboard|kb)
        echo "Mode: Keyboard Control"
        echo ""
        echo "Step 1: Start MuJoCo simulation (in this terminal)"
        echo "  python $SCRIPT_DIR/run_simulation.py"
        echo ""
        echo "Step 2: Start keyboard control (in another terminal)"
        echo "  python $SCRIPT_DIR/teleop_keyboard.py"
        echo ""
        echo "Starting simulation..."
        echo "=============================================="
        python "$SCRIPT_DIR/run_simulation.py"
        ;;

    quest|vr|xr)
        echo "Mode: Meta Quest Teleoperation"
        echo ""
        echo "Prerequisites:"
        echo "  1. Meta Quest connected to same network"
        echo "  2. SSL certificates configured (~/.config/xr_teleoperate/)"
        echo "  3. Image server running on robot/simulation"
        echo ""
        echo "Step 1: Start MuJoCo simulation (in this terminal)"
        echo "  python $SCRIPT_DIR/run_simulation.py"
        echo ""
        echo "Step 2: Start teleop (in another terminal)"
        echo "  python $SCRIPT_DIR/teleop_hybrid.py --sim"
        echo ""
        echo "Starting simulation..."
        echo "=============================================="
        python "$SCRIPT_DIR/run_simulation.py"
        ;;

    help|--help|-h)
        echo ""
        echo "Usage: $0 [mode]"
        echo ""
        echo "Modes:"
        echo "  keyboard, kb    Keyboard control test"
        echo "  quest, vr, xr   Meta Quest teleoperation test"
        echo ""
        ;;

    *)
        echo "Unknown mode: $MODE"
        echo "Use '$0 help' for usage"
        exit 1
        ;;
esac
