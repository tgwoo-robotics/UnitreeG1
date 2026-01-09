#!/bin/bash
# G1 Inspire Pipeline - Environment Setup Script
# Usage: ./setup.sh

set -e

echo "=========================================="
echo "G1 Inspire Pipeline Setup"
echo "=========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo -e "${YELLOW}[1/5] Initializing git submodules...${NC}"
git submodule update --init --recursive
echo -e "${GREEN}Done${NC}"

echo -e "${YELLOW}[2/5] Installing base requirements...${NC}"
pip install -r requirements.txt
echo -e "${GREEN}Done${NC}"

echo -e "${YELLOW}[3/5] Installing Unitree SDK2 Python...${NC}"
if [ -d "external/unitree_mujoco/unitree_sdk2_python" ]; then
    cd external/unitree_mujoco/unitree_sdk2_python
    pip install -e .
    cd "$SCRIPT_DIR"
    echo -e "${GREEN}Done${NC}"
else
    echo -e "${RED}Warning: unitree_sdk2_python not found. Skipping...${NC}"
fi

echo -e "${YELLOW}[4/5] Setting up unitree_mujoco config...${NC}"
# Create default config for G1 simulation
CONFIG_FILE="external/unitree_mujoco/simulate_python/config.py"
if [ -f "$CONFIG_FILE" ]; then
    # Backup original config
    cp "$CONFIG_FILE" "${CONFIG_FILE}.bak"

    # Update config for G1
    cat > "$CONFIG_FILE" << 'EOF'
ROBOT = "g1" # Robot name
ROBOT_SCENE = "../unitree_robots/g1/scene_29dof.xml" # Robot scene
DOMAIN_ID = 1 # Domain id (1=sim, 0=real)
INTERFACE = "lo" # Interface

USE_JOYSTICK = 0 # Disable joystick for teleop mode
JOYSTICK_TYPE = "xbox"
JOYSTICK_DEVICE = 0

PRINT_SCENE_INFORMATION = True
ENABLE_ELASTIC_BAND = True # Virtual spring band - keeps robot from falling

SIMULATE_DT = 0.005
VIEWER_DT = 0.02  # 50 fps
EOF
    echo -e "${GREEN}Done${NC}"
else
    echo -e "${RED}Warning: config.py not found. Skipping...${NC}"
fi

echo -e "${YELLOW}[5/5] Installing optional dependencies...${NC}"
echo "  - For LeRobot (imitation learning):"
echo "    cd external/lerobot && pip install -e '.[unitree_g1]'"
echo ""
echo "  - For XR Teleoperation (Meta Quest):"
echo "    pip install -r external/xr_teleoperate/requirements.txt"
echo ""
echo "  - For Isaac Gym RL:"
echo "    Follow NVIDIA Isaac Gym installation guide"

echo ""
echo -e "${GREEN}=========================================="
echo "Setup Complete!"
echo "==========================================${NC}"
echo ""
echo "Quick Start:"
echo "  1. Start MuJoCo simulation:"
echo "     cd external/unitree_mujoco/simulate_python && python unitree_mujoco.py"
echo ""
echo "  2. In another terminal, run upper body control:"
echo "     python scripts/sim_upper_body_control.py"
echo ""
echo "Controls in simulation window:"
echo "  - Key 7/8: Adjust ElasticBand length"
echo "  - Key 9: Toggle ElasticBand on/off"
echo ""
