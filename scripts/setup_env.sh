#!/bin/bash
# G1 Inspire Pipeline 환경 설정 스크립트

set -e

echo "=========================================="
echo "G1 Inspire Pipeline Setup"
echo "=========================================="

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 프로젝트 루트
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

# 1. Git submodules 초기화
echo -e "\n${YELLOW}[1/5] Initializing git submodules...${NC}"
git submodule update --init --recursive
echo -e "${GREEN}Done${NC}"

# 2. Conda 환경 확인
echo -e "\n${YELLOW}[2/5] Checking conda environment...${NC}"
if ! command -v conda &> /dev/null; then
    echo -e "${RED}Conda not found. Please install Anaconda/Miniconda first.${NC}"
    exit 1
fi

ENV_NAME="g1"
if conda env list | grep -q "^$ENV_NAME "; then
    echo "Conda environment '$ENV_NAME' already exists"
else
    echo "Creating conda environment '$ENV_NAME'..."
    conda create -n $ENV_NAME python=3.10 -y
fi
echo -e "${GREEN}Done${NC}"

# 3. 기본 패키지 설치
echo -e "\n${YELLOW}[3/5] Installing base packages...${NC}"
eval "$(conda shell.bash hook)"
conda activate $ENV_NAME

pip install --upgrade pip
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
pip install mujoco numpy pyyaml

echo -e "${GREEN}Done${NC}"

# 4. LeRobot 설치 (G1 지원)
echo -e "\n${YELLOW}[4/5] Installing LeRobot with G1 support...${NC}"
cd "$PROJECT_ROOT/external/lerobot"
pip install -e '.[unitree_g1]' || {
    echo -e "${YELLOW}LeRobot unitree_g1 extra failed, installing base...${NC}"
    pip install -e .
}
cd "$PROJECT_ROOT"
echo -e "${GREEN}Done${NC}"

# 5. Unitree SDK2 설치
echo -e "\n${YELLOW}[5/5] Installing Unitree SDK2...${NC}"
if [ -d "$PROJECT_ROOT/external/unitree_mujoco/unitree_sdk2_python" ]; then
    cd "$PROJECT_ROOT/external/unitree_mujoco"
    # SDK2 Python 설치는 CycloneDDS 필요
    echo "Note: Unitree SDK2 requires CycloneDDS. See:"
    echo "  https://github.com/unitreerobotics/unitree_sdk2_python"
else
    echo "unitree_sdk2_python not found in unitree_mujoco"
fi
cd "$PROJECT_ROOT"
echo -e "${GREEN}Done${NC}"

# 완료 메시지
echo -e "\n${GREEN}=========================================="
echo "Setup complete!"
echo "==========================================${NC}"
echo ""
echo "사용 방법:"
echo "  conda activate g1"
echo ""
echo "시뮬레이션 실행:"
echo "  python scripts/run_mujoco_sim.py"
echo ""
echo "학습 실행 (RL):"
echo "  cd external/unitree_rl_gym"
echo "  python legged_gym/scripts/train.py --task=g1"
echo ""
echo "학습 실행 (IL):"
echo "  cd external/unitree_IL_lerobot"
echo "  python train.py --policy act"
