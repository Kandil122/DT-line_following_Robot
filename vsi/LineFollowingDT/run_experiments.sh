#!/bin/bash
set -e

# Always run from the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Check required programs
command -v python3 >/dev/null 2>&1 || { echo "python3 not found"; exit 127; }
command -v vsiSim >/dev/null 2>&1 || { echo "vsiSim not found in PATH (source your VSI environment)"; exit 127; }

echo "====================================================="
echo " Starting Digital Twin Experiments..."
echo "====================================================="

ROOT_DIR="$SCRIPT_DIR"

DT_FILE="${ROOT_DIR}/LineFollowingDT.dt"

OUT_PLOTS="${ROOT_DIR}/output/plots"
OUT_LOGS="${ROOT_DIR}/output/logs"
OUT_CSV="${ROOT_DIR}/output/results/results.csv"

mkdir -p "${OUT_PLOTS}"
mkdir -p "${OUT_LOGS}"
mkdir -p "$(dirname "${OUT_CSV}")"

rand_uniform() {
python3 - "$@" <<'PY'
import random, sys
seed = int(sys.argv[1])
min_v = float(sys.argv[2])
max_v = float(sys.argv[3])
rng = random.Random(seed)
print(rng.uniform(min_v, max_v))
PY
}

run_experiment() {

    local run_name=$1
    local experiment=$2
    local path_type=$3
    local kp=$4
    local ki=$5
    local kd=$6
    local noise=$7
    local disturbance=$8
    local seed=$9
    local init_y=${10}
    local init_theta=${11}

    echo ""
    echo "-----------------------------------------------------"
    echo "Running: ${run_name}"
    echo "-----------------------------------------------------"

    # Start VSI server
    echo "Starting VSI..."
    vsiSim "${DT_FILE}" > "${OUT_LOGS}/${run_name}.vsi.log" 2>&1 &
    VSI_PID=$!

    sleep 3

    # Start simulator
    echo "Starting simulator..."
    python3 src/simulator/simulator.py \
        --noise "${noise}" \
        --disturbance "${disturbance}" \
        --seed "${seed}" \
        --init-y "${init_y}" \
        --init-theta "${init_theta}" \
        > "${OUT_LOGS}/${run_name}.sim.log" 2>&1 &
    SIM_PID=$!

    # Start controller
    echo "Starting controller..."
    python3 src/controller/controller.py \
        --path-type "${path_type}" \
        --Kp "${kp}" \
        --Ki "${ki}" \
        --Kd "${kd}" \
        > "${OUT_LOGS}/${run_name}.ctrl.log" 2>&1 &
    CTRL_PID=$!

    # Run visualizer (foreground)
    echo "Starting visualizer..."
    python3 src/visualizer/visualizer.py \
        --path-type "${path_type}" \
        --Kp "${kp}" \
        --Ki "${ki}" \
        --Kd "${kd}" \
        --noise "${noise}" \
        --disturbance "${disturbance}" \
        --seed "${seed}" \
        --init-y "${init_y}" \
        --init-theta "${init_theta}" \
        --experiment "${experiment}" \
        --run-name "${run_name}" \
        --out-csv "${OUT_CSV}" \
        --save-plot "${OUT_PLOTS}/${run_name}.png" \
        > "${OUT_LOGS}/${run_name}.vis.log" 2>&1

    echo "Cleaning up processes..."

    kill $SIM_PID $CTRL_PID $VSI_PID 2>/dev/null || true
    wait $VSI_PID 2>/dev/null || true

    echo "Run ${run_name} finished."
    sleep 2
}

# Example test run
run_experiment "TEST_RUN" "TEST" "straight" "1.5" "0.05" "0.1" "0.0" "0.0" "1234" "0.0" "0.0"

echo ""
echo "====================================================="
echo " All experiments finished."
echo " Results saved to: ${OUT_CSV}"
echo "====================================================="