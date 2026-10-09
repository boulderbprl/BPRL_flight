#!/bin/sh
# Build and run the host simulations. See README.md in this directory.
#   tools/sim/run.sh [DRONE] [OUT_DIR]      (default: Drone3, a temp dir)
set -e
SIM=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$SIM/../.." && pwd)
DRONE=${1:-Drone3}
OUT=${2:-$(mktemp -d)}
mkdir -p "$OUT"
CXX="${CXX:-g++} -std=gnu++14 -O1 -Wall -Wextra -I$SIM/stubs -I$REPO"
C=$REPO/src/controllers
COMMON="$C/PID.cpp $C/AltControl.cpp $C/PosControl.cpp $REPO/src/math/math.cpp $REPO/configs/$DRONE/drone_config.cpp"

$CXX -o "$OUT/sim" "$SIM/sim.cpp" $COMMON \
    $C/HeadingControl.cpp $C/TrajectoryTracker.cpp $C/FlightStateMachine.cpp \
    $C/Attitude_PID.cpp $C/Attitude_INDI.cpp $C/Attitude_PID_PI.cpp $C/Unmixer.cpp
"$OUT/sim" "$OUT"
echo "CSV output in $OUT"
