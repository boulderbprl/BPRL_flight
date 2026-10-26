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
COMMON="$C/extras/PID.cpp $C/position/AltControl.cpp $C/position/PosControl.cpp $REPO/src/math/math.cpp $REPO/configs/$DRONE/drone_config.cpp"

$CXX -o "$OUT/sim" "$SIM/sim.cpp" $COMMON \
    $C/position/HeadingControl.cpp $C/position/TrajectoryTracker.cpp $C/FlightStateMachine.cpp \
    $C/attitude/Attitude_PID.cpp $C/attitude/Attitude_INDI.cpp \
    $C/attitude/Attitude_JINDI.cpp $C/attitude/Attitude_PID_Jerk.cpp $C/attitude/Attitude_PID_PI.cpp $C/extras/Unmixer.cpp $C/extras/SysId.cpp
"$OUT/sim" "$OUT"
echo "CSV output in $OUT"
