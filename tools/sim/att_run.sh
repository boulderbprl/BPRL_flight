#!/bin/sh
# Build and run the attitude-loop sim. See README.md ("Attitude sim").
#   tools/sim/att_run.sh [DRONE] [MODE ARGS...]     (default: Drone2 margins)
#   CFG=path/to/drone_config.cpp tools/sim/att_run.sh Drone2 step   # try a config copy
set -e
SIM=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$SIM/../.." && pwd)
DRONE=${1:-Drone2}; [ $# -gt 0 ] && shift
CFG=${CFG:-$REPO/configs/$DRONE/drone_config.cpp}
BIN=${BIN:-$(mktemp -d)/att_sim}
C=$REPO/src/controllers
${CXX:-g++} -std=gnu++14 -O2 -Wall -Wextra -I$SIM/stubs -I$REPO -o "$BIN" "$SIM/att_sim.cpp" \
    $C/extras/PID.cpp $C/position/AltControl.cpp $C/position/PosControl.cpp $REPO/src/math/math.cpp "$CFG" \
    $C/position/HeadingControl.cpp $C/position/TrajectoryTracker.cpp $C/FlightStateMachine.cpp \
    $C/attitude/Attitude_PID.cpp $C/attitude/Attitude_INDI.cpp \
    $C/attitude/Attitude_JINDI.cpp $C/attitude/Attitude_PID_Jerk.cpp $C/attitude/Attitude_PID_PI.cpp \
    $C/extras/Unmixer.cpp $C/extras/SysId.cpp $C/extras/MotorMixer.cpp
[ $# -eq 0 ] && set -- margins
"$BIN" "$@"
