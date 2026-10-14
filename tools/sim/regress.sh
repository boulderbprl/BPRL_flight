#!/bin/sh
# Refactor regression for the POS_HOLD / ALT_HOLD stick paths: builds
# regress.cpp against the sources at a git ref and against the working tree,
# and compares every controller output of a 40 s scripted flight.
#   tools/sim/regress.sh [GIT_REF] [DRONE]      (default: HEAD, Drone3)
# "IDENTICAL" means PosControl::update() and AltControl::alt_hold() behave
# exactly as they did at GIT_REF.
set -e
SIM=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$SIM/../.." && pwd)
REF=${1:-HEAD}
DRONE=${2:-Drone3}
TMP=$(mktemp -d)
mkdir -p "$TMP/base"
git -C "$REPO" archive "$REF" src configs | tar -x -C "$TMP/base"

build() {   # $1 = source root, $2 = output binary
    ${CXX:-g++} -std=gnu++14 -O1 -ffp-contract=off -I"$SIM/stubs" -I"$1" -o "$2" "$SIM/regress.cpp" \
        "$1/src/controllers/PosControl.cpp" "$1/src/controllers/AltControl.cpp" \
        "$1/src/controllers/PID.cpp" "$1/src/math/math.cpp" "$1/configs/$DRONE/drone_config.cpp"
}
build "$TMP/base" "$TMP/regress_base"
build "$REPO"     "$TMP/regress_new"
"$TMP/regress_base" > "$TMP/base.txt"
"$TMP/regress_new"  > "$TMP/new.txt"
if cmp -s "$TMP/base.txt" "$TMP/new.txt"; then
    echo "IDENTICAL to $REF ($(wc -l < "$TMP/new.txt") ticks, $DRONE)"
else
    echo "DIFFERENT from $REF — first differing tick:"
    cmp "$TMP/base.txt" "$TMP/new.txt" || true
    echo "outputs kept in $TMP"
    exit 1
fi
