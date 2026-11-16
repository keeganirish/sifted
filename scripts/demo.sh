#!/usr/bin/env bash
# Starts the demo pieces in order (docs/demo.md): daemon, impairment proxy,
# sender; then waits, stops everything and checks the replay is identical.
# Serve the map first, (cd web && python3 -m http.server 8000), and open
# http://localhost:8000/. (The app, ./sifted, runs the same demo with one click.)
#
#   scripts/demo.sh [build/release/src/apps/sifted_fusiond] [speed]
set -euo pipefail
cd "$(dirname "$0")/.."
FUSIOND=${1:-build/release/src/apps/sifted_fusiond}
SPEED=${2:-1}
OUT=${DEMO_OUT:-/tmp/sifted-demo}
mkdir -p "$OUT"
[[ -x $FUSIOND ]] || { echo "build first: cmake --preset release && cmake --build --preset release" >&2; exit 1; }
# Outage 70-78 s of scenario time, expressed in proxy (wall-clock) seconds.
OUT_A=$(python3 -c "print(70/$SPEED)")
OUT_B=$(python3 -c "print(78/$SPEED)")
DURATION=$(python3 -c "print(150/$SPEED + 5)")

"$FUSIOND" --sensors tests/fixtures/scenarios/demo/sensors.cfg --listen 127.0.0.1:5600 \
    --record "$OUT/demo.siftlog" --output "$OUT/live.txt" --stats-interval 5 &
DAEMON=$!
trap 'kill $DAEMON 2>/dev/null || true' EXIT
sleep 0.5
(cd sim && python3 -m sifted_sim.impair --listen 127.0.0.1:5601 --target 127.0.0.1:5600 \
    --seed 7 --loss 0.03 --burst 0.01 0.25 0.6 --delay-ms 50 --jitter-ms 150 --duplicate 0.02 \
    --outage "$OUT_A:$OUT_B" --duration "$DURATION") &
PROXY=$!
sleep 0.3
echo "demo: map: http://localhost:8000/ (cd web && python3 -m http.server 8000); outage at scenario time 70-78 s" >&2
(cd sim && python3 -m sifted_sim.send demo --target 127.0.0.1:5601 --speed "$SPEED")
wait $PROXY
sleep 1
kill -INT $DAEMON
wait $DAEMON || true
trap - EXIT
"$FUSIOND" --replay "$OUT/demo.siftlog" --output "$OUT/replay.txt" --ws-port 0
cmp "$OUT/live.txt" "$OUT/replay.txt" && echo "demo: replay identical to the live run ($OUT)"
