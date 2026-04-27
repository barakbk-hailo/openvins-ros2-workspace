#!/usr/bin/env bash
# RPi5 / Docker only — final A/B test: max-interval with vs without Docker RT flags.
# Both use openvins-humble-maxinterval (same image — same code, same sync
# constraint). The only difference is the Docker flags.
#
# Run sequentially to minimize system-load differences between the two.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PWT_SCRIPT="$SCRIPT_DIR/run_pwt_benchmark_v2.sh"
[ -x "$PWT_SCRIPT" ] || [ -f "$PWT_SCRIPT" ] || {
  echo "ERROR: cannot find run_pwt_benchmark_v2.sh next to this script: $PWT_SCRIPT" >&2
  exit 1
}

echo "=========================================================="
echo "  Final A/B: max-interval with vs without Docker RT flags"
echo "=========================================================="

echo ""
echo ">>> A: max-interval WITHOUT RT flags >>>"
bash "$PWT_SCRIPT" pwt_final_maxinterval openvins-humble-maxinterval 10

echo ""
echo ">>> B: max-interval WITH RT flags >>>"
bash "$PWT_SCRIPT" pwt_final_combined openvins-humble-maxinterval 10 \
    --cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 --cpuset-cpus=0-3

echo ""
echo "=========================================================="
echo "  Done — see results/rpi5/pwt_final_{maxinterval,combined}/"
echo "=========================================================="
