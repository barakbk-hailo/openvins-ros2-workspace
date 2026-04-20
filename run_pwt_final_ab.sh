#!/usr/bin/env bash
# Final A/B test: max-interval with vs without Docker RT flags.
# Both use openvins-humble-maxinterval (same image — same code, same sync
# constraint). The only difference is the Docker flags.
#
# Run sequentially to minimize system-load differences between the two.

set -eo pipefail

echo "=========================================================="
echo "  Final A/B: max-interval with vs without Docker RT flags"
echo "=========================================================="

echo ""
echo ">>> A: max-interval WITHOUT RT flags >>>"
bash ~/workspace/run_pwt_benchmark_v2.sh pwt_final_maxinterval openvins-humble-maxinterval 10

echo ""
echo ">>> B: max-interval WITH RT flags >>>"
bash ~/workspace/run_pwt_benchmark_v2.sh pwt_final_combined openvins-humble-maxinterval 10 \
    --cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 --cpuset-cpus=0-3

echo ""
echo "=========================================================="
echo "  Done — see results/rpi5/pwt_final_{maxinterval,combined}/"
echo "=========================================================="
