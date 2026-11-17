#!/bin/bash
# Run one full Flytrap sim episode: fresh sim -> fly_mission_px4.py --live --cam gz -> ground-truth analysis.
#   bash run_mission.sh clean|patched [T_MAX] [extra mission args...]
#   POS=ev bash run_mission.sh patched 45     # GPS off, position from (simulated) mocap + mocap geofence
cd ~/flytrap_sim
KIND=${1:-clean}; TMAX=${2:-45}; shift 2 2>/dev/null; EXTRA="$@"
POS=${POS:-gps}
POS=$POS bash flytrap_sim.sh start $KIND 2>&1 | tail -8
if [ "$POS" = "ev" ]; then
  # board at gz world x=2.5 (east) -> NED (N=0, E=2.5); flight box ~ 3 m cube around the run line
  EXTRA="--mocap gz:flytrap_$KIND:x500_d435i_0 --board-ned=0,2.5 --fence=-1.5,1.5,-0.7,2.4,2.0 $EXTRA"
fi
OUT=~/flytrap_sim/out/${KIND}_$POS; rm -rf $OUT; mkdir -p $OUT/frames
venv/bin/python -u fly_mission_px4.py --target bus --live --conn udpin:0.0.0.0:14540 --cam gz \
  --model yolov8n.pt --tgt-h 0.396 --t-max $TMAX --out $OUT --save-frames $OUT/frames $EXTRA \
  > $OUT/mission.log 2>&1
grep -v -i warn $OUT/mission.log | grep -E "mocap|link|intrinsics|calib|ACQUIRED|DECEIT|HALT|RESULT|pilot|abort|LAND|px4\]" | head -30
echo "--- every ~2 s:"
grep -E "^\[[0-9.]+\] (SEARCH|FOLLOW|SAFETY_HALT)" $OUT/mission.log | awk 'NR%8==1' | cut -c1-110
sleep 8   # let it land
venv/bin/python analyze_episode.py ${KIND}_$POS
