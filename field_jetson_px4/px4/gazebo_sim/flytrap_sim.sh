#!/bin/bash
# Flytrap PX4 + Gazebo Harmonic sim launcher (WSL). Uses its own working dir (~/flytrap_sim/rootfs) so the
# shared PX4 SITL parameters (other projects) are never touched.
#   bash flytrap_sim.sh start clean|patched [gui]
#   bash flytrap_sim.sh stop
#   bash flytrap_sim.sh status
set -u
ROOT=~/flytrap_sim
PX4B=~/PX4-Autopilot/build/px4_sitl_default
PIDS=$ROOT/pids
mkdir -p $ROOT/rootfs $ROOT/logs

stop() {
  if [ -f $PIDS ]; then
    while read -r p; do kill "$p" 2>/dev/null && echo "stopped pid $p"; done < $PIDS
    rm -f $PIDS; sleep 3
  fi
}

case "${1:-}" in
stop) stop; exit 0 ;;
status) [ -f $PIDS ] && { cat $PIDS; ps -o pid,cmd -p $(paste -sd, $PIDS) 2>/dev/null; } || echo "not running"; exit 0 ;;
start) ;;
*) echo "usage: $0 start clean|patched [gui] | stop | status"; exit 1 ;;
esac

KIND=${2:-clean}; WORLD=flytrap_$KIND
stop
if pgrep -f "bin/px4" >/dev/null || pgrep -f "gz sim" >/dev/null; then
  echo "another PX4/gz sim is running (not ours) -> refusing to start (would clash on ports)"; pgrep -af "bin/px4|gz sim"; exit 1
fi

export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH:-} GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
set +u; . $PX4B/rootfs/gz_env.sh; set -u
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$ROOT/models:$ROOT/worlds

echo "[1/4] gz sim world $WORLD"
gz sim -r -s --headless-rendering $ROOT/worlds/$WORLD.sdf > $ROOT/logs/gz.log 2>&1 &
echo $! >> $PIDS
for i in $(seq 1 60); do gz topic -l 2>/dev/null | grep -q "^/world/$WORLD/clock" && break; sleep 1; done
gz topic -l | grep -q "^/world/$WORLD/clock" || { echo "gz world did not come up"; tail -20 $ROOT/logs/gz.log; exit 1; }
if [ "${3:-}" = "gui" ]; then gz sim -g > /dev/null 2>&1 & echo $! >> $PIDS; fi

POS=${POS:-gps}   # gps (default) | ev  (ev = GPS off, position only from mocap via fly_mission_px4.py --mocap)

start_px4() {
  cd $ROOT/rootfs
  rm -f $ROOT/rootfs/etc
  PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_NAME=x500_d435i_0 PX4_GZ_WORLD=$WORLD HEADLESS=1 \
    nohup $PX4B/bin/px4 -d $PX4B/etc -w $ROOT/rootfs > $ROOT/logs/px4.log 2>&1 &
  echo $! > $ROOT/px4.pid; echo $! >> $PIDS
  for i in $(seq 1 60); do grep -q "Startup script returned" $ROOT/logs/px4.log && break; sleep 1; done
  grep -q "Startup script returned" $ROOT/logs/px4.log || { echo "PX4 did not start"; tail -25 $ROOT/logs/px4.log; exit 1; }
}
P=$PX4B/bin/px4-param

echo "[2/4] PX4 SITL (x500 airframe, attach to x500_d435i_0)"
start_px4

echo "[3/4] params (this sim's own rootfs only): real-FC match + position source '$POS'"
$P set NAV_DLL_ACT 0 >/dev/null      # real KOA FC: 0
$P set COM_OF_LOSS_T 1.0 >/dev/null  # real KOA FC: 1.0
$P set COM_OBL_RC_ACT 0 >/dev/null   # real KOA FC: 0
if [ "$POS" = "ev" ]; then
  $P set EKF2_GPS_CTRL 0 >/dev/null  # no GNSS indoors
  $P set EKF2_EV_CTRL 11 >/dev/null  # bits: horiz pos(1) + vert pos(2) + yaw(8)
  $P set EKF2_HGT_REF 3 >/dev/null   # height reference = vision (reboot)
  $P set EKF2_MAG_TYPE 5 >/dev/null  # no magnetometer, yaw from mocap (reboot)
  $P set EKF2_EV_DELAY 0 >/dev/null
else
  $P set EKF2_GPS_CTRL 7 >/dev/null; $P set EKF2_EV_CTRL 0 >/dev/null
  $P set EKF2_HGT_REF 1 >/dev/null;  $P set EKF2_MAG_TYPE 0 >/dev/null
fi
$P save
echo "[4/4] restart PX4 so reboot-required params take effect"
kill $(cat $ROOT/px4.pid); sleep 4
grep -v "^$(cat $ROOT/px4.pid)\$" $PIDS > $PIDS.tmp; mv $PIDS.tmp $PIDS
start_px4
for p in NAV_DLL_ACT EKF2_GPS_CTRL EKF2_EV_CTRL EKF2_HGT_REF EKF2_MAG_TYPE; do $P show $p | grep -E "^\s*x.* $p " ; done
if [ "$POS" = "gps" ]; then
  for i in $(seq 1 60); do grep -q "Ready for takeoff" $ROOT/logs/px4.log && break; sleep 1; done
  grep -q "Ready for takeoff" $ROOT/logs/px4.log || { echo "PX4 not ready for takeoff"; tail -15 $ROOT/logs/px4.log; exit 1; }
else
  echo "(ev mode: PX4 gets a position only once fly_mission_px4.py --mocap starts streaming)"
fi
echo "camera topics:"; gz topic -l | grep flytrap
echo "READY: world=$WORLD pos=$POS  (MAVLink onboard: udpin:0.0.0.0:14540)"
