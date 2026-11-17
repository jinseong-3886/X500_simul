#!/bin/bash
# Start PX4 SITL (gz_x500, headless) as a daemon. Does NOT kill other processes.
# Log: /tmp/px4_sitl.log
if pgrep -f "bin/px4" >/dev/null; then echo "PX4 SITL already running"; exit 0; fi
cd ~/PX4-Autopilot/build/px4_sitl_default || exit 1
export HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=default
nohup ./bin/px4 -d > /tmp/px4_sitl.log 2>&1 &
for i in $(seq 1 60); do
  sleep 2
  if grep -q "Ready for takeoff" /tmp/px4_sitl.log; then echo "READY after $((i*2)) s"; break; fi
done
tail -15 /tmp/px4_sitl.log
