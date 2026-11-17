#!/bin/bash
# Match SITL params to the real KOA FC values read on 2026-11-13 (fc_check.py). SITL only.
cd ~/PX4-Autopilot/build/px4_sitl_default || exit 1
pgrep -f "bin/px4" >/dev/null || { echo "SITL not running"; exit 1; }
P=./bin/px4-param
echo "before:"; $P show NAV_DLL_ACT; $P show COM_OF_LOSS_T; $P show COM_OBL_RC_ACT
$P set NAV_DLL_ACT 0       # real FC: 0
$P set COM_OF_LOSS_T 1.0   # real FC: 1.0
$P set COM_OBL_RC_ACT 0    # real FC: 0
echo "after:"; $P show NAV_DLL_ACT; $P show COM_OF_LOSS_T; $P show COM_OBL_RC_ACT
sleep 3; tail -5 /tmp/px4_sitl.log
