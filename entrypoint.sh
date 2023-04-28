#!/bin/bash
source /opt/ros/melodic/setup.bash
source /jackal_ws/devel_isolated/setup.bash
cd /jackal_ws/src/robust_fast_navigation/scripts
exec ${@:1}
