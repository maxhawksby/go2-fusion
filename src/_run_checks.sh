#!/bin/bash
# Inside the container: rebuild the panel, run every package's tests and the linters.
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
cd /ros2_ws && colcon build --symlink-install --packages-select go2_rviz_plugins go2_robot_sdk \
  --event-handlers console_cohesion- status- 2>&1 | grep -E 'Finished|Failed|error' | head -4
cd /ros2_ws/src
for p in go2_robot_sdk go2_auto_mapper object_detection lidar_processor go2_sim; do
  echo "===== $p"
  (cd $p && timeout 300 python3 -m pytest -q -p no:cacheprovider test 2>&1 \
     | grep -E '^(FAILED|ERROR)|passed|failed' | cut -c1-150 | head -8)
done
cd /ros2_ws/src/go2_auto_mapper
echo "== pep257 on the files Luis added"
ament_pep257 go2_auto_mapper/nav_log.py go2_auto_mapper/social_space.py go2_auto_mapper/socialnav.py \
  go2_auto_mapper/socialnav_view.py go2_auto_mapper/vlm_ranking.py go2_auto_mapper/vlm_view.py \
  test/test_vlm_explorer.py test/test_yield_for_people.py test/test_nav_log.py \
  test/test_social_space.py test/test_socialnav.py test/test_vlm_ranking.py 2>&1 \
  | grep -E ': D[0-9]+' | cut -c1-140 | head -12
echo "== flake8 go2_auto_mapper"
ament_flake8 . 2>&1 | grep -E '^\./.*:[0-9]+:' | head -8
cd /ros2_ws/src/object_detection
echo "== object_detection pep257 count"; ament_pep257 . 2>&1 | grep -cE ': D[0-9]+'
echo "== object_detection flake8"; ament_flake8 . 2>&1 | grep -E '^\./.*:[0-9]+:' | head
cd /ros2_ws/src/go2_robot_sdk
echo "== go2_robot_sdk flake8 on changed files"
ament_flake8 go2_robot_sdk/main.py go2_robot_sdk/infrastructure/webrtc/webrtc_adapter.py test/test_connection_health.py 2>&1 | grep -E ':[0-9]+:[0-9]+:' | head
