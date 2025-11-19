set -e

colcon build --packages-skip px4_msgs microxrcedds_agent --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
