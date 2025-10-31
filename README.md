# Project LostnFound

colcon build --symlink-install 
source install/setup.bash

ros2 launch foxglove_bridge foxglove_bridge_launch.xml


source ~/venv-ardupilot/bin/activate
sim_vehicle.py -v ArduCopter -f quad \
  --custom-location=7.252591,80.592751,0,0 \
  --out=127.0.0.1:14550 \
  --out=127.0.0.1:14551

ros2 run mavros mavros_node --ros-args \
    -p fcu_url:="udp://:14551@127.0.0.1:14551" \
    -r __ns:=/uav1

  ros2 launch foxglove_bridge foxglove_bridge_launch.xml


  ros2 launch lostnfound bringup.launch.py

  ros2 run bringup launch_sequence
