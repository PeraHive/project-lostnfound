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

  ros2 run bringup launch_sequence
  
  ros2 run lostnfound rssi_sim --ros-args -p mode:=noisy 

# Launch

ros2 launch lostnfound bringup.launch.py

SITL: 
ros2 launch lostnfound bringup.launch.py \
  sim:=true \
  sitl_fcu_url:=udp://:14551@127.0.0.1:14551 \
  sitl_gcs_url:=udp://:14550@127.0.0.1:14550

Real hardware:
ros2 launch lostnfound bringup.launch.py \
  sim:=false \
  mavros_ns:=/mavros \
  hw_fcu_serial_url:=serial:///dev/ttyUSB0:115200 \
  hw_gcs_url:=udp://@0.0.0.0:14550 \
  tgt_system:=1 \
  tgt_component:=1
