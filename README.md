# rbtc_pick_place

Rutina de pick and place con un UR5e en ROS 2 (Kilted), usando MoveIt 2 y
pymoveit2. Dos puntos de recogida, cada uno con puntos intermedios arriba del
pick y del place, simulacion de efector final y payload (caja de colision
adjunta/separada), y espera de una entrada digital (/digital_input) para
repetir el ciclo.

## Dependencias

    pip install pymoveit2
    sudo apt install ros-$ROS_DISTRO-ur ros-$ROS_DISTRO-ur-simulation-gz \
      ros-$ROS_DISTRO-moveit

## Uso

Terminal 1:

    ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e

Terminal 2:

    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true use_sim_time:=true

Terminal 3:

    ros2 run rbtc_pick_place pick_and_place

Para repetir el ciclo despues de que termine, en otra terminal:

    ros2 topic pub --once /digital_input std_msgs/msg/Bool "{data: true}"
