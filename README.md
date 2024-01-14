# Robot Manipulation ERL    

This repository contains ongoing research on robot manipulation conducted in the Existential Robotics Laboratory at UC San Diego.

Group Members: Niyas, Aniket, Nikola

# Sourcing 
(At /home/erl-tianyu/Nikola_ws) source virtualenv_Nikola/bin/activate

(At /home/erl-tianyu/Nikola_ws/ros2_ws) source install/setup.bash

# Currently Working on
ros2 launch my_moveit xarm6.launch.py

ros2 run my_moveit my_moveit

# Launch Files
ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py (Real Robot)

ros2 launch xarm_moveit_config xarm6_moveit_fake.launch.py (Simulation)
