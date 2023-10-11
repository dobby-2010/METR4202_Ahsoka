# Waffle pi Test

## In one terminal
```  
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```  
## A1 rIZ
```  
ros2 launch turtlebot3_bringup rviz2.launch.py
```  
## To Control
```  
ros2 run turtlebot3_teleop teleop_keyboard
```  
## Autonomous Collision Aviodance
```  
ros2 run turtlebot3_gazebo turtlebot3_drive
```  
