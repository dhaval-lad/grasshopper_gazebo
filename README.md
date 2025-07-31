# grasshopper_gazebo

## Description
![3D Simulation Environment](models/3D%20Simulation%20Environment)

The `grasshopper_gazebo`package is responsible for simulating 4-WD robot called GrassHopper in different simulation environment in Gazebo simulator. 

## Dependencies
- [Ubuntu 22.04](https://releases.ubuntu.com/22.04/)
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [actor_colision_plugin](https://github.com/dhaval-lad/actor_collision_plugin.git)
- [grasshopper_description](https://github.com/dhaval-lad/grasshopper_description.git)

## Installation Instructions
1. First, clone this repository inside the `src` folder of your ROS 2 workspace (replace `ros2_ws` with the name of your ROS 2 workspace):
    ```sh
    cd ~/ros2_ws/src
    git clone https://github.com/dhaval-lad/actor_collision_plugin.git
    ```
2. The package consist of 2 models "actor_1" and "actor_2" for simulating walking person. Replace there path in `smalltown_final.world`, line number 6417 and 6421 with the path of those models in your system. And copy the remaining models to `.gazebo` by following command (Replace `ros2_ws` with the name of your ROS2 workspace):
    ```sh
    cd ~/ros2_ws/src/grasshopper_gazebo/models
    cp -r road_model/. ~/.gazebo/models
    ```
3. Next, build your ROS 2 workspace to install the package (replace `ros2_ws` with the name of your ROS 2 workspace):
    ```sh
    cd ~/ros2_ws
    colcon build --packages-select grasshopper_gazebo
    ```

## Usage Instructions
To verify that everything is working:
1. Run the following command in the terminal to visualize the GrassHopper robot in outdoor simulation environment in Gazebo:
    ```sh
    ros2 launch grasshopper_gazebo gazebo.launch.py 
    ```

## License
This project is licensed under the Apache License, Version 2.0, January 2004. See [http://www.apache.org/licenses/](http://www.apache.org/licenses/) for more details.