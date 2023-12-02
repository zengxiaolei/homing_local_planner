# homing_local_planner ROS Package

**A simple, easy-to-use, and effective path tracking planner with a complete demo.**

The homing_local_planner package implements a plug-in to the *nav_core::BaseLocalPlanner* of the 2D navigation stack in ROS1 or a plug-in to the *nav2_core::Controller* of the  Nav2 in ROS2. The underlying method called Homing Control has the objective to guide a robot along a reference path, which is a pure pursuit algorithm Implemented based on [1]  as reference. And code implementation of the package has some reference to [teb_local_planner](http://wiki.ros.org/teb_local_planner).

This scheme considers a dynamic goal pose on the path located some distance ahead of the robots current pose. The robot is supposed to chase the moving goal pose (look-ahead pose) on the path. This path tracking strategy is similar to human drivers that steer a vehicle towards a dynamic lookahead point on the road, which distance depends on the vehicle speed, road curvature and visibility. 



##  Install 

Git clone this repository  and checkout the corresponding branch, then compile and run.

```shell
git clone https://github.com/zengxiaolei/homing_local_planner.git
```



## Run and Demo

There's a complete demo based on 2D stage simulator in this package. Firstly make sure the simulation platform is installed.

- For ROS2: [stage_ros2](https://github.com/n0nzzz/stage_ros2)

- For ROS1:  [stage_ros](https://github.com/ros-simulation/stage_ros)




Then you can launch it easily by following command:

- For ROS2:


```shell
ros2 run stage_ros stageros /home/.../homing_local_planner/test/stage/maze_diff_drive.world
ros2 launch homing_local_planner demo.launch.py
```


- For ROS1:

```
roslaunch homing_local_planner demo.launch
```



Dyamic gif demo is as following.

If there's a problem with display, you can check file path: /README_img/homing_demo.gif

![homing_demo](./README_img/homing_demo.gif)



## Todo

- The obstacle avoidance function is incomplete and is still under development.
- The documentation for the parameters is still in preparation, but you can get an idea of what it does based on the name of them.
- Support more rosversions.



## References

[1] Astolfi, A., Exponential Stabilization of a Wheeled Mobile Robot Via Discontinuous
Control, Journal of Dynamic Systems, Measurement, and Control, vol. 121, 1999

[2] C. Rösmann, F. Hoffmann and T. Bertram: Integrated online trajectory planning and optimization in distinctive topologies, Robotics and Autonomous Systems, Vol. 88, 2017, pp. 142–153.

[3] Mobile Robot Course of The Institute of Control Theory and Systems Engineering at TU Dortmund



## License

The *homing_local_planner* package is licensed under the **BSD 3-Clause** license. It depends on other ROS packages, which are listed in the package.xml. They are also BSD licensed.

Some third-party dependencies are included that are licensed under different terms:

- *Eigen*, MPL2 license, [http://eigen.tuxfamily.org](http://eigen.tuxfamily.org/)
- *Boost*, Boost Software License, [http://www.boost.org](http://www.boost.org/)

All packages included are distributed in the hope that they will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 