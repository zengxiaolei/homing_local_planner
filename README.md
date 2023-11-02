# homing_local_planner ROS Package

A simple, easy-to-use, and effective path tracking planner.

The homing_local_planner package implements a plugin to the base_local_planner of the 2D navigation stack. The underlying method called Homing Control has the objective to guide a robot along a reference path, which is a pure pursuit algorithm Implemented based on [1]  as reference. And code implementation of the package has some reference to [teb_local_planner](http://wiki.ros.org/teb_local_planner).

This scheme considers a dynamic goal pose on the path located some distance ahead of the robots current pose. The robot is supposed to chase the moving goal pose (look-ahead pose) on the path. This path tracking strategy is similar to human drivers that steer a vehicle towards a dynamic lookahead point on the road, which distance depends on the vehicle speed, road curvature and visibility. 



##  Install 

```shell
cd ~/your_ros_ws/src
git clone https://github.com/zengxiaolei/homing_local_planner.git
catkin_make
```


## Run and Demo

There's a complete demo in this package and you can launch it easily by following command:

```shell
roslaunch homing_local_planner demo.launch
```
Dyamic gif demo is as following.

If there's a problem with display, you can check file path: /README_img/homing_demo.gif

![homing_demo](./README_img/homing_demo.gif)



## Note

- The obstacle avoidance function is incomplete and is still under development.
- The documentation for the parameters is still in preparation, but you can get an idea of what it does based on the name of them.



## References

[1] Astolfi, A., Exponential Stabilization of a Wheeled Mobile Robot Via Discontinuous
Control, Journal of Dynamic Systems, Measurement, and Control, vol. 121, 1999

[2] C. Rösmann, F. Hoffmann and T. Bertram: Integrated online trajectory planning and optimization in distinctive topologies, Robotics and Autonomous Systems, Vol. 88, 2017, pp. 142–153.

[3] Mobile Robot Course of The Institute of Control Theory and Systems Engineering at TU Dortmund



## License

The *homing_local_planner* package is licensed under the BSD license. It depends on other ROS packages, which are listed in the package.xml. They are also BSD licensed.

Some third-party dependencies are included that are licensed under different terms:

- *Eigen*, MPL2 license, [http://eigen.tuxfamily.org](http://eigen.tuxfamily.org/)
- *Boost*, Boost Software License, [http://www.boost.org](http://www.boost.org/)

All packages included are distributed in the hope that they will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 