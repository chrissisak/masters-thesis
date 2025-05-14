# Master's thesis in Applied Robotics

This repository contains the packages I developed for my master's thesis. The packages are an addition to [NASA's ISAAC project](https://github.com/nasa/isaac), and can also be found in a forked repository: [isaac/anomaly/anomaly-resolution](https://github.com/chrissisak/isaac/tree/develop/anomaly/anomaly-resolution). 




## About
### grasp_detection

The grasp detection package consists of two components: 

* **Edge detector** - processes depth data from Astrobee’s onboard HazCam (a flash LiDAR sensor) to identify object boundaries. It uses the Point Cloud Library (PCL) to detect significant depth discontinuities, highlighting edges and geometric features essential for grasping.
* **Grasp candidate detector** - analyzes these detected edges to find feasible grasp points within Astrobee's gripper width. It identifies and ranks grasp candidates based on proximity, generating coordinates and orientations that guide Astrobee's robotic arm for autonomous object capture and relocation.




### anomaly_resolution

The anomaly resolution package takes the grasp pose coordinates as input and uses them to steer Astrobee's gripper to the location. It consists of a sequence of motion commands for navigation and controlling the perching arm.




## Usage

The packages will not work on their own, and require the [installment of both Astrobee](https://nasa.github.io/astrobee/v/develop/md_INSTALL.html) and [ISAAC frameworks](https://nasa.github.io/isaac/html/md_INSTALL.html). Also, make sure you have added the [anomaly-resolution](https://github.com/chrissisak/isaac/tree/develop/anomaly/anomaly-resolution) packages.


Some file changes are needed to run the simulation smoothly. Check out the following files (from my forked repo):

**Necessary:**
* Coordinates and physics parameters: [description/urdf/sock.urdf.xacro](https://github.com/chrissisak/isaac/blob/develop/description/urdf/sock.urdf.xacro)

**Optional:**
* Enable HazCam debugger: [astrobee/config/simulation/simulation.config](https://github.com/chrissisak/astrobee/blob/develop/astrobee/config/simulation/simulation.config)
* Disable HeatCam (if it creates issues when running the simulator): [isaac/config/hw/heat_cam.config](https://github.com/chrissisak/isaac/blob/develop/isaac/config/hw/heat_cam.config)



The following command will launch the simulation environment in RViz.
```
roslaunch isaac sim.launch dds:=false robot:=sim_pub rviz:=true
```

Alternatively, both RViz and Gazebo can be launched using this command:

```
roslaunch isaac sim.launch dds:=false robot:=sim_pub rviz:=true gzclient:=true
```

The anomalous object, an astronaut sock, can be spawned in using the following command:

```
roslaunch isaac_gazebo spawn_object.launch spawn:=sock
```

Astrobee will spawn in on its docking station. To undock, execute the following command:

```
rosrun dock dock_tool -undock
```

Wait for the Astrobee to complete the undocking sequence, then run the following command to navigate it close to the astronaut sock:

```
rosrun mobility teleop -move -pos "11 -9.2 5.2" -att "1.57 0 0 1" -planner qp
```

Finally, run the anomaly resolution pipeline by executing this command:

```
roslaunch anomaly_resolution anomaly_resolution.launch
```







