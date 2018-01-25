The terrain-server
==============================================


## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Introduction
The terrain server computes a risk associate to the terrain (i.e. terrain costmap) and its surface; information important for motion planning and control in legged locomotion. The terrain costmap quantifies how desirable it is to place a foot at a specific location. The cost value for each voxel in the map is computed using geometric terrain features such as height  deviation, slope and curvature. It computes the slope and curvature through regression in a configurable window around the cell in question; the features are computed from a Octomap map. For instance,  the estimated surface normals and curvatures are computed from a set of neighboring occupied voxels. It terrain servers is used in the different motion planning and control methods, see publication section.

The dwl-rviz-plugin contains a plugin for visualization of the terrain costmap and normals, i.e. dwl_rviz_plugin::TerrainMapDisplay. Here you can see an example of mapping and visualization

| [![](https://i.imgur.com/RKe3sNo.gif)](https://www.youtube.com/watch?v=KI9x1GZWRwE) | [![](https://i.imgur.com/4kKhryj.gif)](https://www.youtube.com/watch?v=KI9x1GZWRwE)


The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Carlos Mastalli, carlos.mastalli@laas.fr<br />
With support from the Dynamic Legged Systems lab at Istituto Italiano di Tecnologia<br />**



## <img align="center" height="20" src="https://i.imgur.com/fjS3xIe.png"/> Dependencies

The algorithms are built primarily in C/C++. The library uses a number of the local dependencies, which some of them are optionals.

The dwl-rviz-pluin is a ROS packages with the following required dependencies:
* [dwl](https://github.com/robot-locomotion/dwl)
* [dwl-msgs](https://github.com/robot-locomotion/dwl-msgs)
* [Octomap](http://octomap.github.io) (version 1.6.8 or higher)



## <img align="center" height="20" src="https://i.imgur.com/x1morBF.png"/> Building

Before building the dwl_msgs you need to install the dependencies of DWL. Additionally you have to build dwl with catkin.

The terrain_server is a catkin project which can be built as:

	cd your_ros_ws/
	catkin_make




## <img align="center" height="20" src="http://www.pvhc.net/img205/oohmbjfzlxapxqbpkawx.png"/> Publications

* C. Mastalli, I. Havoutis, M. Focchi, D. G. Caldwell, C. Semini, [Motion planning for challenging locomotion: a study of decoupled and coupled approaches](https://hal.archives-ouvertes.fr/hal-01649836v1), IEEE International Conference on Robotics and Automation (ICRA), 2017
* B. Aceituno-Cabezas, C. Mastalli, H. Dai, M. Focchi, A. Radulescu, D. G. Calwell, J. Cappelletto, J. C. Griego, G. Fernardez, C. Semini, [Simultaneous Contact, Gait and Motion Planning for Robust Multi-Legged Locomotion via MICP](http://ieeexplore.ieee.org/document/8141917/), IEEE Robotics and Automation Letters  (RAL), 2017
* C. Mastalli, M. Focchi, I. Havoutis, A. Radulescu, D. G. Caldwell, C. Semini, [Trajectory and Foothold Optimization using Low-Dimensional Models for Rough Terrain Locomotion](https://old.iit.it/images/stories/advanced-robotics/hyq_files/publications/mastalli17icra.pdf), IEEE International Conference on Robotics and Automation (ICRA), 2017
* C. Mastalli, A. Winkler, I. Havoutis, D. G. Caldwell, C. Semini, [On-line and On-board Planning and Perception for Quadrupedal Locomotion](http://iit.it/images/stories/advanced-robotics/hyq_files/publications/mastalli15tepra.pdf), IEEE International Conference on Technologies for Practical Robot Applications (TEPRA), 2015
* A. Winkler, C. Mastalli, I. Havoutis, M. Focchi, D. G. Caldwell, C. Semini, [Planning and Execution of Dynamic Whole-Body Locomotion for a Hydraulic Quadruped on Challenging Terrain](http://iit.it/images/stories/advanced-robotics/hyq_files/publications/winkler15icra.pdf), IEEE International Conference on Robotics and Automation (ICRA), 2015

