# pc2image
This code converts a point cloud obtained by a Velodyne VLP16 3D-Lidar sensor into a depth image mono16.

<p float="left">
  <img src="/images/pc2image.GIF" width="770"  />
</p>

## Requisites
- [ROS](http://wiki.ros.org/ROS/Installation) Kinetic or Melodic
- [Velodyne](https://github.com/ros-drivers/velodyne) repository
  ```
  sudo apt-get install ros-melodic-velodyne-pointcloud
  ```
- [PCL](https://pointclouds.org/) (Point Cloud Library)

## Clone repository
```
    cd ~/catkin_ws/src
    git clone https://github.com/EPVelasco/pc2image.git
    cd ..
    catkin_make --only-pkg-with-deps pc2image
```
## Ros Launch
```
    roslaunch pc2image VLP16_image.launch
```
