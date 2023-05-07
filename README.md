# Rust is for Robotics

List of (awesome) Rust libraries for Robotics. If you know something awesome (or your project), please let me know from [here](https://github.com/robotics-rs/robotics.rs/pulls).

## ROS

*[ROS](http://www.ros.org/) related libraries.*

* [rosrust](https://github.com/adnanademovic/rosrust) - Pure Rust implementation of a ROS client library.
* [ros2_rust](https://github.com/ros2-rust/ros2_rust) -  Rust bindings for ROS2
* [roslibrust](https://github.com/Carter12s/roslibrust) - Pure Rust implementation of a [rosbridge](https://github.com/RobotWebTools/rosbridge_suite) client.
* [r2r](https://github.com/sequenceplanner/r2r) - Minimal ROS2 Rust bindings
* [rclrust](https://github.com/rclrust/rclrust) - Yet another ROS2 Rust client
* [RustDDS](https://github.com/jhelovuo/RustDDS) - Rust implementation of Data Distribution Service
* [rosbag](https://github.com/SkoltechRobotics/rosbag-rs) - Reading rosbag files in pure Rust
* [rustros_tf](https://github.com/arjo129/rustros_tf) - A rust implementation of the Tf library.
* [ros_pointcloud2](https://github.com/stelzo/ros_pointcloud2) - The safe way of using PointCloud2 messages in ROS1 and ROS2.
* [optimization-engine](https://alphaville.github.io/optimization-engine/) - Fast & Accurate Embedded Optimization for next-generation Robotics and Autonomous Systems

## AI

* [bonsai-bt](https://github.com/Sollimann/bonsai) - A Behavior Tree implementation in Rust

## Framework

* [OpenRR](https://github.com/openrr/openrr) - Open Rust Robotics

* [Zenoh](https://zenoh.io) - A high performance and extremely low overhead Pub/Sub/Query protocol. Quickly becoming the protocol of choice for Robot-to-Anything communication. 

## gRPC

*[gRPC](https://grpc.io/) A high performance, open source universal RPC framework.*

* [CleanIt](https://github.com/Sollimann/CleanIt) - Open-source Autonomy Software in Rust-lang with gRPC for the Roomba series robot vacuum cleaners.

## Math and Geometry

*Math related libraries for Robotics.*

* [nalgebra](https://github.com/sebcrozet/nalgebra) - Linear algebra library for Rust.
* [ncollide](https://github.com/sebcrozet/ncollide) - 2 and 3-dimensional collision detection library in Rust.
* [kdtree](https://github.com/mrhooray/kdtree-rs) - K-dimensional tree in Rust for fast geospatial indexing.
* [k](https://github.com/OTL/k) -  k: Kinematics Library for rust-lang.
* [static-math](https://github.com/elsuizo/static-math) - Safe and fast mathematical operations with static arrays in Rust programming language thinked for robotics
* [ndarray](https://github.com/rust-ndarray/ndarray) - N-dimensional tensor arithmetic library, inspired by python's NumPy.

## Path Planning

*Path planning libraries.*

* [pathfinding](https://github.com/samueltardieu/pathfinding) - Pathfinding library for rust
* [rrt](https://github.com/openrr/rrt) - RRT (Rapidly-exploring Random Tree) library in Rust
* [openrr-planner](https://github.com/openrr/openrr) - Collision Avoidance Path Planning in Rust-lang

## Simulation

*Physics simulation for robots*

* [rapier](https://github.com/dimforge/rapier) - 2 and 3-dimensional rigid body physics engine for Rust.

## Visualization

*Graphic and visualization for Robotics.*

* [kiss3d](https://github.com/sebcrozet/kiss3d) - Keep it simple, stupid 3d graphics engine for Rust.
* [urdf-viz](https://github.com/openrr/urdf-viz) - URDF visualizer
* [rerun](https://github.com/rerun-io/rerun) — A logging SDK and visualizer for computer vision and robotics

## File Loading

*Import/Export various files related with Robotics*

* [assimp-rs (open-asset-importer)](https://github.com/Vurich/assimp-rs) - Rust bindings for the [Assimp](http://www.assimp.org/) library.
* [urdf-rs](https://github.com/OTL/urdf-rs) - URDF Loader for Rust
* [pcd-ros](https://github.com/jerry73204/pcd-rs) - Read point cloud data from PCD file format

## Device Driver

*Robotics releated sensor/motor drivers*

* [libsweep](https://github.com/andygrove/libsweep-rs) - Rust wrapper for Scanse Sweep LIDAR libsweep
* [freenect-rs](https://github.com/Entscheider/freenect-rs) - Freenect wrapper for rust
* [rplidar-rs](https://github.com/cnwzhjs/rplidar.rs) - Slamtec RPLIDAR public SDK for Rust
* [dynpick-force-torque-sensor](https://github.com/Amelia10007/dynpick-force-torque-sensor-rs) - Wacoh-tech 6-axis force sensor (Dynpick) driver
* [leptrino-force-torque-sensor](https://github.com/Amelia10007/leptrino-force-torque-sensor-rs) - Leptrino 6-axis force sensor driver
* [realsense-rust](https://github.com/Tangram-Vision/realsense-rust) - RealSense Bindings for Rust
