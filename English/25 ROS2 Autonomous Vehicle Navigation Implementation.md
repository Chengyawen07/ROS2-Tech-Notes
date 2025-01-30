### **ROS 2 Project: Autonomous Vehicle Navigation Implementation**

#### **Project Background**

The goal of this project is to implement an autonomous navigation system on the ROS 2 platform, enabling a self-driving vehicle to plan a path and safely reach a target position within a known map. The system integrates various ROS 2 components, such as SLAM (Simultaneous Localization and Mapping), path planning, sensor fusion, and dynamic obstacle avoidance.

------

### **System Architecture**

The autonomous navigation system consists of the following key modules:

1. **SLAM Module:**
   - Generates an environment map using `slam_toolbox` or `cartographer`.
   - Provides real-time localization of the vehicle using LiDAR and odometry data.
2. **Path Planning Module:**
   - Utilizes the ROS 2 Navigation 2 (`nav2`) stack for path planning.
   - Global planner: Uses costmap-based algorithms like A* or Dijkstra for path generation.
   - Local planner: Uses the Dynamic Window Approach (DWB) controller for obstacle avoidance.
3. **Sensor Fusion Module:**
   - Employs the `robot_localization` package to fuse IMU, LiDAR, and odometry data for accurate vehicle localization.
4. **Control Module:**
   - Implements velocity control to manage the left and right wheel speeds of the differential-drive vehicle.
5. **Data Visualization:**
   - Monitors vehicle position, path planning, LiDAR data, and environment mapping in RViz.

------

### **Hardware Configuration**

1. **Robot Platform:** A differential-drive robot with four wheels.
2. Sensors:
   - **2D LiDAR (RPLIDAR A2):** For obstacle detection and mapping.
   - **IMU (MPU6050):** Provides angular velocity and acceleration data.
   - **Odometry (Encoders):** Measures left and right wheel speeds.
3. Computing Platform:
   - NVIDIA Jetson or an x86 industrial PC running ROS 2 (Humble).

------

### **Implementation Steps**

#### **1. Map Generation and Localization**

1. **Start SLAM** Use `slam_toolbox` to generate the environment map and enable real-time localization:

   ```bash
   ros2 launch slam_toolbox online_async_launch.py
   ```

   - LiDAR data is published on the `/scan` topic.
   - The generated map is published on the `/map` topic.

2. **Save the Map** Save the map after completing the mapping process:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
   ```

------

#### **2. Sensor Fusion**

1. **Configure `robot_localization` for Data Fusion** Use an EKF (Extended Kalman Filter) to fuse IMU, odometry, and LiDAR data, producing accurate position data on `/odom`:

   ```yaml
   ekf_filter_node:
     ros__parameters:
       frequency: 30
       sensor_timeout: 0.1
       two_d_mode: true
       odom_frame: odom
       base_link_frame: base_link
       world_frame: map
       odom0: /odom
       odom0_config: [true, true, false, false, false, true, true, true, false, false, false, false, false, false, false]
       imu0: /imu
       imu0_config: [false, false, false, true, true, true, true, true, true, false, false, false, false, false, false]
   ```

2. **Launch the Sensor Fusion Node**

   ```bash
   ros2 launch robot_localization ekf.launch.py
   ```

------

#### **3. Navigation Features**

1. **Configure `Navigation 2`** Edit the `nav2_params.yaml` file to set up the global planner, local planner, and costmaps:

   ```yaml
   planner_server:
     ros__parameters:
       planner_plugin_ids: ["GridBased"]
       use_sim_time: true
   controller_server:
     ros__parameters:
       controller_plugin_ids: ["FollowPath"]
       FollowPath:
         plugin: "dwb_core::DWBLocalPlanner"
         goal_checkers: ["goal_checker"]
         vx_samples: 20
         vy_samples: 0
   ```

2. **Launch Navigation**

   ```bash
   ros2 launch nav2_bringup navigation_launch.py map:=/path/to/my_map.yaml
   ```

3. **Send Navigation Goals** Use the "2D Goal Pose" tool in RViz to send a target goal.

------

#### **4. Dynamic Obstacle Avoidance**

1. Local Path Planning

   The DWB controller dynamically adjusts the vehicle's velocity to avoid obstacles:

   - Input: LiDAR data from `/scan` and real-time position from `/odom`.
   - Output: Smooth velocity commands to `/cmd_vel`.

------

### **Key Implementation Details**

#### **1. ROS 2 Node Design**

Each functional module is encapsulated in a ROS 2 node. Smart pointers are used to manage their lifecycle. Example:

```cpp
#include <rclcpp/rclcpp.hpp>

class NavigationNode : public rclcpp::Node {
public:
    NavigationNode() : Node("navigation_node") {
        RCLCPP_INFO(this->get_logger(), "Navigation Node started!");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

#### **2. Controller Implementation**

The `DiffDriveController` handles base velocity control:

- Sends velocity commands (

  ```
  /cmd_vel
  ```

  ) to control the left and right wheels:

  ```bash
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"
  ```

#### **3. Data Visualization**

In RViz, the following elements are visualized:

- Map (`/map`)
- LiDAR data (`/scan`)
- Vehicle position (`/odom`)
- Path planning (`/plan`)

------

### **Results**

1. Path Planning and Tracking:
   - The vehicle successfully generates a global path to the target and follows it.
2. Dynamic Obstacle Avoidance:
   - The local planner dynamically adjusts the path to avoid moving obstacles.
3. Accurate Localization:
   - By fusing IMU and odometry data, the vehicle maintains precise localization in dynamic environments.

------

### **Challenges and Solutions**

1. **Challenges:**
   - **Sensor Noise:** LiDAR and IMU data often contain noise.
   - **Obstacle Avoidance:** In complex environments, the robot may struggle to avoid dynamic obstacles.
2. **Solutions:**
   - **Data Filtering:** Use EKF filters to reduce the impact of sensor noise.
   - **Tuning Local Planner:** Adjust DWB controller parameters for better performance in local obstacle avoidance.

------

### **Summary**

This project implemented autonomous navigation for a differential-drive vehicle using ROS 2. Key functionalities include map generation, path planning, dynamic obstacle avoidance, and sensor fusion. With modular design and parameter optimization, the vehicle navigates safely and efficiently in complex environments.
