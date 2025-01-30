### **Using Plugin Mechanism in ROS 2 (pluginlib) to Extend Functionality**

#### **What is `pluginlib`?**

`pluginlib` is a plugin management framework in ROS 2 that allows developers to dynamically load and manage plugins without needing to bind to specific implementations at compile time. The plugin mechanism enables flexible functionality extension while keeping code modular.

------

### **1. Basics of `pluginlib`**

- **Plugin:** A shared library (`.so` file) that implements a specific interface.
- **Plugin Manager:** `pluginlib::ClassLoader` dynamically loads plugins that implement a specific interface.
- **Interface:** An abstract base class that plugins must inherit and implement.

This mechanism allows the main application to dynamically load plugins as needed, without hardcoding specific implementations.

------

### **2. Use Cases for `pluginlib`**

1. **Dynamic Functionality Extension:** For example, dynamically loading different motion control algorithms or path planners.
2. **Modular Design:** Separate functionality into independent modules for better maintainability.
3. **Runtime Flexibility:** Switch or load plugins at runtime without recompiling or restarting the program.

------



## 2.**ROS 2 Plugin Usage Example**

Below are examples to demonstrate **how to use built-in plugins in ROS 2** and **how to create and use custom plugins**. These examples showcase the practical application of ROS 2's plugin mechanism.

------

### **1. Using ROS 2 Built-in Plugins**

#### **Objective**

Load the `GridBased` path planner plugin provided by ROS 2 `nav2` and use it to generate a path.

#### **Steps**

##### **1.1 Configure the `nav2` Parameters**

Create a `nav2_params.yaml` file to configure and load the `GridBased` path planner plugin.

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: true
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

------

##### **1.2 Write the Test Code**

Use the `nav2_util` interface to call the path planning plugin.

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/simple_action_server.hpp"

class PathPlanningNode : public rclcpp::Node {
public:
    PathPlanningNode() : Node("path_planning_node") {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PathPlanningNode::planPath, this));
    }

private:
    void planPath() {
        // Define start and goal poses
        geometry_msgs::msg::PoseStamped start, goal;
        start.header.frame_id = "map";
        goal.header.frame_id = "map";

        start.pose.position.x = 0.0;
        start.pose.position.y = 0.0;

        goal.pose.position.x = 5.0;
        goal.pose.position.y = 5.0;

        // Example: Publish a dummy path
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.poses.push_back(start);
        path.poses.push_back(goal);

        path_pub_->publish(path);
        RCLCPP_INFO(this->get_logger(), "Published path!");
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlanningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

------

##### **1.3 Launch the Nodes**

1. Start `nav2` and load the `GridBased` plugin:

   ```bash
   ros2 launch nav2_bringup navigation_launch.py params_file:=/path/to/nav2_params.yaml
   ```

2. Run the path planning node:

   ```bash
   ros2 run your_package path_planning_node
   ```

------

### **2. Using Custom Plugins**

#### **Objective**

Create a simple custom plugin that dynamically logs a message at runtime.

------

#### **2.1 Define an Interface**

Define an abstract interface class that plugins need to implement.

```cpp
// include/my_plugin_base.hpp
#ifndef MY_PLUGIN_BASE_HPP
#define MY_PLUGIN_BASE_HPP

#include <string>

class MyPluginBase {
public:
    virtual ~MyPluginBase() = default;

    virtual void initialize(const std::string& name) = 0;
    virtual void execute() = 0;
};

#endif // MY_PLUGIN_BASE_HPP
```

------

#### **2.2 Implement the Plugin**

Create a plugin class that inherits and implements the interface.

```cpp
// src/my_plugin_one.cpp
#include "my_plugin_base.hpp"
#include <iostream>

class MyPluginOne : public MyPluginBase {
public:
    void initialize(const std::string& name) override {
        name_ = name;
        std::cout << "MyPluginOne initialized with name: " << name_ << std::endl;
    }

    void execute() override {
        std::cout << "MyPluginOne is executing!" << std::endl;
    }

private:
    std::string name_;
};

// Register the plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MyPluginOne, MyPluginBase)
```

------

#### **2.3 Create a Plugin Description File**

Define the plugin in a `plugin_description.xml` file.

```xml
<library path="my_plugin_one">
  <class name="MyPluginOne" type="MyPluginOne" base_class_type="MyPluginBase">
    <description>A custom plugin that logs messages.</description>
  </class>
</library>
```

------

#### **2.4 Write the Main Application**

Use `pluginlib::ClassLoader` to dynamically load and use the plugin.

```cpp
#include "pluginlib/class_loader.hpp"
#include "my_plugin_base.hpp"
#include <memory>
#include <iostream>

int main() {
    try {
        // Create a plugin loader
        pluginlib::ClassLoader<MyPluginBase> loader("my_package", "MyPluginBase");

        // Load the plugin
        auto plugin = loader.createSharedInstance("MyPluginOne");

        // Initialize and execute the plugin
        plugin->initialize("TestPlugin");
        plugin->execute();
    } catch (const pluginlib::PluginlibException& ex) {
        std::cerr << "Failed to load the plugin: " << ex.what() << std::endl;
    }

    return 0;
}
```

------

#### **2.5 Configure `CMakeLists.txt`**

Configure the plugin library and main application.

```cmake
cmake_minimum_required(VERSION 3.5)
project(my_plugin_example)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)

# Interface library
add_library(my_plugin_base INTERFACE)
target_include_directories(my_plugin_base INTERFACE include)

# Plugin library
add_library(my_plugin_one src/my_plugin_one.cpp)
ament_target_dependencies(my_plugin_one pluginlib)
target_link_libraries(my_plugin_one my_plugin_base)

pluginlib_export_plugin_description_file(my_plugin_base plugin_description.xml)

# Main application
add_executable(main_app src/main_app.cpp)
ament_target_dependencies(main_app pluginlib)
target_link_libraries(main_app my_plugin_base)

install(TARGETS my_plugin_base my_plugin_one main_app
        DESTINATION lib/${PROJECT_NAME})
install(FILES plugin_description.xml
        DESTINATION share/${PROJECT_NAME})
ament_package()
```

------

#### **2.6 Run the Program**

1. **Build the project:**

   ```bash
   colcon build
   ```

2. **Run the main application:**

   ```bash
   ros2 run my_plugin_example main_app
   ```

------

### **3. Example Outputs**

#### **Using Built-in Plugin**

```
Published path!
```

#### **Using Custom Plugin**

```
MyPluginOne initialized with name: TestPlugin
MyPluginOne is executing!
```

------

### **4. Summary**

#### **Built-in Plugin:**

- Example: `GridBased` plugin from `nav2`.
- Shows how to load and use plugins from the ROS 2 ecosystem for path planning or other tasks.

#### **Custom Plugin:**

- Demonstrates how to define, implement, and dynamically load your own plugins.
- Useful for modular and extensible systems.

By combining these two examples, you can fully understand how to utilize ROS 2's plugin mechanism to build flexible and scalable applications.
