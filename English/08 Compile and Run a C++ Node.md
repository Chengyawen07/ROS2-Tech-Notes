### **How to Compile and Run a Simple ROS 2 C++ Node**

Here’s a step-by-step guide to creating, compiling, and running a simple ROS 2 C++ node. This applies to any ROS 2 distribution (e.g., **Humble**).

------

### **Step 1: Create a Workspace**

1. Create a new workspace:

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

2. Initialize the workspace:

   ```bash
   colcon build
   source install/setup.bash
   ```

------

### **Step 2: Create a ROS 2 C++ Package**

1. Create a new ROS 2 package in the workspace:

   ```bash
   ros2 pkg create --build-type ament_cmake my_cpp_pkg
   ```

   - **`--build-type ament_cmake`**: Specifies the CMake build system.
   - **`my_cpp_pkg`**: The name of your package.

2. Verify the package directory structure:

   ```bash
   my_cpp_pkg/
   ├── CMakeLists.txt
   ├── package.xml
   └── src/
   ```

------

### **Step 3: Write the C++ Node**

1. Create a new C++ source file in the `src` directory, e.g., `simple_node.cpp`:

   ```bash
   touch src/simple_node.cpp
   ```

2. Edit `src/simple_node.cpp`:

   ```cpp
   #include "rclcpp/rclcpp.hpp"
   
   class SimpleNode : public rclcpp::Node
   {
   public:
       SimpleNode() : Node("simple_node")  // Define node name as "simple_node"
       {
           RCLCPP_INFO(this->get_logger(), "Hello, ROS 2! This is a C++ Node.");
       }
   };
   
   int main(int argc, char **argv)
   {
       rclcpp::init(argc, argv);  // Initialize ROS 2
       auto node = std::make_shared<SimpleNode>();
       rclcpp::spin(node);        // Run the node
       rclcpp::shutdown();        // Shutdown ROS 2
       return 0;
   }
   ```

------

### **Step 4: Configure CMakeLists.txt**

1. Open the `CMakeLists.txt` file and ensure the following is included:

   - **Add dependencies:**

     ```cmake
     find_package(ament_cmake REQUIRED)
     find_package(rclcpp REQUIRED)
     ```

   - **Add the executable:**

     ```cmake
     add_executable(simple_node src/simple_node.cpp)
     ament_target_dependencies(simple_node rclcpp)
     ```

   - **Install the executable:**

     ```cmake
     install(TARGETS simple_node
         DESTINATION lib/${PROJECT_NAME})
     ```

   - **Complete example:**

     ```cmake
     cmake_minimum_required(VERSION 3.5)
     project(my_cpp_pkg)
     
     find_package(ament_cmake REQUIRED)
     find_package(rclcpp REQUIRED)
     
     add_executable(simple_node src/simple_node.cpp)
     ament_target_dependencies(simple_node rclcpp)
     
     install(TARGETS simple_node
         DESTINATION lib/${PROJECT_NAME})
     
     ament_package()
     ```

2. Save and close the file.

------

### **Step 5: Build the Package**

1. Navigate back to the workspace root and build the package:

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_cpp_pkg
   ```

2. Once the build is complete, source the workspace:

   ```bash
   source install/setup.bash
   ```

------

### **Step 6: Run the Node**

1. Run the compiled node:

   ```bash
   ros2 run my_cpp_pkg simple_node
   ```

2. You should see the following output:

   ```
   [INFO] [<timestamp>] [simple_node]: Hello, ROS 2! This is a C++ Node.
   ```

------

### **Step 7: Debug and Optimize**

#### **Set Logging Level**

Change the logging level at runtime:

```bash
ros2 run my_cpp_pkg simple_node --ros-args --log-level debug
```

#### **Test the Node**

- List running nodes:

  ```bash
  ros2 node list
  ```

- Get detailed information about the node:

  ```bash
  ros2 node info /simple_node
  ```

------

### **Summary of the Workflow**

1. **Create Workspace:** `~/ros2_ws/src`
2. **Create Package:** `ros2 pkg create --build-type ament_cmake my_cpp_pkg`
3. **Write Node:** Implement a simple `simple_node.cpp` in C++.
4. **Configure CMakeLists.txt:** Add dependencies, executables, and installation rules.
5. **Build and Run:** Use `colcon build` to compile the package and `ros2 run` to execute the node.



