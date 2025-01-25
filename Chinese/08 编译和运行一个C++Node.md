### **如何编译和运行一个简单的 ROS 2 C++ 节点**

以下是编译和运行一个简单的 ROS 2 C++ 节点的完整流程，适用于 ROS 2 的任意发行版本（例如 **Humble**）

------

### **步骤 1：创建工作空间**

1. 创建一个新的工作空间：

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

2. 初始化工作空间：

   ```bash
   colcon build
   source install/setup.bash
   ```

------

### **步骤 2：创建一个 ROS 2 C++ 包**

1. 在工作空间中创建一个新的 ROS 2 包：

   ```bash
   ros2 pkg create --build-type ament_cmake my_cpp_pkg
   ```

   - **`--build-type ament_cmake`**：指定使用 CMake 构建系统。
   - **`my_cpp_pkg`**：包的名称。

2. 检查包目录结构：

   ```bash
   my_cpp_pkg/
   ├── CMakeLists.txt
   ├── package.xml
   └── src/
   ```

------

### **步骤 3：编写 C++ 节点代码**

1. 在 `src` 目录中创建一个 C++ 源文件，例如 `simple_node.cpp`：

   ```bash
   touch src/simple_node.cpp
   ```

2. 编辑 `src/simple_node.cpp`：

   ```cpp
   #include "rclcpp/rclcpp.hpp"
   
   class SimpleNode : public rclcpp::Node
   {
   public:
       SimpleNode() : Node("simple_node")  // 定义节点名称为 "simple_node"
       {
           RCLCPP_INFO(this->get_logger(), "Hello, ROS 2! This is a C++ Node.");
       }
   };
   
   int main(int argc, char **argv)
   {
       rclcpp::init(argc, argv);  // 初始化 ROS 2
       auto node = std::make_shared<SimpleNode>(); // 实例化节点
       rclcpp::spin(node);        // 运行节点
       rclcpp::shutdown();        // 关闭 ROS 2
       return 0;
   }
   ```

------

### **步骤 4：配置 CMakeLists.txt**

1. 打开 `CMakeLists.txt`，确保包含以下内容：

   - **添加依赖：**

     ```cmake
     find_package(ament_cmake REQUIRED)
     find_package(rclcpp REQUIRED)
     ```

   - **添加可执行文件：**

     ```cmake
     add_executable(simple_node src/simple_node.cpp)
     ament_target_dependencies(simple_node rclcpp)
     ```

   - **安装可执行文件：**

     ```cmake
     install(TARGETS simple_node
         DESTINATION lib/${PROJECT_NAME})
     ```

   - 完整示例：

     ```cmake
     cmake_minimum_required(VERSION 3.5)
     project(my_cpp_pkg)
     
     # 找到依赖项
     find_package(ament_cmake REQUIRED)
     find_package(rclcpp REQUIRED)
     
     # 添加可执行文件
     add_executable(simple_node src/simple_node.cpp)
     # 链接库
     ament_target_dependencies(simple_node rclcpp)
     
     # 设置安装路径，让ros2 run能够找到路径
     install(TARGETS simple_node
         DESTINATION lib/${PROJECT_NAME})
     
     # 使包可用
     ament_package()
     ```

   这段配置做了以下几件事：

   - 找到 `rclcpp` 和 `std_msgs` 库。
   - 指定 `simple_node.cpp` 为可执行文件，并链接它的依赖库。
   - 设置安装路径，以便 `ros2 run` 能找到它。

2. 保存并关闭文件。

------

### **步骤 5：编译包**

1. 返回工作空间根目录并编译包：

   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_cpp_pkg
   ```

2. 确保编译成功后，加载环境：

   ```bash
   source install/setup.bash
   ```

------

### **步骤 6：运行节点**

1. 运行编译好的节点：

   ```bash
   ros2 run my_cpp_pkg simple_node
   ```

2. 你应该在终端看到输出：

   ```
   [INFO] [<timestamp>] [simple_node]: Hello, ROS 2! This is a C++ Node.
   ```

------

### **步骤 7：调试和优化**

#### **使用日志级别**

通过命令行更改日志级别：

```bash
ros2 run my_cpp_pkg simple_node --ros-args --log-level debug
```

#### **测试节点**

- 查看运行中的节点：

  ```bash
  ros2 node list
  ```

- 查看节点信息：

  ```bash
  ros2 node info /simple_node
  ```

------

### **完整流程总结**

1. **创建工作空间**：`~/ros2_ws/src`
2. **创建 ROS 2 包**：`ros2 pkg create --build-type ament_cmake my_cpp_pkg`
3. **编写 C++ 节点**：实现一个简单的 `simple_node.cpp`。
4. **配置 CMakeLists.txt**：确保正确添加依赖、目标和安装规则。
5. **编译并运行**：使用 `colcon build` 编译并通过 `ros2 run` 运行节点。

