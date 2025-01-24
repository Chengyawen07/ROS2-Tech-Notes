

## 1**什么是ROS 2？**
ROS 2 是一个开源的机器人开发框架，设计用于支持分布式系统、实时性能和多语言开发。它提供模块化工具链，方便开发者快速构建和部署机器人应用。

**基本架构及作用**

1. **节点（Nodes）**
   - 功能：ROS 2 中的最小计算单元，每个节点通常负责一个特定功能（如控制传感器、处理数据等）。
   - 作用：通过模块化设计使系统更易维护和扩展。多个节点可以并行运行，实现分布式计算。
2. **主题（Topics）**
   - 功能：节点之间用于传递无状态数据的通信通道，基于发布-订阅模式。
   - 作用：用于传递传感器数据、控制指令等。例如，摄像头节点将图像数据发布到一个主题，其他节点可订阅该主题以获取数据。
3. **服务（Services）**
   - 功能：实现节点之间的请求-响应通信，适用于短期任务。
   - 作用：当需要同步操作时使用，例如机器人移动时请求某个目标的坐标。
4. **动作（Actions）**
   - 功能：实现长时间运行任务的请求-反馈机制。
   - 作用：例如导航任务，可以在执行过程中实时反馈进度或提前取消。
5. **DDS（数据分发服务）**
   - 功能：底层通信中间件，提供可靠的发布-订阅功能。
   - 作用：支持实时性能、分布式计算和跨平台通信。
6. **参数服务器（Parameters）**
   - 功能：存储和管理节点运行时需要的配置参数。
   - 作用：可以动态调整参数而无需重启节点，例如调整机器人的速度。
7. **生命周期管理（Lifecycle Management）**
   - 功能：管理节点的状态（如未激活、激活、关闭等）。
   - 作用：提高系统的可控性和稳定性，特别适合复杂应用。
8. **工具链（Toolchain）**
   - 功能：包括rviz（可视化工具）、rosbag（数据记录工具）、colcon（构建工具）等。
   - 作用：简化开发、调试和运行过程。



ROS 2 的基本组件总结
节点（Node）：机器人系统的基本执行单元。
话题（Topic）：发布/订阅模型，传输数据。
服务（Service）：请求/响应模型，处理控制和查询。
动作（Action）：长时间任务和反馈机制。
DDS（中间件）：底层通信框架，提供实时数据分发。
参数服务器：节点的可配置参数管理。
生命周期管理：节点的启动、暂停、停止控制。
跨平台支持：支持多个操作系统。
安全性：提供数据加密、身份验证等安全机制。



## 2. **什么是节点（Node）？**

节点是 ROS 2 中的基本执行单元，负责处理特定的任务，例如传感器数据的获取、数据处理或机器人控制。多个节点可以通过 ROS 2 的通信机制（如话题、服务、动作等）协同工作，实现分布式计算。

**如何在 ROS 2 中创建一个简单的节点？**

以下是基于 **ROS 2 Humble** 的节点创建方法，分别用 **Python** 和 **C++** 编写。

------

### **Python 示例**

1. **依赖安装**
    确保 `rclpy` 已安装，且 ROS 2 环境已正确配置。

2. 首先创建一个workspace

    ```python
    # 创建一个工作区
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```

    

3. **创建节点代码**

   ```python
   import rclpy
   from rclpy.node import Node
   
   class SimpleNode(Node):
       def __init__(self):
           super().__init__('simple_node')
           self.get_logger().info('Simple Node is running in ROS 2 Humble.')
   
   def main(args=None):
       rclpy.init(args=args)
       node = SimpleNode()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()
   
   if __name__ == '__main__':
       main()
   ```

4. **运行节点**

   - 创建一个 Python 包（如果尚未创建）：

     ```bash
     ros2 pkg create --build-type ament_python my_python_pkg
     ```

   - 将上述代码保存为 `simple_node.py` 并放入包的 `my_python_pkg/my_python_pkg` 目录。

   - 编辑 

     ```
     setup.py
     ```

     ，确保 

     ```
     entry_points
     ```

      中注册节点：

     ```python
     entry_points={
         'console_scripts': [
             'simple_node = my_python_pkg.simple_node:main',
         ],
     },
     ```

   - 构建包：

     ```bash
     colcon build --packages-select my_python_pkg
     ```

   - 运行节点：

     ```bash
     ros2 run my_python_pkg simple_node
     ```

------

### **C++ 示例**

1. **依赖安装**
    确保 `rclcpp` 和 `std_msgs` 已安装，并正确配置 ROS 2 环境。

2. **创建节点代码**

   ```cpp
   #include "rclcpp/rclcpp.hpp"
   
   class SimpleNode : public rclcpp::Node {
   public:
       SimpleNode() : Node("simple_node") {
           RCLCPP_INFO(this->get_logger(), "Simple Node is running in ROS 2 Humble.");
       }
   };
   
   int main(int argc, char **argv) {
       rclcpp::init(argc, argv);
       auto node = std::make_shared<SimpleNode>();
       rclcpp::spin(node);
       rclcpp::shutdown();
       return 0;
   }
   ```

3. **运行节点**

   - 创建一个 C++ 包（如果尚未创建）：

     ```bash
     ros2 pkg create --build-type ament_cmake my_cpp_pkg
     ```

   - 将上述代码保存为 `simple_node.cpp`，放入 `my_cpp_pkg/src` 目录。

   - 编辑 

     ```
     CMakeLists.txt
     ```

     ，添加以下内容：

     ```cmake
     add_executable(simple_node src/simple_node.cpp)
     ament_target_dependencies(simple_node rclcpp)
     install(TARGETS simple_node DESTINATION lib/${PROJECT_NAME})
     ```

   - 构建包：

     ```bash
     colcon build --packages-select my_cpp_pkg
     ```

   - 运行节点：

     ```bash
     ros2 run my_cpp_pkg simple_node
     ```

------

### 注意事项

- **Python 包**：记得在 `setup.py` 中正确注册脚本，并在 `package.xml` 中添加依赖，例如 `rclpy`。
- **C++ 包**：确保在 `CMakeLists.txt` 和 `package.xml` 中声明所需依赖，例如 `rclcpp` 和其他用到的库。





## 3. **什么是话题（Topic）？**

**话题**是 ROS 2 中节点之间交换数据的一种通信机制，基于 **发布-订阅（Publish-Subscribe）模型**。

- **发布者（Publisher）：** 把数据发送到一个特定的“话题”。
- **订阅者（Subscriber）：** 监听特定的话题并接收数据。

通俗地说，**话题就像一个广播电台**：

1. **电台（发布者）：** 播放一段音乐（数据）。
2. **收音机（订阅者）：** 调到这个电台，接收并播放音乐。

在 ROS 2 中，多个节点可以通过话题协同工作，例如：

- **摄像头节点：** 发布图像数据到 `/camera_image` 话题。
- **处理节点：** 订阅 `/camera_image` 话题，对图像进行处理。

------

### **如何在 ROS 2 中发布和订阅话题？**

以下是 Python 实现发布和订阅话题的完整代码示例。

------

#### **发布者代码（Publisher）**

**功能：** 每隔 0.5 秒发布一条消息到话题 `/chatter`。

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # 消息类型

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')  # 节点名称
        self.publisher_ = self.create_publisher(String, 'chatter', 10)  # 创建发布者
        self.timer = self.create_timer(0.5, self.timer_callback)  # 定时器，0.5秒触发一次
        self.get_logger().info('Publisher is running!')  # 打印日志信息

    def timer_callback(self):
        msg = String()  # 创建消息对象
        msg.data = 'Hello, ROS 2!'  # 设置消息内容
        self.publisher_.publish(msg)  # 发布消息
        self.get_logger().info(f'Publishing: "{msg.data}"')  # 打印发布内容到控制台

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)  # 保持节点运行
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**运行步骤：**

1. 将代码保存为 `publisher.py`。

2. 启动发布者：

   ```bash
   ros2 run <package_name> publisher
   ```

------

#### **订阅者代码（Subscriber）**

**功能：** 订阅话题 `/chatter` 并打印接收到的消息。

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # 消息类型

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')  # 节点名称
        self.subscription = self.create_subscription(
            String, 'chatter', self.listener_callback, 10)  # 创建订阅者
        self.subscription  # 防止订阅者被垃圾回收
        self.get_logger().info('Subscriber is running!')  # 打印日志信息

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')  # 打印接收到的消息

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)  # 保持节点运行
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**运行步骤：**

1. 将代码保存为 `subscriber.py`。

2. 启动订阅者：

   ```bash
   ros2 run <package_name> subscriber
   ```

------

#### **同时运行发布者和订阅者：**

- 启动发布者：

  ```bash
  ros2 run <package_name> publisher
  ```

- 启动订阅者：

  ```bash
  ros2 run <package_name> subscriber
  ```

在订阅者的终端中，你会看到类似如下的输出：

```
[INFO] [<时间戳>] [simple_subscriber]: Received: "Hello, ROS 2!"
```

------

### **代码讲解**

1. **消息类型 (`std_msgs.msg.String`)：**
    ROS 2 使用标准消息类型（如 `String`）来定义发布和订阅的数据结构。
2. **发布者核心逻辑：**
   - `create_publisher`: 创建发布者。
   - `create_timer`: 定时触发回调函数 `timer_callback`。
   - `publish`: 将消息发布到指定话题。
3. **订阅者核心逻辑：**
   - `create_subscription`: 创建订阅者。
   - 回调函数 `listener_callback`: 每当收到消息时自动触发。

------

### **总结**

- **话题** 是 ROS 2 中节点通信的重要机制，适用于实时数据流的场景（如传感器数据、机器人状态）。
- 发布者负责发送消息，订阅者负责接收消息。
- 使用 ROS 2 的标准接口，开发者可以快速实现模块化的机器人系统。







