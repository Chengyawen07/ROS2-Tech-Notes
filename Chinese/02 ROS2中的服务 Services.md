## 1.**什么是 ROS 2 中的服务（Service）？**

**服务（Service）** 是 ROS 2 中的一种通信机制，基于 **请求-响应模型（Request-Response Model）**。

- **客户端（Client）：** 发送请求给服务端，并等待响应。
- **服务端（Server）：** 处理客户端的请求并返回结果。

通俗来说，服务就像一个 **点菜系统**：

1. 客户端（顾客）发送请求（下订单）。
2. 服务端（餐厅）处理请求并返回响应（完成订单并送上菜品）。

------

### **服务与话题的区别**

| 特性           | 服务（Service）                  | 话题（Topic）                      |
| -------------- | -------------------------------- | ---------------------------------- |
| **通信模式**   | 请求-响应模型                    | 发布-订阅模型                      |
| **数据流向**   | 双向（请求 -> 响应）             | 单向（发布 -> 订阅）               |
| **适用场景**   | 一次性任务，如查询状态、控制命令 | 持续数据流，如传感器数据、状态广播 |
| **同步/异步**  | 通常是同步（等待响应后继续执行） | 通常是异步（无需等待响应）         |
| **实时性要求** | 适用于需要立即反馈的操作         | 适用于频繁、无状态的数据流传输     |

------

### **如何在 ROS 2 中实现服务和客户端？**

以下是一个 Python 示例，演示如何实现服务和客户端。

------

#### **服务端代码（Server）**

**功能：** 实现一个服务，接收两个整数并返回它们的和。

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # 使用标准服务接口

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')  # 节点名称
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)  # 创建服务
        self.get_logger().info('Service is ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b  # 计算请求参数的和
        self.get_logger().info(f'Received request: a={request.a}, b={request.b}, sum={response.sum}')
        return response  # 返回响应

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)  # 保持节点运行
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**运行步骤：**

1. 将代码保存为 `add_two_ints_server.py`。

2. 运行服务端：

   ```bash
   ros2 run <package_name> add_two_ints_server
   ```

------

#### **客户端代码（Client）**

**功能：** 创建一个客户端，发送两个整数到服务端并接收它们的和。

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # 使用标准服务接口

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')  # 节点名称
        self.client = self.create_client(AddTwoInts, 'add_two_ints')  # 创建客户端
        while not self.client.wait_for_service(timeout_sec=1.0):  # 等待服务可用
            self.get_logger().info('Waiting for service...')
        self.get_logger().info('Service is available.')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.client.call_async(request)  # 异步发送请求
        return self.future

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    a, b = 5, 10  # 要发送的两个整数
    future = node.send_request(a, b)

    rclpy.spin_until_future_complete(node, future)  # 等待响应
    if future.result() is not None:
        response = future.result()
        node.get_logger().info(f'Response: {response.sum}')
    else:
        node.get_logger().error('Service call failed.')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**运行步骤：**

1. 将代码保存为 `add_two_ints_client.py`。

2. 运行客户端：

   ```bash
   ros2 run <package_name> add_two_ints_client
   ```

------

#### **服务端和客户端的交互**

当服务端和客户端同时运行时，客户端会发送两个整数，服务端计算它们的和并返回给客户端。例如：

- **客户端发送请求：** `a=5, b=10`
- **服务端返回响应：** `sum=15`

客户端输出类似：

```
[INFO] [<timestamp>] [add_two_ints_client]: Response: 15
```

服务端输出类似：

```
[INFO] [<timestamp>] [add_two_ints_server]: Received request: a=5, b=10, sum=15
```

------



### ROS 2 中的实现方式

发布者节点：

- publisher = node.create_publisher(String, 'topic_name', 10)

订阅者节点：

- subscription = node.create_subscription(String, 'topic_name', callback_function, 10)

服务器节点：

- service = node.create_service(MyServiceType, 'service_name', handle_service_request) 

客户端节点：

- client = node.create_client(MyServiceType, 'service_name')

#### 适用场景：

- 话题的适用场景：

激光雷达、IMU、GPS等传感器数据的实时流。
机器人的状态发布和传输（如位置、速度、传感器状态等）。
机器人感知信息的持续更新（如环境地图、图像流等）。

- 服务的适用场景：

获取机器人当前位置或特定参数（如位置、传感器状态）。
请求机器人执行特定的任务（如启动、停止动作、调整电机速度）。
请求机器人状态或传感器配置（如获取电池电量、读取配置参数等）。

### **总结**

- **服务（Service）：** 适用于需要立即反馈的场景，如查询状态或发送控制命令。
- **话题（Topic）：** 更适合持续的数据流，如传感器数据或状态广播。
- **选择依据：** 根据任务的实时性和数据特性，选择服务或话题作为通信方式。







