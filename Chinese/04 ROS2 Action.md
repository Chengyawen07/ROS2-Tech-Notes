## 1. **什么是 Action？**

**Action** 是 ROS 2 中的一种通信机制，专门用于处理 **长时间运行的任务**。

- 它基于 **目标-反馈-结果（Goal-Feedback-Result）** 模型，使客户端（Client）能够发送一个目标（Goal），并通过服务端（Server）接收任务的实时反馈和最终结果。
- **适用场景：** 比如机器人导航、机械臂的路径规划或复杂任务的执行。

------

### **Action 的工作流程**

1. **客户端发送目标（Goal）：** 客户端将任务目标发送给服务端。
2. **服务端处理任务：** 服务端开始执行任务，并周期性地发送实时反馈（Feedback）。
3. **结果返回（Result）：** 当任务完成时，服务端返回最终结果给客户端。

通俗来说，**Action 就像点外卖**：

- **发送目标（Goal）：** 客户端（顾客）下单（选择外卖）。
- **反馈（Feedback）：** 外卖员（服务端）提供实时进度（正在取餐、正在送餐）。
- **结果（Result）：** 外卖员最终完成配送，并确认订单完成。

------

### **Action 与 Topic、Service 的对比**

| 特性             | **Action**                                 | **Topic**                            | **Service**                            |
| ---------------- | ------------------------------------------ | ------------------------------------ | -------------------------------------- |
| **通信模型**     | **目标-反馈-结果（Goal-Feedback-Result）** | 发布-订阅（Publish-Subscribe）       | 请求-响应（Request-Response）          |
| **数据流向**     | 双向，多阶段：Goal -> Feedback -> Result   | 单向：发布者 -> 订阅者               | 双向：请求 -> 响应                     |
| **适用场景**     | **长时间运行任务（如导航、路径规划）**     | 持续数据流（如传感器数据、状态广播） | **一次性任务（如查询状态、控制命令）** |
| **实时反馈支持** | 支持（Feedback）                           | 不支持                               | 不支持                                 |
| **同步/异步**    | 异步                                       | 异步                                 | 通常是同步（等待响应后继续）           |

------

### **Action 的实现示例**

以下是一个简单的 Action 示例，演示如何创建服务端和客户端。

#### **Action 服务端（Server）**

**功能：** 接收目标并返回实时反馈和最终结果。

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci  # 使用标准 Action 接口

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self.action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()
        sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            sequence.append(sequence[i] + sequence[i - 1])
            feedback_msg.partial_sequence = sequence
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()  # 标记任务完成
        result = Fibonacci.Result()
        result.sequence = sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

------

#### **Action 客户端（Client）**

**功能：** 发送目标并接收实时反馈和最终结果。

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self.action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        self.get_logger().info('Sending goal...')
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.action_client.wait_for_server()  # 等待服务端可用
        self.future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback received: {feedback_msg.feedback.partial_sequence}')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result received: {result.sequence}')

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionClient()
    node.send_goal(order=10)  # 发送目标
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

------

### **Action 示例解释**

1. **服务端：**
   - 定义 `execute_callback`，用于处理客户端目标并发送反馈。
   - 通过 `publish_feedback` 发送实时反馈。
   - 返回最终结果 `Fibonacci.Result()`。
2. **客户端：**
   - 使用 `send_goal_async` 发送目标。
   - 通过 `feedback_callback` 接收实时反馈。
   - 最终通过 `result_callback` 获取结果。

------

### **Action 的特点**

- **支持实时反馈：** 能够在任务执行过程中提供进度信息。
- **异步操作：** 客户端不需要等待任务完成，可以同时处理其他操作。
- **多阶段通信：** 包含目标、反馈和结果三个阶段，适合复杂任务。

------

### **总结**

- **Action:** 适用于长时间运行的任务，支持实时反馈和最终结果。
- **Topic:** 用于持续的、无状态的数据流（如传感器数据）。
- **Service:** 用于一次性、同步的任务请求（如查询或命令）。
- 根据任务的需求选择合适的通信方式，例如导航使用 Action，而传感器数据适合 Topic。



## 2. **Navigation 如何使用 Action 通信？**

在 ROS 2（例如 **Navigation2**）中，导航任务广泛使用 **Action 通信** 来实现，例如导航到目标点、跟随路径或取消任务等。
 **Action** 提供了一种处理这些长时间任务的框架，支持 **实时反馈**、**目标管理** 和 **任务结果返回**。

------

### **ROS 2 Navigation 中的主要 Action**

1. **NavigateToPose** (`nav2_msgs/action/NavigateToPose`)
   - 用于导航机器人到指定位置和方向。
   - 包括：
     - **目标（Goal）：** 目标位置和方向（例如坐标和角度）。
     - **反馈（Feedback）：** 如距离目标的剩余距离等任务进展信息。
     - **结果（Result）：** 最终是否成功到达目标点。
2. **FollowPath** (`nav2_msgs/action/FollowPath`)
   - 指示机器人沿着预先计算的路径移动。
3. **Spin** (`nav2_msgs/action/Spin`)
   - 让机器人原地旋转一定角度。
4. **Wait** (`nav2_msgs/action/Wait`)
   - 暂停导航任务，等待指定时间。
5. **ComputePathToPose** (`nav2_msgs/action/ComputePathToPose`)
   - 计算到目标位置的路径，但不执行导航。

------

### **Navigation 中 Action 通信的工作流程**

1. **发送目标（Goal）**
    **NavigateToPose** 的客户端向服务端发送导航目标。
   - **目标：** 包括目标的坐标和方向（以地图坐标系为基准）。
2. **接收反馈（Feedback）**
    服务端定期向客户端发送 **实时反馈**，例如：
   - 距离目标点的剩余距离。
   - 当前机器人速度或位置。
3. **获取结果（Result）**
    任务完成（或被取消）后，服务端向客户端返回最终结果，说明：
   - 是否成功到达目标点。
   - 如果失败，失败的原因（例如障碍物、定位问题）。
4. **取消目标（Cancel Goal）**
    在任务执行中，客户端可以随时取消任务（例如用户更改目标点）。

------

### **示例：使用 Action 导航到目标点**

#### **Action 客户端代码：导航到目标点**

以下是一个使用 **NavigateToPose** 的客户端代码示例，演示如何发送目标并接收反馈和结果。

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, target_pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        self.action_client.wait_for_server()  # 等待导航服务端启动
        self.get_logger().info('发送导航目标...')
        self.future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'反馈：距离目标点剩余 {feedback.distance_remaining:.2f} 米')

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('成功到达目标点！')
        else:
            self.get_logger().error('导航失败，未能到达目标点。')

def main(args=None):
    rclpy.init(args=args)

    # 创建 NavigationClient 节点
    navigation_client = NavigationClient()

    # 定义目标点位姿（Pose）
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'map'  # 使用地图坐标系
    target_pose.pose.position.x = 2.0    # 目标位置 x 坐标
    target_pose.pose.position.y = 3.0    # 目标位置 y 坐标
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 1.0

    # 发送导航目标
    navigation_client.send_goal(target_pose)

    # 保持节点运行
    rclpy.spin(navigation_client)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

------

### **代码说明**

1. **初始化 Action 客户端：**
   - `ActionClient(self, NavigateToPose, 'navigate_to_pose')`：创建导航的 Action 客户端，用于与服务端通信。
2. **定义目标点：**
   - 使用 `PoseStamped` 定义目标点的位置（x, y 坐标）和方向（四元数表示）。
3. **发送目标：**
   - `send_goal_async` 异步发送目标，并注册反馈和结果的回调函数。
4. **接收反馈：**
   - `feedback_callback` 定期接收任务进展，例如距离目标点的剩余距离。
5. **处理结果：**
   - `result_callback` 获取任务最终结果，并判断是否成功到达目标点。

------

### **Action 与其他通信方式的对比**

| **特性**      | **Action（NavigateToPose）**                 | **Topic（例如 /cmd_vel）**   | **Service（例如 ComputePathToPose）** |
| ------------- | -------------------------------------------- | ---------------------------- | ------------------------------------- |
| **适用场景**  | **长时间运行的任务（如导航到目标点）**       | 实时控制（如机器人速度控制） | 一次性任务（如路径规划计算）          |
| **实时反馈**  | **支持（通过 Feedback）**                    | 不支持                       | 不支持                                |
| **结果返回**  | 支持                                         | 不支持                       | 支持                                  |
| **通信模型**  | **多阶段通信（Goal -> Feedback -> Result）** | 单向通信（发布者 -> 订阅者） | 双向通信（请求 -> 响应）              |
| **同步/异步** | 异步                                         | 异步                         | 通常是同步                            |

------

### **总结**

- ROS 2 Navigation 中使用 **Action** 通信实现复杂任务，如导航到目标点、路径规划等。
- Action 的优势：
  - 支持实时反馈，客户端可以随时查看任务进展。
  - 提供任务最终结果，便于判断任务是否成功。
  - 支持任务的中途取消，灵活处理任务变化。
- 选择通信方式：
  - 使用 **Action** 处理需要实时反馈和结果的长时间任务。
  - 使用 **Topic** 传递持续的实时控制数据（如速度命令）。
  - 使用 **Service** 处理简单的一次性任务（如路径计算）。

