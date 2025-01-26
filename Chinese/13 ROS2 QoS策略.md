

## 1.**什么是 QoS？（通俗解释）**⭐️

**QoS（Quality of Service）可以简单理解为通信的“规则”或“约定”，它控制 ROS 2 节点之间如何发送和接收消息。**你可以用 QoS 来“定制”消息的传递方式，确保它们符合你的实际需求。

------

### **为什么需要 QoS？（用途）**

<u>**不同的机器人应用对消息传递的要求可能差别很大，比如：**</u>

- **实时性**：某些任务需要快速、及时地接收消息（比如机器人避障）。
- **可靠性**：某些任务则需要确保每一条消息都被传递（比如机器人控制命令）。
- **资源优化**：某些任务需要减少内存占用或带宽（比如视频流数据）。

QoS 的作用是让你可以根据应用需求，灵活调整消息的传递行为，从而提高系统的性能和稳定性。

------

### **一个简单类比**

把 ROS 2 中的通信看作是邮寄包裹，而 QoS 就是控制邮寄方式的规则。例如：

1. **Reliability（可靠性）**
   - **快递（RELIABLE）：** 每个包裹都必须安全送到，丢失了会重发，适合重要文件。
   - **普通邮寄（BEST_EFFORT）：** 只尽量送到，丢了就算了，适合普通广告信。
2. **Durability（持久性）**
   - **“即时配送”（VOLATILE）：** 新加入的订阅者只会收到从现在开始的新包裹，之前的内容无法获取。
   - **“留件服务”（TRANSIENT_LOCAL）：** 快递员会保存最近的包裹，如果新订阅者加入，他们也能收到这些之前保存的包裹。
3. **History（历史）**
   - **只存最近的包裹（KEEP_LAST）：** 例如，只存最近 10 个包裹，之前的被覆盖。
   - **存所有包裹（KEEP_ALL）：** 不管多久前的包裹都存起来，但需要很大的存储空间。
4. **Depth（队列深度）**
   - 控制快递堆积的最大数量，比如同时只允许最多 5 个包裹等待配送。
5. **Lifespan（生存时间）**
   - 包裹过期了就丢掉，比如鲜花只送当天的，不送几天前的旧货。
6. **Deadline（时限）**
   - 包裹必须在一定时间内送达，否则视为失败。
7. **Liveliness（存活性）**
   - 检查快递员是否还在工作。如果他停止发货，系统会发现问题并报警。

------

### **用途和示例**

#### **1. 确保关键数据可靠传输**

- 场景：机器人控制指令，例如移动到某个位置。
- QoS 配置：
  - **Reliability：RELIABLE** （确保每条命令都能到达）。
  - **Durability：VOLATILE** （不需要历史指令）。

#### **2. 处理实时数据流**

- 场景：摄像头实时视频流。
- QoS 配置：
  - **Reliability：BEST_EFFORT** （允许丢帧，降低延迟）。
  - **History：KEEP_LAST** （只保留最近几帧）。

#### **3. 给新加入的节点发送状态**

- 场景：机器人启动后，需要让新连接的节点知道当前状态。
- QoS 配置：
  - **Durability：TRANSIENT_LOCAL** （新加入的节点可以接收到最近一次状态消息）。

#### **4. 节省资源**

- 场景：激光雷达数据发布，传感器发布速度高但处理速度有限。
- QoS 配置：
  - **Depth：10** （限制队列深度，防止内存占用过多）。
  - **Reliability：BEST_EFFORT** （只接收尽可能多的数据，不需要重发）。

------

### **总结**

**通俗来说，QoS 就是给消息传递设定一套“规则”，让你可以：**

1. 决定消息“能不能丢”、“丢了要不要重发”。
2. 决定新加入的节点“能不能收到历史消息”。
3. 控制消息存储的方式和数量。
4. 设置消息的时限和过期策略。

它的用途是为了适应不同场景的需求，比如实时性、可靠性或资源优化。在实际应用中，你可以根据场景灵活选择合适的 QoS 配置，让系统更加稳定和高效。





## 2. QoS (Quality of Service) 定义

**<u>QoS（Quality of Service） 是 ROS 2 中的一组策略，用于控制节点之间通信的行为和性能。</u>**

QoS 决定了消息在 **Publisher（发布者）** 和 **Subscriber（订阅者）** 之间的传递方式，尤其是在网络不稳定或带宽有限的情况下。

QoS 的作用：

1. **提高通信的可靠性：** 确保消息按预期被接收或处理。
2. **优化性能：** 在实时性和可靠性之间找到平衡。
3. **适应多种应用需求：** 从实时控制到延迟容忍的任务。

------

### **ROS 2 中的 QoS 策略**

ROS 2 的 QoS 策略基于 DDS（Data Distribution Service）标准，提供了多种配置选项以满足不同场景需求。以下是 ROS 2 中的主要 QoS 策略：

------

#### **1. Reliability (可靠性)**

控制消息传递的可靠性。

- **RELIABLE:**
   每条消息都必须被成功传递。如果接收方未接收到消息，发布方会重试，适用于高可靠性需求的场景，例如机器人控制命令。
- **BEST_EFFORT:**
   尽力而为地传递消息，不保证消息一定到达接收方，适用于低延迟和网络不稳定的场景，例如相机图像流。

**示例：**

```python
from rclpy.qos import ReliabilityPolicy, QoSProfile

qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE)
```

------

#### **2. Durability (持久性)**

<u>控制新加入的订阅者是否可以收到发布方的历史消息。</u>

- **VOLATILE:**
   订阅者只接收从订阅开始时发布的消息，之前的消息不会被缓存。
- **TRANSIENT_LOCAL:**
   发布者会缓存最近的消息，新订阅者可以收到这些历史消息。例如，用于保存最近的状态信息。

**示例：**

```python
from rclpy.qos import DurabilityPolicy, QoSProfile

qos = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL)
```

------

#### **3. History (历史)**

控制消息的存储方式以及存储的消息数量。

- **KEEP_LAST:**
   只保留最近的 `N` 条消息（默认值为 10），适用于数据量大的场景。
- **KEEP_ALL:**
   保留所有消息，适用于需要完整数据记录的场景，但可能占用大量内存。

**示例：**

```python
from rclpy.qos import HistoryPolicy, QoSProfile

qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5)
```

- **`depth`:** 设置 `KEEP_LAST` 策略下的消息缓存深度。

------

#### **4. Depth (深度)**

定义消息队列的大小，即发布方最多可以存储多少未处理的消息。

- 适用场景：
  - 如果消息发布速度快于订阅速度，增加队列深度可以避免消息丢失。

**示例：**

```python
qos = QoSProfile(depth=10)
```

------

#### **5. Liveliness (生存性)**

控制节点的活动性检查机制，用于检测发布者是否仍然活跃。

- **AUTOMATIC:**
   系统自动管理生存性，通常适用于大多数场景。
- **MANUAL_BY_TOPIC:**
   发布者需要主动发送心跳信号，订阅者通过心跳信号检测发布者是否存活。

**示例：**

```python
from rclpy.qos import LivelinessPolicy, QoSProfile

qos = QoSProfile(liveliness=LivelinessPolicy.MANUAL_BY_TOPIC)
```

------

#### **6. Deadline (时限)**

规定消息在一定时间内必须被接收，如果超时，系统会报告通信异常。

- 适用场景：
  - 用于实时系统，例如定期发送的传感器数据。

**示例：**

```python
from builtin_interfaces.msg import Duration
qos = QoSProfile(deadline=Duration(sec=1, nanosec=0))
```

------

#### **7. Lifespan (生存时间)**

定义消息的有效期。如果消息在队列中停留的时间超过生存时间，它将被丢弃。

- 适用场景：
  - 避免处理过期的旧消息，例如实时视频流中的旧帧。

**示例：**

```python
qos = QoSProfile(lifespan=Duration(sec=5))
```

------

### **综合示例：自定义 QoS 配置**

以下是一个自定义 QoS 配置的示例，适用于高可靠性、深度有限的场景：

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Duration

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    deadline=Duration(sec=1),
    lifespan=Duration(sec=5)
)
```

此配置说明：

- 使用 **可靠传输**（RELIABLE）。
- 订阅者不会收到历史消息（VOLATILE）。
- 保留最近的 10 条消息（KEEP_LAST，depth=10）。
- 每 1 秒必须接收消息（deadline）。
- 消息的有效期为 5 秒（lifespan）。

------

### **QoS 策略的典型应用场景**

| **场景**                       | **建议的 QoS 配置**                                          |
| ------------------------------ | ------------------------------------------------------------ |
| **实时视频流**                 | `BEST_EFFORT`（低延迟），`VOLATILE`（无需历史消息）。        |
| **机器人控制指令**             | `RELIABLE`（高可靠性），`KEEP_LAST`（仅需最近的指令）。      |
| **状态信息发布（如心跳信号）** | `RELIABLE`（必须接收），`TRANSIENT_LOCAL`（新订阅者可接收最近状态）。 |
| **日志数据记录**               | `RELIABLE`（无数据丢失），`KEEP_ALL`（保存所有历史消息）。   |
| **实时传感器数据**             | `RELIABLE`（准确传递），`KEEP_LAST`（只保留最新传感器数据，避免过多内存占用）。 |

------

### **总结**

1. **QoS 是 ROS 2 中用于控制通信行为的关键工具，支持各种应用场景的优化。**
2. 主要策略：
   - **Reliability:** 控制消息传输的可靠性。
   - **Durability:** 控制新订阅者是否接收历史消息。
   - **History:** 决定消息的存储方式（最近 N 条或全部消息）。
   - **Depth:** 决定消息队列的大小。
   - **Liveliness, Deadline, Lifespan:** 增强通信的实时性和有效性。
3. **选择合适的 QoS 配置：** 根据具体的应用场景（如实时性、可靠性或资源限制）调整 QoS 策略，以达到最佳效果。



## 3. **QoS 怎么使用？在哪里使用？**⭐️

在 ROS 2 中，**QoS（Quality of Service）** 是用于配置 **发布者（Publisher）** 和 **订阅者（Subscriber）** 之间通信行为的工具。**你可以在创建发布者或订阅者时使用 QoS，来定制消息的传递方式**。

------

### **在哪里使用 QoS？**

QoS 主要在以下场景使用：

1. **创建发布者时：指定消息的发送方式，例如是否保证消息可靠传输。**
2. **创建订阅者时**：**指定消息的接收方式，例如是否接收历史消息或是否丢弃过期数据。**

**注意：**

- 发布者和订阅者必须使用兼容的 QoS 配置，才能正常通信。例如，**如果发布者使用 `RELIABLE`（可靠传输），订阅者也应使用相同的设置。**

------

### **QoS 怎么使用？**

#### **1. 定义一个 QoS 配置**

使用 `QoSProfile` 类定义一个 QoS 配置，用于指定通信的行为，例如可靠性（Reliability）、持久性（Durability）、深度（Depth）等。

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# 定义一个 QoS 配置
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,       # 确保消息可靠传输
    durability=DurabilityPolicy.TRANSIENT_LOCAL, # 新订阅者可以收到最近的消息
    depth=10                                     # 队列深度为 10，最多存储 10 条消息
)
```

------

#### **2. 在创建发布者时使用 QoS**

将定义好的 QoS 配置传递给发布者。

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

class QoSPublisher(Node):
    def __init__(self):
        super().__init__('qos_publisher')
        
        # 定义 QoS 配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # 可靠传输
            depth=5                                  # 队列深度为 5
        )
        
        # 创建发布者并应用 QoS 配置
        self.publisher = self.create_publisher(String, 'qos_topic', qos_profile)
        
        # 定时器，每隔 1 秒发布一条消息
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = 'Hello with QoS!'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = QoSPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**解释：**

- **QoS 配置：** 发布者使用 `RELIABLE` 确保消息可靠传输，同时队列深度为 5，最多存储 5 条消息。
- **主题：** 发布到 `qos_topic` 话题，使用上述 QoS 配置。

------

#### **3. 在创建订阅者时使用 QoS**

同样，在订阅者中也需要传递 QoS 配置，确保接收消息的方式与发布者兼容。

```python
from rclpy.qos import ReliabilityPolicy

class QoSSubscriber(Node):
    def __init__(self):
        super().__init__('qos_subscriber')
        
        # 定义 QoS 配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE  # 匹配发布者的可靠传输配置
        )
        
        # 创建订阅者并应用 QoS 配置
        self.subscriber = self.create_subscription(
            String,
            'qos_topic',
            self.listener_callback,
            qos_profile
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
```

**解释：**

- **QoS 配置：** 订阅者也使用 `RELIABLE`，以确保能够接收到发布者发送的所有消息。
- **订阅：** 监听 `qos_topic` 话题，接收符合 QoS 配置的消息。

------

### **常见场景中的 QoS 使用方法**

1. **可靠传输（关键命令）**
   - **场景：** 控制机器人移动的命令（例如“移动到目标位置”）。
   - QoS 配置：
     - **Reliability：** `RELIABLE`（确保每条命令被接收）。
     - **Durability：** `VOLATILE`（不需要保存历史命令）。
2. **实时数据流（摄像头数据）**
   - **场景：** 发布实时视频流。
   - QoS 配置：
     - **Reliability：** `BEST_EFFORT`（允许丢帧以减少延迟）。
     - **History：** `KEEP_LAST`（只保存最近几帧）。
3. **共享状态信息（心跳信号、诊断信息）**
   - **场景：** 向新加入的节点提供最近的状态。
   - QoS 配置：
     - **Durability：** `TRANSIENT_LOCAL`（新订阅者可以接收到最近的状态信息）。
4. **资源优化（高频率传感器数据）**
   - **场景：** 发布激光雷达数据，发布速率高，订阅者处理速度慢。
   - QoS 配置：
     - **Depth：** `10`（限制队列深度以防止内存溢出）。
     - **Reliability：** `BEST_EFFORT`（减少数据重传的开销）。

------

### **完整示例：发布者和订阅者匹配 QoS 配置**

以下是一个完整的例子，其中发布者和订阅者使用相同的 QoS 配置，确保可靠通信。

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class QoSPublisher(Node):
    def __init__(self):
        super().__init__('qos_publisher')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # 可靠传输
            history=HistoryPolicy.KEEP_LAST,         # 保存最近的消息
            depth=10                                 # 队列深度为 10
        )
        
        self.publisher = self.create_publisher(String, 'qos_topic', qos_profile)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = 'Hello, QoS!'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

class QoSSubscriber(Node):
    def __init__(self):
        super().__init__('qos_subscriber')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # 匹配发布者的配置
            history=HistoryPolicy.KEEP_LAST,         # 保存最近的消息
            depth=10                                 # 队列深度为 10
        )
        
        self.subscriber = self.create_subscription(
            String,
            'qos_topic',
            self.listener_callback,
            qos_profile
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    publisher = QoSPublisher()
    subscriber = QoSSubscriber()
    
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(subscriber)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**运行效果：**

- 发布者和订阅者使用相同的 QoS 配置。
- 每 1 秒发布一条消息，订阅者可靠接收消息。

------

### **总结**

- **在哪里使用 QoS？**
  - 在 **创建发布者和订阅者** 时使用，配置消息的发送和接收行为。
- **怎么使用 QoS？**
  - 定义一个 `QoSProfile`，并将其传递给发布者或订阅者。

通过使用 QoS，可以根据具体应用需求（如可靠性、实时性或资源优化）调整 ROS 2 的通信行为，使得系统更加稳定、高效。
