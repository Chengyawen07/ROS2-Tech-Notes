## 1.**What is QoS? (Simplified Explanation)**

**QoS (Quality of Service)** is like a set of "rules" or "policies" in ROS 2 that controls how nodes communicate by sending and receiving messages. QoS lets you customize how messages are transmitted, ensuring they fit your application's specific needs.

------

### **Why Do We Need QoS? (Purpose)**

Different robotic applications have different communication requirements:

- **Real-time systems:** Some tasks require fast and timely delivery of messages (e.g., obstacle avoidance).
- **Reliable systems:** Other tasks need to ensure every single message is received (e.g., robot control commands).
- **Resource optimization:** Some tasks need to minimize memory or bandwidth usage (e.g., video streaming).

**QoS helps tailor communication behavior** to achieve better performance, reliability, or resource efficiency depending on your needs.

------

### **An Easy Analogy**

Think of communication in ROS 2 like sending packages by mail. QoS is the set of "delivery rules" that decide how those packages (messages) are handled. For example:

1. **Reliability:**
   - **RELIABLE (like tracked delivery):** Every package must be delivered safely. If a package gets lost, it will be resent (e.g., critical documents).
   - **BEST_EFFORT (like regular mail):** No guarantees—if the package gets lost, it’s gone (e.g., promotional flyers).
2. **Durability:**
   - **VOLATILE ("no delivery history"):** New subscribers only get packages sent after they joined; no past packages are saved.
   - **TRANSIENT_LOCAL ("temporary storage"):** The publisher keeps recent packages, and new subscribers can receive them.
3. **History:**
   - **KEEP_LAST ("limited storage"):** Only the latest `N` packages are stored (e.g., last 10), with older ones overwritten.
   - **KEEP_ALL ("store everything"):** All packages are stored, but this uses more memory.
4. **Depth:**
   - Controls how many packages can wait in line to be delivered at once (e.g., a queue size of 5).
5. **Lifespan:**
   - Packages expire if they sit in the queue for too long (e.g., fresh food only valid for one day).
6. **Deadline:**
   - A package must be delivered within a specific time; otherwise, it’s considered failed delivery.
7. **Liveliness:**
   - Checks if the sender (publisher) is still alive. If it stops sending, the system detects it and raises an alarm.

------

### **Uses and Examples**

#### **1. Ensuring Reliable Delivery**

- **Scenario:** Sending critical robot control commands (e.g., "Move to position X").
- QoS Configuration:
  - **Reliability:** `RELIABLE` (ensure all commands are delivered).
  - **Durability:** `VOLATILE` (no need for old commands).

#### **2. Handling Real-Time Data Streams**

- **Scenario:** Streaming live video from a camera.
- QoS Configuration:
  - **Reliability:** `BEST_EFFORT` (dropping frames is acceptable to reduce latency).
  - **History:** `KEEP_LAST` (only store recent frames).

#### **3. Sending State Information to New Nodes**

- **Scenario:** Sharing the robot's current state with a new subscriber (e.g., a diagnostic tool).
- QoS Configuration:
  - **Durability:** `TRANSIENT_LOCAL` (new subscribers receive the most recent state message).

#### **4. Optimizing Resources**

- **Scenario:** Publishing laser scan data at high frequency, but the subscriber processes data slowly.
- QoS Configuration:
  - **Depth:** `10` (limit queue size to prevent memory overload).
  - **Reliability:** `BEST_EFFORT` (reduce overhead of retransmissions).

------

### **Summary**

Simplifying QoS:

- QoS is a way to define "rules" for message transmission to control **reliability, storage, and timing**.
- It allows you to adjust communication behavior based on your specific application's needs.

------

### **When to Use QoS**

| **Scenario**                                 | **Recommended QoS**                              |
| -------------------------------------------- | ------------------------------------------------ |
| **Critical commands (e.g., robot movement)** | `RELIABLE`, `VOLATILE`                           |
| **Live video streaming**                     | `BEST_EFFORT`, `VOLATILE`, `KEEP_LAST`           |
| **Sharing robot state with new subscribers** | `RELIABLE`, `TRANSIENT_LOCAL`                    |
| **Saving all data for logging**              | `RELIABLE`, `KEEP_ALL`                           |
| **Optimizing memory (e.g., sensor data)**    | `BEST_EFFORT`, `KEEP_LAST`, with limited `depth` |

**QoS Purpose:** To ensure message delivery behavior is tailored to meet the system's specific performance, reliability, and resource requirements.



## 2. **What is QoS (Quality of Service)?**

**QoS (Quality of Service)** in ROS 2 refers to a set of policies that control how nodes communicate and ensure performance, reliability, and adaptability in different scenarios. QoS determines how messages are transmitted between **Publishers** and **Subscribers**, particularly in cases of network instability or limited bandwidth.

#### **Purpose of QoS:**

1. **Improve Communication Reliability:** Ensure messages are delivered as expected.
2. **Optimize Performance:** Balance between real-time constraints and reliability.
3. **Adapt to Application Needs:** Support scenarios ranging from real-time control to delay-tolerant tasks.

------

### **QoS Policies in ROS 2**

ROS 2's QoS policies are based on the DDS (Data Distribution Service) standard and provide flexibility to meet various application requirements. Below are the main QoS policies available in ROS 2:

------

#### **1. Reliability**

Controls the reliability of message delivery.

- **RELIABLE:**
   Guarantees message delivery. If a message is not received, the system retries. Suitable for critical data, such as robot control commands.
- **BEST_EFFORT:**
   Attempts to deliver messages but does not guarantee delivery. Suitable for scenarios requiring low latency, such as video streams.

**Example:**

```python
from rclpy.qos import ReliabilityPolicy, QoSProfile

qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE)
```

------

#### **2. Durability**

Controls whether new subscribers receive messages published before they started subscribing.

- **VOLATILE:**
   Subscribers only receive messages published after they start subscribing. No history is retained.
- **TRANSIENT_LOCAL:**
   The publisher stores recent messages, and new subscribers can access this history. Useful for sharing the latest state information.

**Example:**

```python
from rclpy.qos import DurabilityPolicy, QoSProfile

qos = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL)
```

------

#### **3. History**

Determines how messages are stored and how many are retained.

- **KEEP_LAST:**
   Retains only the last `N` messages (default: 10). Suitable for large datasets where storing all messages is impractical.
- **KEEP_ALL:**
   Retains all messages, ensuring no data loss. This may consume significant memory.

**Example:**

```python
from rclpy.qos import HistoryPolicy, QoSProfile

qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5)
```

- **`depth`:** Specifies how many messages to store when using `KEEP_LAST`.

------

#### **4. Depth**

Defines the size of the message queue, i.e., the maximum number of messages a publisher or subscriber can store.

- **Use Case:**
   If the publishing rate exceeds the subscriber's processing rate, increasing the depth prevents message loss.

**Example:**

```python
qos = QoSProfile(depth=10)
```

------

#### **5. Liveliness**

Determines how to monitor the activity of a publisher to ensure it is alive.

- **AUTOMATIC:**
   The system automatically checks the publisher's activity. Suitable for most cases.
- **MANUAL_BY_TOPIC:**
   The publisher must manually signal that it is alive, providing greater control.

**Example:**

```python
from rclpy.qos import LivelinessPolicy, QoSProfile

qos = QoSProfile(liveliness=LivelinessPolicy.MANUAL_BY_TOPIC)
```

------

#### **6. Deadline**

Specifies the time period within which a message must be received. If the deadline is missed, the system reports a communication issue.

- **Use Case:**
   Ideal for real-time systems where periodic updates are critical, such as sensor data.

**Example:**

```python
from builtin_interfaces.msg import Duration
qos = QoSProfile(deadline=Duration(sec=1, nanosec=0))
```

------

#### **7. Lifespan**

Defines the validity period of a message. Messages that exceed this duration in the queue are discarded.

- **Use Case:**
   Ensures outdated messages (e.g., old video frames) are not processed.

**Example:**

```python
qos = QoSProfile(lifespan=Duration(sec=5))
```

------

### **Comprehensive Example: Custom QoS Configuration**

Below is an example of a custom QoS configuration for a scenario requiring high reliability and controlled message retention:

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

Explanation:

- **RELIABLE:** Ensures every message is delivered.
- **VOLATILE:** No historical messages are sent to new subscribers.
- **KEEP_LAST, depth=10:** Retains the last 10 messages.
- **Deadline:** Messages must be received within 1 second.
- **Lifespan:** Messages older than 5 seconds are discarded.

------

### **Typical QoS Configurations for Different Scenarios**

| **Scenario**                            | **Recommended QoS Configuration**                            |
| --------------------------------------- | ------------------------------------------------------------ |
| **Real-Time Video Streaming**           | `BEST_EFFORT` (low latency), `VOLATILE` (no history needed). |
| **Robot Control Commands**              | `RELIABLE` (critical commands), `KEEP_LAST` (retain recent commands only). |
| **State Information (e.g., Heartbeat)** | `RELIABLE` (guaranteed delivery), `TRANSIENT_LOCAL` (new subscribers get the latest state). |
| **Data Logging**                        | `RELIABLE` (no data loss), `KEEP_ALL` (retain all messages for analysis). |
| **Sensor Data Updates**                 | `RELIABLE` (accuracy), `KEEP_LAST` (retain only recent data to reduce memory usage). |

------

### **Summary**

1. **QoS (Quality of Service)** is a crucial tool in ROS 2 to control communication behavior and optimize performance in different scenarios.
2. **Key QoS Policies:**
   - **Reliability:** Ensures message delivery (`RELIABLE`, `BEST_EFFORT`).
   - **Durability:** Handles historical messages (`VOLATILE`, `TRANSIENT_LOCAL`).
   - **History:** Controls message retention (`KEEP_LAST`, `KEEP_ALL`).
   - **Depth:** Sets the message queue size.
   - **Liveliness, Deadline, Lifespan:** Ensure timely and valid message communication.
3. **Choosing the Right QoS:** Select QoS policies based on your application's requirements for real-time performance, reliability, and resource constraints.

By understanding and applying QoS effectively, you can tailor ROS 2 communication to meet the specific needs of your robotic system.



## 3. **How to Use QoS in ROS 2?**

In ROS 2, **QoS (Quality of Service)** is used to configure the behavior of communication between **Publishers** and **Subscribers**. You use it when creating publishers or subscribers to customize how messages are sent and received.

------

### **Where to Use QoS?**

QoS is primarily used when:

1. **Creating a Publisher** to define how it sends messages.
2. **Creating a Subscriber** to define how it receives messages.

Both publishers and subscribers must align their QoS settings to communicate effectively. For example, if a publisher uses `RELIABLE`, but a subscriber uses `BEST_EFFORT`, their communication might not work as expected.

------

### **How to Use QoS?**

#### **1. Define a QoS Profile**

In ROS 2, you create a **QoS profile** using the `QoSProfile` class. This profile specifies the desired QoS settings such as reliability, durability, depth, and more.

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Define a QoS profile
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,       # Ensure reliable communication
    durability=DurabilityPolicy.TRANSIENT_LOCAL, # Retain recent messages for new subscribers
    depth=10                                     # Keep the last 10 messages in the queue
)
```

------

#### **2. Use QoS When Creating a Publisher**

When creating a publisher, pass the QoS profile as an argument.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

class QoSPublisher(Node):
    def __init__(self):
        super().__init__('qos_publisher')
        
        # Define a QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=5  # Retain the last 5 messages
        )
        
        # Create a publisher with the QoS profile
        self.publisher = self.create_publisher(String, 'qos_topic', qos_profile)
        
        # Publish a message every second
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

**Explanation:**

- **QoS Profile:** Configures the publisher to use reliable communication (`RELIABLE`) and keep the last 5 messages in its queue.
- **Topic:** The topic `qos_topic` will use the defined QoS settings.

------

#### **3. Use QoS When Creating a Subscriber**

When creating a subscriber, pass a matching QoS profile to ensure proper communication with the publisher.

```python
from rclpy.qos import ReliabilityPolicy

class QoSSubscriber(Node):
    def __init__(self):
        super().__init__('qos_subscriber')
        
        # Define a QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE  # Match publisher's QoS
        )
        
        # Create a subscriber with the QoS profile
        self.subscriber = self.create_subscription(
            String,
            'qos_topic',
            self.listener_callback,
            qos_profile
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
```

**Explanation:**

- **QoS Profile:** Matches the publisher's `RELIABLE` policy to ensure successful communication.
- **Subscriber:** Subscribes to the same `qos_topic` with the specified QoS.

------

### **Common QoS Usage Scenarios**

1. **Reliable Communication (Critical Commands):**
   - Use `RELIABLE` reliability and `KEEP_LAST` history to ensure important messages (e.g., robot control commands) are delivered.
2. **Low-Latency Streaming (Camera Data):**
   - Use `BEST_EFFORT` reliability for high-throughput, low-latency data like video streams.
3. **State Sharing (Diagnostics/Heartbeats):**
   - Use `TRANSIENT_LOCAL` durability to ensure new subscribers receive the latest state information.
4. **Logging or Recording Data:**
   - Use `KEEP_ALL` history to store all messages for later analysis.

------

### **Key Points for Using QoS**

1. **Define QoS Per Topic:** You can customize QoS for each topic individually.
2. **Matching QoS:** Publishers and subscribers communicating on the same topic must have compatible QoS settings (e.g., both use `RELIABLE`).
3. **Default QoS:** If no custom QoS is provided, ROS 2 uses a default profile with reasonable settings.
4. **Debugging QoS:** Use `ros2 topic echo` or `ros2 topic info` to check a topic's QoS settings during runtime.

------

### **Practical Example: Matching Publisher and Subscriber**

Here’s a complete example with a publisher and a subscriber using matching QoS settings:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class QoSPublisher(Node):
    def __init__(self):
        super().__init__('qos_publisher')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
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
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
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

**How it works:**

- Both the publisher and subscriber use `RELIABLE` communication with a queue depth of 10.
- The subscriber receives every message published by the publisher.

------

### **Conclusion**

- **Where to use QoS?**
  - Use QoS when creating publishers and subscribers to define how they handle communication.
- **How to use QoS?**
  - Define a `QoSProfile` and pass it to the publisher or subscriber during creation.

By understanding and applying QoS effectively, you can ensure reliable, efficient, and adaptable communication between ROS 2 nodes tailored to your application's requirements.
