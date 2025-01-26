### **How to Use Timers in ROS 2**

In ROS 2, a **timer** is a mechanism to periodically execute specific tasks, such as publishing messages, reading sensor data, or running computations. Timers are managed by the ROS 2 **event loop** (executor) and are tied to a node.

------

### **Basic Concepts of Timers**

1. **Core Functionality:**
   - A timer triggers a callback function after a specified interval.
   - It is created using the `create_timer()` method and runs as part of a node.
2. **Interval:**
   - The timer interval is specified in seconds and can be an integer (e.g., 1 second) or a floating-point value (e.g., 0.5 seconds).
3. **Callback Function:**
   - When the timer is triggered, the specified callback function is executed.

------

### **Common Use Cases for Timers**

#### **1. Creating a Simple Timer**

Here’s an example of a simple timer that logs a message every second.

**Code Example:**

```python
import rclpy
from rclpy.node import Node

class SimpleTimerNode(Node):
    def __init__(self):
        super().__init__('simple_timer_node')  # Define the node name
        self.counter = 0
        # Create a timer that triggers every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f"Timer triggered {self.counter} times.")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTimerNode()
    rclpy.spin(node)  # Keep the node running
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Execution:**

```bash
ros2 run <package_name> simple_timer_node
```

**Output:**

```
[INFO] [<timestamp>] [simple_timer_node]: Timer triggered 1 times.
[INFO] [<timestamp>] [simple_timer_node]: Timer triggered 2 times.
...
```

------

#### **2. Using a Timer to Publish Messages**

Here’s an example of using a timer to periodically publish messages.

**Code Example:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TimerPublisherNode(Node):
    def __init__(self):
        super().__init__('timer_publisher_node')
        self.publisher = self.create_publisher(String, 'example_topic', 10)  # Create a publisher
        self.timer = self.create_timer(0.5, self.timer_callback)  # Timer triggers every 0.5 seconds
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello, ROS 2! Message count: {self.counter}"
        self.publisher.publish(msg)  # Publish the message
        self.get_logger().info(f"Published: '{msg.data}'")
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TimerPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Execution:**

```bash
ros2 run <package_name> timer_publisher_node
```

**Output:**

```
[INFO] [<timestamp>] [timer_publisher_node]: Published: 'Hello, ROS 2! Message count: 0'
[INFO] [<timestamp>] [timer_publisher_node]: Published: 'Hello, ROS 2! Message count: 1'
...
```

------

#### **3. Dynamically Adjusting Timer Frequency**

You can use ROS 2 parameters to dynamically change the timer interval.

**Code Example:**

```python
import rclpy
from rclpy.node import Node

class DynamicTimerNode(Node):
    def __init__(self):
        super().__init__('dynamic_timer_node')
        self.declare_parameter('timer_interval', 1.0)  # Declare a parameter with a default value of 1.0 seconds
        self.timer_interval = self.get_parameter('timer_interval').get_parameter_value().double_value
        self.timer = self.create_timer(self.timer_interval, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(f"Timer triggered with interval: {self.timer_interval} seconds.")

def main(args=None):
    rclpy.init(args=args)
    node = DynamicTimerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Execution:**

```bash
ros2 run <package_name> dynamic_timer_node
```

**Adjust Timer Interval Dynamically:** Use the `ros2 param` command to change the interval at runtime:

```bash
ros2 param set /dynamic_timer_node timer_interval 0.5
```

**Effect:**

- The interval will update in real-time, and the timer callback frequency will change accordingly.

------

#### **4. Using Multiple Timers in a Node**

You can create multiple timers in a single node. Here’s an example:

**Code Example:**

```python
import rclpy
from rclpy.node import Node

class MultiTimerNode(Node):
    def __init__(self):
        super().__init__('multi_timer_node')
        # Timer 1 triggers every 1 second
        self.timer1 = self.create_timer(1.0, self.timer1_callback)
        # Timer 2 triggers every 0.5 seconds
        self.timer2 = self.create_timer(0.5, self.timer2_callback)

    def timer1_callback(self):
        self.get_logger().info("Timer 1 triggered!")

    def timer2_callback(self):
        self.get_logger().info("Timer 2 triggered!")

def main(args=None):
    rclpy.init(args=args)
    node = MultiTimerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Execution:**

```bash
ros2 run <package_name> multi_timer_node
```

**Output:**

- Timer 1 and Timer 2 run independently at their respective intervals.

------

### **Summary**

| **Functionality**          | **Method**                                                   |
| -------------------------- | ------------------------------------------------------------ |
| **Create a Timer**         | `create_timer(interval, callback)` creates a timer with a set interval. |
| **Dynamic Timer Interval** | Use parameters (`declare_parameter` and `get_parameter`) to adjust dynamically. |
| **Publish Messages**       | Use timers to periodically publish messages.                 |
| **Multiple Timers**        | Create and manage multiple timers in one node.               |

------

### **Typical Use Cases**

1. **Periodic Data Publishing:** For example, publishing sensor data at regular intervals.
2. **State Updates:** Regularly checking and updating robot states.
3. **Dynamic Behavior:** Adjusting timer frequency based on external conditions.

By mastering ROS 2 timers, you can handle periodic tasks efficiently and create dynamic, responsive robotic systems.
