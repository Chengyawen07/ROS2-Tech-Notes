### **What is a Message in ROS 2?**

A **message** in ROS 2 is the basic data carrier used for communication between nodes.

- Each **topic** and **service** in ROS 2 relies on a message type to define the structure and format of the transmitted data.
- Messages typically consist of fields and their types, such as strings, integers, floats, or even nested message types.

In simple terms, a message is like an **envelope** that contains the data to be transmitted, and ROS 2 nodes exchange these "envelopes" through topics or services.

**Standard Message Types:** ROS 2 provides commonly used standard message types, including:

- **`std_msgs`:** Basic types like `String`, `Int32`, etc.
- **`sensor_msgs`:** Sensor data types like `Image`, `LaserScan`, etc.
- **`geometry_msgs`:** Geometric data types like `Pose`, `Twist`, etc.

------

### **How to Define a Custom Message Type?**

Sometimes, standard message types may not meet your requirements, and you need to create custom message types.

------

#### **Step 1: Create a Message Package**

1. Create a new package for the custom messages:

   ```bash
   ros2 pkg create my_custom_msgs --build-type ament_cmake
   ```

2. Navigate to the package directory:

   ```bash
   cd my_custom_msgs
   ```

------

#### **Step 2: Define a Custom Message File**

1. Inside the package, create a `msg` directory:

   ```bash
   mkdir msg
   touch msg/CustomMessage.msg
   ```

2. Define the message content in `CustomMessage.msg`. For example:

   ```plaintext
   int32 id           # Integer ID
   string name        # Name as a string
   float32[] values   # Array of floating-point numbers
   ```

------

#### **Step 3: Update `CMakeLists.txt`**

1. Ensure `rosidl_default_generators` is included in `find_package`:

   ```cmake
   find_package(rosidl_default_generators REQUIRED)
   ```

2. Add the message file:

   ```cmake
   set(msg_files
       "msg/CustomMessage.msg"
   )
   ```

3. Generate message interfaces:

   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
       ${msg_files}
   )
   ```

4. Install the message files:

   ```cmake
   install(
       DIRECTORY msg
       DESTINATION share/${PROJECT_NAME}
   )
   ```

------

#### **Step 4: Update `package.xml`**

1. Add the required dependencies:

   ```xml
   <build_depend>rosidl_default_generators</build_depend>
   <exec_depend>rosidl_default_runtime</exec_depend>
   ```

2. Ensure you export the necessary settings:

   ```xml
   <export>
       <build_type>ament_cmake</build_type>
   </export>
   ```

------

#### **Step 5: Build the Message Package**

1. Build the package:

   ```bash
   colcon build --packages-select my_custom_msgs
   ```

2. Verify the generated message type:

   ```bash
   ros2 interface show my_custom_msgs/msg/CustomMessage
   ```

------

#### **Step 6: Use the Custom Message**

1. Declare the dependency in other packages:

   - In 

     ```
     package.xml
     ```

     :

     ```xml
     <depend>my_custom_msgs</depend>
     ```

   - In 

     ```
     CMakeLists.txt
     ```

     :

     ```cmake
     find_package(my_custom_msgs REQUIRED)
     ```

2. Publish the custom message (Python example):

   ```python
   import rclpy
   from rclpy.node import Node
   from my_custom_msgs.msg import CustomMessage  # Import custom message
   
   class CustomPublisher(Node):
       def __init__(self):
           super().__init__('custom_publisher')
           self.publisher_ = self.create_publisher(CustomMessage, 'custom_topic', 10)
           self.timer = self.create_timer(1.0, self.publish_message)
   
       def publish_message(self):
           msg = CustomMessage()
           msg.id = 1
           msg.name = "example"
           msg.values = [1.0, 2.0, 3.0]
           self.publisher_.publish(msg)
           self.get_logger().info(f"Publishing: {msg}")
   
   def main(args=None):
       rclpy.init(args=args)
       node = CustomPublisher()
       rclpy.spin(node)
       rclpy.shutdown()
   
   if __name__ == '__main__':
       main()
   ```

3. Subscribe to the custom message (Python example):

   ```python
   import rclpy
   from rclpy.node import Node
   from my_custom_msgs.msg import CustomMessage  # Import custom message
   
   class CustomSubscriber(Node):
       def __init__(self):
           super().__init__('custom_subscriber')
           self.subscription = self.create_subscription(
               CustomMessage, 'custom_topic', self.callback, 10)
   
       def callback(self, msg):
           self.get_logger().info(f"Received: id={msg.id}, name={msg.name}, values={msg.values}")
   
   def main(args=None):
       rclpy.init(args=args)
       node = CustomSubscriber()
       rclpy.spin(node)
       rclpy.shutdown()
   
   if __name__ == '__main__':
       main()
   ```

------

### **Summary**

- **Message:** The fundamental structure for transferring data between ROS 2 nodes, consisting of fields and types.
- **Custom Messages:** Defined using `.msg` files, suitable for specific or complex data structures.
- **Use Case:** Custom messages are ideal for handling specialized tasks like robot states, image processing, and other advanced scenarios.
