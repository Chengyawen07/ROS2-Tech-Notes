## 1. **What is an Action in ROS 2?**

An **Action** in ROS 2 is a communication mechanism specifically designed for handling **long-running tasks**.

- It follows the **Goal-Feedback-Result** model, where a client sends a goal, receives periodic feedback during task execution, and finally gets the result when the task is completed.
- **Use Cases:** Actions are ideal for tasks like robot navigation, arm movement planning, or any operation that requires progress updates over time.

------

### **How Actions Work**

1. **Goal:** The client sends a task goal to the server.
2. **Feedback:** The server processes the task and periodically sends feedback to the client.
3. **Result:** Once the task is complete, the server sends the final result to the client.

In simple terms, **Action is like ordering food for delivery**:

- **Goal:** The client (customer) places an order (goal).
- **Feedback:** The delivery person (server) updates the status (e.g., picking up the order, on the way).
- **Result:** The delivery is completed, and the client receives the food.

------

### **Comparison of Action with Topic and Service**

| Feature                 | **Action**                                               | **Topic**                                   | **Service**                                      |
| ----------------------- | -------------------------------------------------------- | ------------------------------------------- | ------------------------------------------------ |
| **Communication Model** | Goal-Feedback-Result                                     | Publish-Subscribe                           | Request-Response                                 |
| **Data Flow**           | Bi-directional, multi-stage (Goal -> Feedback -> Result) | Uni-directional (Publisher -> Subscriber)   | Bi-directional (Request -> Response)             |
| **Use Case**            | Long-running tasks (e.g., navigation, path planning)     | Continuous data streams (e.g., sensor data) | One-time tasks (e.g., querying status, commands) |
| **Real-Time Feedback**  | Supported (via Feedback)                                 | Not supported                               | Not supported                                    |
| **Sync/Async**          | Asynchronous                                             | Asynchronous                                | Typically synchronous (wait for response)        |

------

### **How to Implement Actions in ROS 2**

Below is a simple example showing how to create an Action server and client.

------

#### **Action Server**

**Functionality:** Receives a goal, provides real-time feedback, and sends the final result.

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci  # Standard Action interface

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

        goal_handle.succeed()  # Mark task as successful
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

#### **Action Client**

**Functionality:** Sends a goal to the server, receives feedback, and processes the result.

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

        self.action_client.wait_for_server()  # Wait for server availability
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
    node.send_goal(order=10)  # Send goal with order=10
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

------

### **Explanation of the Example**

1. **Action Server:**
   - The `execute_callback` handles the received goal.
   - Feedback is sent to the client using `publish_feedback`.
   - The task is marked complete with `goal_handle.succeed()`, and the result is returned.
2. **Action Client:**
   - Sends the goal asynchronously using `send_goal_async`.
   - Processes feedback through `feedback_callback`.
   - Handles the final result in `result_callback`.

------

### **Features of Action**

- **Supports Real-Time Feedback:** Clients can track progress during task execution.
- **Asynchronous Operation:** Clients can perform other operations while the task is being executed.
- **Multi-Stage Communication:** Actions include goals, feedback, and results, making them ideal for complex tasks.

------

### **Summary**

- **Action:** Best for long-running tasks that need progress updates and final results (e.g., robot navigation).
- **Topic:** Ideal for continuous, stateless data streams (e.g., sensor data).
- **Service:** Suitable for short-lived, synchronous tasks (e.g., querying a status or sending commands).

Choosing between them depends on the task requirements:

- Use **Action** for long-running tasks with progress feedback.
- Use **Topic** for real-time, continuous data.
- Use **Service** for one-time, immediate operations.





## 2. **How Navigation Uses Action Communication in ROS 2**

Navigation in ROS 2 (e.g., **Navigation2**) heavily relies on **Action Communication** to perform tasks like navigating to a goal, following a path, or canceling navigation tasks. Actions provide a robust framework for these long-running tasks, allowing for **real-time feedback**, **cancellation**, and **final results**.

------

### **Key Navigation Actions in Navigation2**

Here are the main Actions used in ROS 2 Navigation:

1. **NavigateToPose** (`nav2_msgs/action/NavigateToPose`):
   - Used to navigate the robot to a specified pose (goal position and orientation).
   - Includes:
     - **Goal:** Target pose (e.g., coordinates and orientation).
     - **Feedback:** Progress updates, such as the distance remaining to the target.
     - **Result:** Whether the goal was reached successfully or failed.
2. **FollowPath (`nav2_msgs/action/FollowPath`):**
   - Directs the robot to follow a pre-computed path.
3. **Spin** (`nav2_msgs/action/Spin`):
   - Rotates the robot in place by a specific angle.
4. **Wait** (`nav2_msgs/action/Wait`):
   - Pauses navigation for a specific duration.
5. **ComputePathToPose** (`nav2_msgs/action/ComputePathToPose`):
   - Generates a navigation path to a target pose without executing it.

------

### **How Action Communication Works in Navigation**

#### **1. Sending a Goal**

The **NavigateToPose** action client sends a navigation goal to the **action server (e.g., the Navigation2 stack).**

- **Goal:** Specifies the target pose (position and orientation in the map frame).
- The server starts executing the task and provides feedback.

#### **2. Receiving Feedback**

The server periodically sends **feedback** to the client, such as:

- Remaining distance to the goal.
- Current robot velocity or position.

This allows the client to monitor progress in real-time.

#### **3. Getting the Result**

Once the task is complete (or canceled), the server sends the **result**, indicating:

- Whether the goal was reached successfully.
- If the task failed, the reason for failure (e.g., obstacles, localization issues).

#### **4. Canceling a Goal**

The client can also cancel the goal during execution if needed (e.g., if the user changes the destination).

------

### **Example: Navigating to a Pose Using Action**

#### **Action Client Code for Navigation**

Below is a Python example of how to use the **NavigateToPose** action client to send a goal and receive feedback.

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

        self.action_client.wait_for_server()  # Wait for the navigation action server
        self.get_logger().info('Sending goal...')
        self.future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: Remaining distance: {feedback.distance_remaining:.2f} m')

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().error('Failed to reach the goal.')

def main(args=None):
    rclpy.init(args=args)

    # Create a NavigationClient node
    navigation_client = NavigationClient()

    # Define the target pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'map'
    target_pose.pose.position.x = 2.0
    target_pose.pose.position.y = 3.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 1.0

    # Send the goal
    navigation_client.send_goal(target_pose)

    # Keep the node running
    rclpy.spin(navigation_client)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

------

### **Explanation of the Code**

1. **Action Client Initialization:**
   - `ActionClient(self, NavigateToPose, 'navigate_to_pose')` connects to the Navigation2 action server for navigation tasks.
2. **Goal Setup:**
   - The target pose (`PoseStamped`) specifies the desired position and orientation in the `map` frame.
3. **Sending the Goal:**
   - `send_goal_async` sends the navigation goal to the server and registers callbacks for feedback and results.
4. **Feedback Handling:**
   - `feedback_callback` receives periodic updates on the remaining distance to the goal.
5. **Result Handling:**
   - `result_callback` processes the final result, determining whether the goal was reached successfully or not.

------

### **Comparison with Other Communication Mechanisms**

| **Feature**                | **Action (NavigateToPose)**              | **Topic (e.g., /cmd_vel)**                  | **Service (e.g., ComputePathToPose)**       |
| -------------------------- | ---------------------------------------- | ------------------------------------------- | ------------------------------------------- |
| **Use Case**               | Long-running tasks with progress updates | Real-time control (e.g., velocity commands) | One-time computations (e.g., path planning) |
| **Real-Time Feedback**     | Yes, via Feedback                        | No                                          | No                                          |
| **Result**                 | Yes, upon completion                     | No                                          | Yes                                         |
| **Asynchronous Execution** | Yes                                      | Yes                                         | Typically synchronous                       |

------

### **Summary**

- Navigation in ROS 2 uses **Actions** (e.g., `NavigateToPose`) for tasks that require long-running execution, real-time feedback, and final results.
- Actions provide:
  - **Goal management:** Define the target destination.
  - **Real-time feedback:** Monitor the progress of the navigation task.
  - **Result handling:** Determine success or failure of the navigation.
- Compared to Topics and Services, Actions are more suited for complex tasks like navigation due to their multi-stage communication model.
