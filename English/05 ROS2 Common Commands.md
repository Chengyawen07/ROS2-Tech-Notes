### **Common ROS 2 Humble Commands and Their Uses**

Here’s a categorized list of commonly used ROS 2 Humble commands with their purposes.

------

### **1. Environment Commands**

| **Command**                         | **Purpose**                                                  |
| ----------------------------------- | ------------------------------------------------------------ |
| `source /opt/ros/humble/setup.bash` | Load the ROS 2 Humble environment. This enables ROS 2 tools and features (add to `.bashrc` for convenience). |
| `printenv                           | grep ROS`                                                    |

------

### **2. Workspace Commands**

| **Command**                            | **Purpose**                                                  |
| -------------------------------------- | ------------------------------------------------------------ |
| `colcon build`                         | Build all ROS 2 packages in the current workspace.           |
| `colcon build --packages-select <pkg>` | Build only the specified package, e.g., `colcon build --packages-select my_package`. |
| `colcon test`                          | Run the test cases for the current workspace.                |
| `colcon test-result`                   | Display the results of the tests.                            |
| `source install/setup.bash`            | Load the environment of the current workspace after building, enabling custom packages. |

------

### **3. Package Management Commands**

| **Command**                           | **Purpose**                                                  |
| ------------------------------------- | ------------------------------------------------------------ |
| `ros2 pkg create <package_name>`      | Create a new ROS 2 package (supports Python or C++).         |
| `ros2 pkg list`                       | List all available ROS 2 packages in the current environment. |
| `ros2 pkg prefix <package_name>`      | Show the installation path of the specified package.         |
| `ros2 pkg executables <package_name>` | Display the executables available in a package.              |

------

### **4. Node Commands**

| **Command**             | **Purpose**                                                  |
| ----------------------- | ------------------------------------------------------------ |
| `ros2 node list`        | List all active nodes in the current ROS 2 system.           |
| `ros2 node info <node>` | Show information about a specific node, including topics, services, and parameters it uses. |

------

### **5. Topic Commands**

| **Command**                                 | **Purpose**                                                  |
| ------------------------------------------- | ------------------------------------------------------------ |
| `ros2 topic list`                           | List all active topics.                                      |
| `ros2 topic info <topic>`                   | Display information about a specific topic, such as its message type and publishers/subscribers. |
| `ros2 topic echo <topic>`                   | Print real-time data being published to a specific topic.    |
| `ros2 topic pub <topic> <msg_type> '{...}'` | Publish data to a topic, e.g., `ros2 topic pub /example std_msgs/String '{data: "Hello"}'`. |

------

### **6. Service Commands**

| **Command**                                  | **Purpose**                                                  |
| -------------------------------------------- | ------------------------------------------------------------ |
| `ros2 service list`                          | List all active services.                                    |
| `ros2 service info <service>`                | Display information about a specific service.                |
| `ros2 service call <service> <type> '{...}'` | Call a service, e.g., `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts '{a: 5, b: 3}'`. |

------

### **7. Action Commands**

| **Command**                                     | **Purpose**                                                  |
| ----------------------------------------------- | ------------------------------------------------------------ |
| `ros2 action list`                              | List all active actions.                                     |
| `ros2 action info <action>`                     | Display information about a specific action, including goal and result types. |
| `ros2 action send_goal <action> <type> '{...}'` | Send a goal to an action, e.g., `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {...}}'`. |

------

### **8. Parameter Commands**

| **Command**                                 | **Purpose**                                                  |
| ------------------------------------------- | ------------------------------------------------------------ |
| `ros2 param list`                           | List all parameters for active nodes.                        |
| `ros2 param get <node> <parameter>`         | Get the value of a parameter for a specific node, e.g., `ros2 param get /example_node some_parameter`. |
| `ros2 param set <node> <parameter> <value>` | Set the value of a parameter for a specific node, e.g., `ros2 param set /example_node some_parameter 10`. |

------

### **9. Logging and Debugging Commands**

| **Command**                           | **Purpose**                                                  |
| ------------------------------------- | ------------------------------------------------------------ |
| `ros2 run <package> <executable>`     | Run a specific executable from a package, e.g., `ros2 run demo_nodes_cpp talker`. |
| `ros2 launch <package> <launch_file>` | Use a launch file to start multiple nodes, e.g., `ros2 launch nav2_bringup navigation_launch.py`. |
| `ros2 bag record -a`                  | Start recording all topics into a rosbag file.               |
| `ros2 bag play <file>`                | Replay data from a rosbag file, e.g., `ros2 bag play my_bag.db3`. |

------

### **10. Message Type Commands**

| **Command**                | **Purpose**                                                  |
| -------------------------- | ------------------------------------------------------------ |
| `ros2 msg list`            | List all available message types.                            |
| `ros2 msg show <msg_type>` | Display the definition of a specific message type, e.g., `ros2 msg show std_msgs/String`. |

------

### **11. Service/Action Type Commands**

| **Command**                      | **Purpose**                                                  |
| -------------------------------- | ------------------------------------------------------------ |
| `ros2 srv list`                  | List all available service types.                            |
| `ros2 srv show <srv_type>`       | Display the definition of a specific service type, e.g., `ros2 srv show example_interfaces/srv/AddTwoInts`. |
| `ros2 action list`               | List all available action types.                             |
| `ros2 action show <action_type>` | Display the definition of a specific action type, e.g., `ros2 action show nav2_msgs/action/NavigateToPose`. |

------

### **Summary**

- **Environment Commands:** Ensure the ROS 2 environment is correctly sourced.
- **Workspace Commands:** Manage and build ROS 2 workspaces and packages.
- **Node, Topic, and Service Commands:** Debug and interact with ROS 2 nodes, topics, services, and parameters.
- **Logging and Data Recording:** Use commands like `ros2 bag` for logging and debugging.

These commands are essential tools for development and debugging in ROS 2 Humble. Mastering them will greatly improve your efficiency when working on ROS 2 projects.
