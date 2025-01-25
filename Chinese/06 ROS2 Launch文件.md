### **什么是 Launch 文件？**

**Launch 文件** 是 ROS 2 中用于启动、配置和管理一个或多个节点的工具。它通过定义节点、参数、环境变量等信息，简化了复杂系统的启动过程。

**功能特点：**

1. **启动多个节点：** 一次启动多个节点，无需逐个命令运行。
2. **配置参数：** 在启动节点时传递参数（如话题名称、帧 ID 等）。
3. **设置环境变量：** 修改环境变量以适应特定运行需求。
4. **灵活性：** 支持条件分支、事件触发和动态参数传递。

------

### **ROS 2 中 Launch 文件的特点**

- ROS 2 的 Launch 文件使用 **Python 脚本** 而不是 ROS 1 的 XML 文件。
- 提供模块化和动态功能，可以在运行时根据条件配置启动行为。
- 使用 **`launch_ros`** 和 **`launch`** 库。

------

### **如何创建和使用 Launch 文件启动多个节点？**

以下是一个使用 ROS 2 的 Launch 文件启动多个节点的完整过程。

------

#### **步骤 1：创建 Launch 文件**

1. 在包目录下创建 

   ```
   launch
   ```

    文件夹：

   ```bash
   mkdir -p <package_name>/launch
   ```

2. 创建一个新的 Launch 文件，例如 

   ```
   my_launch.py
   touch <package_name>/launch/my_launch.py
   ```

   

------

#### **步骤 2：编写 Launch 文件**

下面是一个示例 Launch 文件，**用于启动两个节点：一个 `talker` 节点和一个 `listener` 节点。**

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',  # 节点所属的包
            executable='talker',       # 可执行文件
            name='talker_node',        # 节点名称
            output='screen'            # 日志输出到屏幕
        ),
        Node(
            package='demo_nodes_py',   # 节点所属的包
            executable='listener',    # 可执行文件
            name='listener_node',     # 节点名称
            output='screen'           # 日志输出到屏幕
        ),
    ])
```

**代码解释：**

1. `Node`:

    定义要启动的节点，包括：

   - **`package`:** 节点所在的包。
   - **`executable`:** 节点的可执行文件名称。
   - **`name`:** （可选）节点的名称。
   - **`output`:** 指定日志输出位置（如 `screen` 或 `log`）。

2. **`LaunchDescription`:** 包含所有启动的节点。

3. 在 ROS 2 的 launch 文件中，**可以通过在 `LaunchDescription` 中添加多个 `Node` 实例来启动多个节点。**例如，以上例子中已经启动了两个节点 talker 和 `listener`。你可以继续添加更多节点来启动多个节点。

------

#### **步骤 3：运行 Launch 文件**

在终端运行 Launch 文件：

```bash
ros2 launch <package_name> my_launch.py
```

------

### **添加参数和配置**

#### **1. 节点参数**

在启动节点时，可以通过 Launch 文件传递参数。例如：

```python
Node(
    package='demo_nodes_cpp',
    executable='talker',
    name='talker_node',
    output='screen',
    parameters=[
        {'publish_rate': 10.0}  # 设置节点参数
    ]
)
```

#### **2. 使用 Launch 参数**

可以在运行时传递参数并在 Launch 文件中使用：

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rate', default_value='5.0', description='Publish rate'),
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker_node',
            output='screen',
            parameters=[
                {'publish_rate': LaunchConfiguration('rate')}  # 使用运行时参数
            ]
        ),
    ])
```

运行时指定参数：

```bash
ros2 launch <package_name> my_launch.py rate:=10.0
```

------

### **启动条件和事件**

#### **1. 条件启动**

可以基于条件决定是否启动某个节点。例如：

```python
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

Node(
    package='demo_nodes_cpp',
    executable='talker',
    name='conditional_talker',
    output='screen',
    condition=IfCondition(LaunchConfiguration('enable_talker'))  # 只有在参数 enable_talker 为 true 时启动
)
```

运行时指定条件：

```bash
ros2 launch <package_name> my_launch.py enable_talker:=true
```

------

### **完整示例：复杂系统的 Launch 文件**

以下是一个完整示例，启动多个节点并设置参数和条件。

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_talker', default_value='true', description='Enable the talker node'),
        DeclareLaunchArgument('talker_rate', default_value='5.0', description='Talker publish rate'),

        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker_node',
            output='screen',
            parameters=[{'publish_rate': LaunchConfiguration('talker_rate')}],
            condition=IfCondition(LaunchConfiguration('enable_talker'))  # 条件启动
        ),
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='listener_node',
            output='screen'
        ),
    ])
```

运行时可以动态指定参数和条件：

```bash
ros2 launch <package_name> my_launch.py enable_talker:=false talker_rate:=10.0
```

------

### **总结**

1. **Launch 文件的核心功能：**
   - 一次启动多个节点。
   - 动态传递参数和配置节点。
   - 支持条件启动和复杂任务管理。
2. **编写步骤：**
   - 使用 Python 编写 Launch 文件。
   - 使用 `Node` 定义节点。
   - 使用 `LaunchDescription` 组织节点。
3. **运行方式：**
   - 直接使用 `ros2 launch` 命令启动。

通过 Launch 文件，ROS 2 的系统启动变得高效和灵活，尤其适用于复杂的机器人系统。
