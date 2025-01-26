### **如何在 ROS 2 中记录和回放数据（Bag 文件）**

**Bag 文件** 是 ROS 2 中的一种数据记录工具，用于存储话题数据（如传感器数据、位置信息、命令等），并可以在之后回放这些数据。它通常用于调试、测试算法以及在没有硬件的情况下运行仿真。

------

### **1. 数据记录**

#### <u>**1.1 使用 `ros2 bag record` 命令**</u>

通<u>过 `ros2 bag record` 命令，可以记录一个或多个话题的数据。</u>

##### **基本命令**

```bash
ros2 bag record -o <bag_name> <topic1> <topic2> ...
```

##### **示例：记录一个话题**

```bash
ros2 bag record -o my_bag /example_topic
```

- **`-o my_bag`**：指定输出文件夹为 `my_bag`。
- **`/example_topic`**：记录的话题名称。

##### **示例：记录所有话题**

```bash
ros2 bag record -a
```

- **`-a`**：记录所有活动的话题。

##### **示例：记录带有特定前缀的话题**

```bash
ros2 bag record -e "/camera/.*"
```

- **`-e`**：使用正则表达式匹配话题，例如记录所有以 `/camera/` 开头的话题。

------

#### **1.2 检查话题**

在记录前，可以使用以下命令检查当前系统中所有活动的话题：

```bash
ros2 topic list
```

**示例输出：**

```
/example_topic
/camera/image_raw
/robot/odometry
```

------

### **2. 数据回放**

#### **2.1 使用 `ros2 bag play` 命令**

<u>通过 `ros2 bag play`，可以回放之前记录的 Bag 文件。</u>

##### **基本命令**

```bash
ros2 bag play <bag_folder>
```

##### **示例：回放 Bag 文件**

```bash
ros2 bag play my_bag
```

- **`my_bag`**：是包含 Bag 文件的文件夹。
- 回放时，Bag 文件中记录的话题会在原始时间戳下重新发布。

------

#### **2.2 控制回放行为**

##### **更改回放速率**

使用 `--rate` 参数调整回放速度：

```bash
ros2 bag play my_bag --rate 2.0
```

- **`--rate 2.0`**：以 2 倍速率回放。
- **`--rate 0.5`**：以 0.5 倍速率（减速）回放。

##### **回放特定话题**

使用 `--topics` 参数指定回放的话题：

```bash
ros2 bag play my_bag --topics /example_topic
```

##### **循环回放**

使用 `--loop` 参数使回放循环进行：

```bash
ros2 bag play my_bag --loop
```

------

### **3. 检查 Bag 文件**

#### **3.1 查看 Bag 文件信息**

使用 `ros2 bag info` 查看 Bag 文件的详细信息：

```bash
ros2 bag info my_bag
```

**示例输出：**

```
Files:             my_bag.db3
Bag size:          2.3 MB
Duration:          10.5s
Start:             Mar 03 2025 12:34:56.123 (1677845696.123)
End:               Mar 03 2025 12:35:06.623 (1677845706.623)
Messages:          500
Topic information:
    Topic: /example_topic  | Type: std_msgs/msg/String | Count: 100
    Topic: /camera/image   | Type: sensor_msgs/msg/Image | Count: 300
```

------

### **4. 示例：完整工作流程**

#### **步骤 1：记录数据**

假设需要记录 `/camera/image_raw` 和 `/robot/odometry` 两个话题：

```bash
ros2 bag record -o robot_data /camera/image_raw /robot/odometry
```

运行后会生成一个名为 `robot_data` 的文件夹，其中包含记录的 Bag 文件。

#### **步骤 2：检查 Bag 文件**

```bash
ros2 bag info robot_data
```

#### **步骤 3：回放数据**

回放数据以重新发布 `/camera/image_raw` 和 `/robot/odometry`：

```bash
ros2 bag play robot_data
```

#### **步骤 4：调整回放速率**

以 0.5 倍速回放：

```bash
ros2 bag play robot_data --rate 0.5
```

------

### **5. Bag 文件格式**

ROS 2 默认使用 **SQLite3 格式** 存储 Bag 文件。

- Bag 文件通常以 `.db3` 结尾。
- 文件夹中会包含一个或多个 `.db3` 文件和元数据。

------

### **6. 高级功能**

#### **6.1 使用过滤器回放**

可以通过正则表达式选择性回放特定的话题：

```bash
ros2 bag play my_bag --topics /camera/.* /robot/odometry
```

#### **6.2 与其他工具结合**

- **与 `rviz2` 配合：** 回放 Bag 文件时，可以用 `rviz2` 实时可视化数据。
- **与算法测试结合：** 使用 Bag 文件的回放数据代替实时传感器数据进行算法调试。

------

### **总结**

| **命令**                        | **功能**                                                    |
| ------------------------------- | ----------------------------------------------------------- |
| `ros2 bag record -o <bag_name>` | 记录指定话题的数据到一个 Bag 文件中。                       |
| `ros2 bag record -a`            | 记录所有活动话题的数据。                                    |
| `ros2 bag play <bag_folder>`    | 回放 Bag 文件，并重新发布记录的数据。                       |
| `ros2 bag info <bag_folder>`    | 查看 Bag 文件的详细信息（如时长、消息数量、包含的话题等）。 |
| `ros2 bag play --rate <rate>`   | 调整回放速度（例如加速或减速）。                            |
| `ros2 bag play --loop`          | 循环回放 Bag 文件中的数据。                                 |

**典型应用场景：**

1. **记录机器人运行数据：** 保存传感器数据、路径信息等。
2. **算法开发与测试：** 回放 Bag 文件中的数据以离线调试算法。
3. **仿真测试：** 在没有硬件的情况下使用回放数据进行仿真。

通过使用 Bag 文件，您可以方便地记录和回放 ROS 2 数据，从而有效支持机器人系统的开发、调试和优化。
