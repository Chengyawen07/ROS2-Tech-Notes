### **How to Record and Replay Data in ROS 2 (Bag Files)**

**Bag files** in ROS 2 are used to record topic data (e.g., sensor readings, positional data, commands) and replay them later. This functionality is useful for debugging, algorithm testing, and running simulations without hardware.

------

### **1. Recording Data**

#### **1.1 Using `ros2 bag record` Command**

The `ros2 bag record` command is used to record data from one or more topics.

##### **Basic Command**

```bash
ros2 bag record -o <bag_name> <topic1> <topic2> ...
```

##### **Example: Record a Single Topic**

```bash
ros2 bag record -o my_bag /example_topic
```

- **`-o my_bag`**: Specifies the output folder for the recorded data.
- **`/example_topic`**: The topic to be recorded.

##### **Example: Record All Topics**

```bash
ros2 bag record -a
```

- **`-a`**: Records all active topics.

##### **Example: Record Topics Matching a Prefix**

```bash
ros2 bag record -e "/camera/.*"
```

- **`-e`**: Enables regular expression matching for topic names.
- This command records all topics that start with `/camera/`.

------

#### **1.2 Checking Active Topics**

Before recording, you can list all active topics in the system:

```bash
ros2 topic list
```

**Sample Output:**

```
/example_topic
/camera/image_raw
/robot/odometry
```

------

### **2. Replaying Data**

#### **2.1 Using `ros2 bag play` Command**

The `ros2 bag play` command is used to replay previously recorded bag files.

##### **Basic Command**

```bash
ros2 bag play <bag_folder>
```

##### **Example: Replay a Bag File**

```bash
ros2 bag play my_bag
```

- **`my_bag`**: The folder containing the recorded bag file.
- Topics in the bag file will be republished at their original timestamps.

------

#### **2.2 Controlling Playback Behavior**

##### **Adjust Playback Speed**

Use the `--rate` option to control playback speed:

```bash
ros2 bag play my_bag --rate 2.0
```

- **`--rate 2.0`**: Plays at 2x speed.
- **`--rate 0.5`**: Plays at half speed (slower).

##### **Replay Specific Topics**

Use the `--topics` option to replay only selected topics:

```bash
ros2 bag play my_bag --topics /example_topic
```

##### **Loop Playback**

Use the `--loop` option to replay the bag file in a continuous loop:

```bash
ros2 bag play my_bag --loop
```

------

### **3. Inspecting Bag Files**

#### **3.1 Viewing Bag File Information**

Use `ros2 bag info` to inspect the contents and details of a bag file:

```bash
ros2 bag info my_bag
```

**Sample Output:**

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

### **4. Workflow Example**

#### **Step 1: Record Data**

Suppose you want to record `/camera/image_raw` and `/robot/odometry` topics:

```bash
ros2 bag record -o robot_data /camera/image_raw /robot/odometry
```

This will create a folder named `robot_data` containing the recorded bag file.

#### **Step 2: Inspect Bag File**

Check the recorded file's details:

```bash
ros2 bag info robot_data
```

#### **Step 3: Replay Data**

Replay the recorded topics:

```bash
ros2 bag play robot_data
```

#### **Step 4: Adjust Playback Rate**

Replay at half speed:

```bash
ros2 bag play robot_data --rate 0.5
```

------

### **5. Bag File Format**

ROS 2 bag files are stored in **SQLite3 format** by default:

- Bag files usually end with `.db3`.
- The folder may contain multiple `.db3` files along with metadata.

------

### **6. Advanced Features**

#### **6.1 Filtered Playback**

Replay only specific topics using regular expressions:

```bash
ros2 bag play my_bag --topics /camera/.* /robot/odometry
```

#### **6.2 Combining with Other Tools**

- **Visualization in `rviz2`:** Replay bag data and visualize it in real-time using `rviz2`.
- **Algorithm Testing:** Use bag data instead of live sensor data to test algorithms offline.

------

### **Summary**

| **Command**                     | **Description**                                              |
| ------------------------------- | ------------------------------------------------------------ |
| `ros2 bag record -o <bag_name>` | Record data from specified topics into a bag file.           |
| `ros2 bag record -a`            | Record all active topics.                                    |
| `ros2 bag play <bag_folder>`    | Replay data from a bag file and republish recorded topics.   |
| `ros2 bag info <bag_folder>`    | Inspect the details of a bag file (e.g., duration, message count, topics). |
| `ros2 bag play --rate <rate>`   | Adjust playback speed (e.g., speed up or slow down).         |
| `ros2 bag play --loop`          | Replay the bag file in a continuous loop.                    |

------

### **Common Use Cases**

1. **Data Recording for Debugging:**
   - Save sensor data, trajectory information, etc., for later analysis.
2. **Algorithm Testing:**
   - Replay bag files to test algorithms offline without needing hardware.
3. **Simulation:**
   - Replay recorded data to simulate a live robot environment.

Bag files are an essential tool in ROS 2 for debugging, testing, and optimizing robotic systems. By mastering their use, you can efficiently record, analyze, and replay system data.
