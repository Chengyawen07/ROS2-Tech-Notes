### **如何在 ROS 2 中使用 Docker 部署节点** ⭐️

使用 Docker 部署 ROS 2 节点是一种保证一致性、可移植性和简化依赖管理的好方法。通过 Docker 容器，您可以将 ROS 2 节点及其依赖项封装在一起，从而在不同环境中无缝运行。

------

### **1. 为什么在 ROS 2 中使用 Docker？**

1. **可移植性：**
   - Docker 确保您的 ROS 2 应用程序可以在任何支持 Docker 的平台上运行。
2. **依赖隔离：**
   - 通过隔离依赖项，避免软件版本冲突。
3. **可复现性：**
   - 使用 Docker 镜像可以创建相同的开发、测试和生产环境。
4. **可扩展性：**
   - 可以轻松将 ROS 2 节点部署到多台机器或云端。

------

### **2. 先决条件**

1. **安装 Docker：**

   - 对于 Linux 系统：

     ```bash
     sudo apt update
     sudo apt install docker.io
     sudo systemctl start docker
     sudo systemctl enable docker
     ```

   - 其他平台请参考 [Docker 官方安装指南](https://docs.docker.com/get-docker/)。

2. **(可选) 安装 Docker Compose：**

   - 用于编排多个容器：

     ```bash
     sudo apt install docker-compose
     ```

3. **将用户添加到 Docker 用户组（可选）：**

   - 避免在运行 Docker 命令时需要使用 sudo

     ```bash
     sudo usermod -aG docker $USER
     ```

   - 重新登录以使更改生效。

4. **安装 ROS 2：**

   - 在主机系统上安装 ROS 2 以验证应用程序在容器化之前是否正常运行。

------

### **3. 编写 ROS 2 的 Dockerfile** 🐱

**Dockerfile** 是用于构建 Docker 镜像的指令文件。以下示例展示了如何为一个 ROS 2 Humble 节点创建 Dockerfile。

#### **示例 Dockerfile：**

```dockerfile
# 使用官方 ROS 2 基础镜像
FROM ros:humble

# 安装额外依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 设置工作目录
WORKDIR /ros2_ws

# 将工作空间文件复制到容器中
COPY . /ros2_ws/

# 构建 ROS 2 工作空间
RUN source /opt/ros/humble/setup.bash && \
    colcon build

# 配置默认运行命令
CMD ["bash", "-c", "source /ros2_ws/install/setup.bash && ros2 run <package_name> <node_name>"]
```

------

### **4. 构建 Docker 镜像**

1. 转到包含 `Dockerfile` 和 ROS 2 工作空间的目录。

2. 构建镜像：

   ```bash
   docker build -t ros2_node_image .
   ```

   - **`-t ros2_node_image`**：将镜像命名为 `ros2_node_image`。

------

### **5. 运行 Docker 容器**

运行容器时，需要为 ROS 2 网络配置合适的参数。

#### **基本运行命令：**

```bash
docker run -it --rm ros2_node_image
```

- **`-it`：** 交互模式运行。
- **`--rm`：** 容器停止后自动删除。

#### **启用多节点通信的网络设置：**

在 Linux 上，可以使用 `--net=host` 选项共享主机网络：

```bash
docker run -it --rm --net=host ros2_node_image
```

对于 macOS 或 Windows，由于不支持 `--net=host`，可以使用 Docker Compose 或手动配置 ROS 2 的 DDS 环境变量。

------

### **6. 使用 Docker Compose 部署多个节点**

如果需要将多个 ROS 2 节点部署在单独的容器中，Docker Compose 可以简化管理。

#### **示例 `docker-compose.yml` 文件：**

```yaml
version: '3.8'

services:
  node1:
    image: ros2_node_image
    container_name: ros2_node1
    network_mode: host
    command: bash -c "source /ros2_ws/install/setup.bash && ros2 run <package_name> <node1_name>"

  node2:
    image: ros2_node_image
    container_name: ros2_node2
    network_mode: host
    command: bash -c "source /ros2_ws/install/setup.bash && ros2 run <package_name> <node2_name>"
```

#### **运行节点：**

```bash
docker-compose up
```

------

### **7. 挂载工作空间以进行开发**

为了在开发过程中无需重新构建镜像，可以将本地工作空间挂载到容器中。

#### **运行时挂载卷：**

```bash
docker run -it --rm \
  -v $(pwd):/ros2_ws \
  ros2_node_image \
  bash -c "source /opt/ros/humble/setup.bash && colcon build && ros2 run <package_name> <node_name>"
```

- **`-v $(pwd):/ros2_ws`**：将当前目录挂载到容器内的 `/ros2_ws`。

------

### **8. ROS 2 中的 Docker 网络配置**

#### **使用 DDS 环境变量配置节点发现：**

如果不使用 `--net=host`，可以通过设置以下环境变量来配置 ROS 2 的 DDS 节点发现：

- **ROS Domain ID：**

  - 确保所有节点使用相同的 Domain ID：

    ```bash
    export ROS_DOMAIN_ID=42
    ```

- **Fast DDS Discovery Server：**

  - 使用共享的发现服务器连接容器：

    ```bash
    docker run -it --rm -e ROS_DISCOVERY_SERVER=<server_ip:port> ros2_node_image
    ```

#### **桥接主机和容器网络：**

为需要的 ROS 2 通信端口暴露容器端口：

```bash
docker run -it --rm -p 7400:7400 -p 7500:7500 ros2_node_image
```

------

### **9. 最佳实践**

1. **使用轻量级基础镜像：**

   - 避免安装不必要的依赖，尽量使用精简版镜像（例如 `ros:humble-slim`）。

2. **减少镜像层：**

   - 将相关命令合并到单个 `RUN` 指令中，减少镜像大小。

3. **多阶段构建：**

   - 使用多阶段构建分离构建环境和运行时环境：

     ```dockerfile
     # 第一阶段：构建阶段
     FROM ros:humble AS builder
     WORKDIR /ros2_ws
     COPY . .
     RUN colcon build
     
     # 第二阶段：运行阶段
     FROM ros:humble
     WORKDIR /ros2_ws
     COPY --from=builder /ros2_ws/install /ros2_ws/install
     CMD ["bash", "-c", "source /ros2_ws/install/setup.bash && ros2 run <package_name> <node_name>"]
     ```

4. **调试容器：**

   - 通过交互式 Shell 进入容器以调试：

     ```bash
     docker run -it --rm ros2_node_image bash
     ```

------

### **10. 总结**

| **步骤**                | **命令/示例**                                    |
| ----------------------- | ------------------------------------------------ |
| **构建镜像**            | `docker build -t ros2_node_image .`              |
| **运行容器**            | `docker run -it --rm --net=host ros2_node_image` |
| **使用 Docker Compose** | `docker-compose up`                              |
| **挂载卷进行开发**      | `docker run -v $(pwd):/ros2_ws ...`              |
| **配置网络**            | 使用 `--net=host` 或 DDS 环境变量配置节点发现    |

通过使用 Docker 对 ROS 2 节点进行容器化，您可以实现高效、可移植且一致的部署流程，适用于开发和生产环境。



# 例子：使用Docker构建和迁移ros2开发环境

目标是将本地 Ubuntu 的 ROS 2 开发环境完全放入 Docker 容器中，之后将容器迁移到 NVIDIA 平台（例如 Jetson 开发板）中开发。

------

### **1. 安装和配置 Docker**

#### **1.1 在本地 Ubuntu 安装 Docker**

```bash
sudo apt update
sudo apt install docker.io
sudo systemctl start docker
sudo systemctl enable docker
```

#### **1.2 将当前用户添加到 Docker 组（可选）**

```bash
sudo usermod -aG docker $USER
```

重新登录以生效，避免每次运行 Docker 都需要 `sudo`。

#### **1.3 在 NVIDIA 平台安装 Docker 和 NVIDIA 容器运行时**

1. 安装 Docker（如上所示）。

2. 安装 NVIDIA Container Toolkit：

   ```bash
   distribution=$(. /etc/os-release;echo $ID$VERSION_ID) && \
   curl -s -L https://nvidia.github.io/nvidia-container-toolkit/gpgkey | sudo apt-key add - && \
   curl -s -L https://nvidia.github.io/nvidia-container-toolkit/$distribution/nvidia-container-toolkit.list | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list && \
   sudo apt update && sudo apt install -y nvidia-container-toolkit && sudo nvidia-ctk runtime configure && sudo systemctl restart docker
   ```

#### **1.4 检查 NVIDIA 容器支持**

运行以下命令，确保 NVIDIA GPU 可以被 Docker 识别：

```bash
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

------

### **2. 准备 ROS 2 Docker 开发环境**

#### **2.1 创建 Dockerfile**

在工作目录中创建一个名为 `Dockerfile` 的文件，用于定义 Docker 镜像的构建过程。

**示例 Dockerfile：**

```dockerfile
# 使用 NVIDIA CUDA 和 ROS 2 Humble 基础镜像
FROM ros:humble

# 安装必要的软件包和工具
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    build-essential \
    nano \
    && rm -rf /var/lib/apt/lists/*

# 安装 NVIDIA 容器运行时支持的工具
RUN apt-get install -y nvidia-container-runtime

# 设置工作空间目录
WORKDIR /ros2_ws

# 拷贝本地代码到容器内（稍后挂载代码会更高效）
COPY . /ros2_ws/

# 构建工作空间
RUN source /opt/ros/humble/setup.bash && colcon build

# 启动命令
CMD ["bash"]
```

------

#### **2.2 构建 Docker 镜像**

在 `Dockerfile` 所在目录中运行以下命令：

```bash
docker build -t ros2_dev_image .
```

- **`-t ros2_dev_image`**：为镜像指定标签名称。

------

### **3. 使用 Docker 开发 ROS 2**

#### **3.1 启动容器并挂载本地代码**

**挂载本地开发代码到容器内，使您无需每次更改代码都重建镜像。**

```bash
docker run -it --rm \
  --gpus all \
  -v $(pwd):/ros2_ws \
  ros2_dev_image \
  bash
```

- **`--gpus all`**：让容器访问 NVIDIA GPU。
- **`-v $(pwd):/ros2_ws`**：将本地当前目录挂载到容器的 `/ros2_ws`。
- **`bash`**：进入容器的交互式 Shell。

#### **3.2 在容器中构建工作空间**

进入容器后，构建 ROS 2 工作空间：

```bash
source /opt/ros/humble/setup.bash
colcon build
```

#### **3.3 运行 ROS 2 节点**

启动构建的节点：

```bash
source /ros2_ws/install/setup.bash
ros2 run <package_name> <node_name>
```

------

### **4. 将容器迁移到 NVIDIA 平台**

#### **4.1 保存镜像为 tar 文件**

将构建好的 Docker 镜像保存为文件：

```bash
docker save ros2_dev_image > ros2_dev_image.tar
```

#### **4.2 将 tar 文件传输到 NVIDIA 平台**

使用 `scp` 或其他工具将文件传输到 NVIDIA 平台：

```bash
scp ros2_dev_image.tar user@nvidia_device_ip:/path/to/destination
```

#### **4.3 在 NVIDIA 平台加载镜像**

在 NVIDIA 平台上加载镜像：

```bash
docker load < ros2_dev_image.tar
```

#### **4.4 运行镜像**

运行容器：

```bash
docker run -it --rm --gpus all ros2_dev_image bash
```

------

### **5. 优化 Docker 开发流程**

#### **5.1 使用 Docker Compose**

通过 Docker Compose 轻松管理多个容器。

**创建 `docker-compose.yml` 文件：**

```yaml
version: '3.8'

services:
  ros2_dev:
    image: ros2_dev_image
    container_name: ros2_dev_container
    network_mode: host
    runtime: nvidia
    volumes:
      - .:/ros2_ws
    command: bash
```

**启动容器：**

```bash
docker-compose up
```

------

#### **5.2 使用 NVIDIA 提供的基础镜像**

如果您的代码需要 CUDA 支持，可以使用 NVIDIA 的基础镜像：

```dockerfile
FROM nvidia/cuda:11.0-base-ubuntu20.04

# 添加 ROS 2 安装
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    apt-add-repository http://packages.ros.org/ros2/ubuntu && \
    apt-get update && apt-get install -y ros-humble-desktop
```

------

### **6. 总结**

| **步骤**               | **命令/说明**                                                |
| ---------------------- | ------------------------------------------------------------ |
| **构建 Docker 镜像**   | `docker build -t ros2_dev_image .`                           |
| **启动容器并挂载代码** | `docker run -it --rm --gpus all -v $(pwd):/ros2_ws ros2_dev_image bash` |
| **保存镜像为文件**     | `docker save ros2_dev_image > ros2_dev_image.tar`            |
| **迁移镜像到 NVIDIA**  | 使用 `scp` 或其他工具传输镜像，并使用 `docker load` 在目标设备加载镜像 |
| **运行容器**           | `docker run -it --rm --gpus all ros2_dev_image bash`         |

通过以上步骤，您可以将 ROS 2 的开发环境完全容器化，并无缝迁移到 NVIDIA 平台进行开发。这样既简化了依赖管理，又能在不同平台上实现一致性。



# 挂载的问题

当将开发环境迁移到 NVIDIA 平台（例如 Jetson）后，由于挂载是基于本地文件系统的，原本挂载到本地 Ubuntu 的文件路径在 NVIDIA 平台上可能不再适用。以下是针对这种情况的处理方法。

------

### **1. 挂载的核心问题**

挂载的本质是**容器访问主机的文件系统**：

- 本地 Ubuntu 上挂载的路径（如 `~/ros2_project`），在 NVIDIA 平台上可能并不存在。
- 在 NVIDIA 平台上，您需要确保相同的文件结构和路径，或者采用其他方法共享和同步代码。

------

### **2. 常见解决方案**

#### **方案 1：在 NVIDIA 平台上重新挂载新路径**

在 NVIDIA 平台上，将开发代码复制到目标设备的某个目录中（比如 `/home/nvidia/ros2_project`），然后挂载这个目录到 Docker 容器。

**步骤：**

1. 将本地 Ubuntu 上的代码复制到 NVIDIA 平台：

   ```bash
   scp -r ~/ros2_project nvidia@<nvidia_device_ip>:/home/nvidia/
   ```

2. 在 NVIDIA 平台上运行容器并挂载代码：

   ```bash
   docker run -it --rm \
     --gpus all \
     -v /home/nvidia/ros2_project:/ros2_ws \
     ros2_dev_image \
     bash
   ```

3. 在容器中构建和运行代码：

   ```bash
   source /opt/ros/humble/setup.bash
   colcon build
   ros2 run <package_name> <node_name>
   ```

------

#### **方案 2：直接将代码打包进 Docker 镜像**

如果您不希望在 NVIDIA 平台上手动挂载代码，可以将本地代码直接打包到 Docker 镜像中。这种方法不需要额外挂载代码，但每次代码更新需要重新构建镜像。

**修改 Dockerfile：**

```dockerfile
# 将代码直接复制到镜像内
COPY . /ros2_ws/

# 在镜像内构建代码
RUN source /opt/ros/humble/setup.bash && colcon build
```

**构建镜像：**

```bash
docker build -t ros2_dev_image .
```

**将镜像迁移到 NVIDIA 平台：**

1. 保存镜像为 tar 文件：

   ```bash
   docker save ros2_dev_image > ros2_dev_image.tar
   ```

2. 将 tar 文件传输到 NVIDIA 平台：

   ```bash
   scp ros2_dev_image.tar nvidia@<nvidia_device_ip>:/home/nvidia/
   ```

3. 在 NVIDIA 平台上加载镜像：

   ```bash
   docker load < ros2_dev_image.tar
   ```

4. 运行容器：

   ```bash
   docker run -it --rm --gpus all ros2_dev_image bash
   ```

------

#### **方案 3：通过网络共享文件**

如果希望本地 Ubuntu 和 NVIDIA 平台同时开发，您可以通过网络共享文件，例如使用 **NFS（网络文件系统）** 或 **Samba**。

**步骤：**

1. **在本地 Ubuntu 设置 NFS 共享：**

   - 安装 NFS：

     ```bash
     sudo apt install nfs-kernel-server
     ```

   - 在 

     ```
     /etc/exports
     ```

      中添加共享路径：

     ```
     /home/user/ros2_project <nvidia_device_ip>(rw,sync,no_subtree_check)
     ```

   - 重启 NFS 服务：

     ```bash
     sudo systemctl restart nfs-kernel-server
     ```

2. **在 NVIDIA 平台挂载 NFS 文件系统：**

   - 安装 NFS 客户端：

     ```bash
     sudo apt install nfs-common
     ```

   - 挂载共享目录：

     ```bash
     sudo mount <ubuntu_host_ip>:/home/user/ros2_project /home/nvidia/ros2_project
     ```

3. **在容器中挂载 NFS 目录：**

   - 启动容器并挂载共享目录：

     ```bash
     docker run -it --rm \
       --gpus all \
       -v /home/nvidia/ros2_project:/ros2_ws \
       ros2_dev_image \
       bash
     ```

**优点：**

- 本地和 NVIDIA 平台实时同步代码，无需手动传输。 **缺点：**
- 需要稳定的网络连接。

------

#### **方案 4：使用代码版本控制（推荐）**

通过 Git 等版本控制工具同步代码，避免复杂的挂载配置。

**步骤：**

1. **将代码推送到远程仓库：**

   - 初始化 Git 仓库：

     ```bash
     cd ~/ros2_project
     git init
     git add .
     git commit -m "Initial commit"
     git remote add origin <your_git_repo_url>
     git push -u origin main
     ```

2. **在 NVIDIA 平台上克隆代码：**

   ```bash
   git clone <your_git_repo_url> /home/nvidia/ros2_project
   ```

3. **运行容器并挂载克隆的代码：**

   ```bash
   docker run -it --rm \
     --gpus all \
     -v /home/nvidia/ros2_project:/ros2_ws \
     ros2_dev_image \
     bash
   ```

4. **本地和远程同步开发：**

   - 在本地修改代码并提交：

     ```bash
     git commit -am "Update code"
     git push
     ```

   - 在 NVIDIA 平台拉取最新代码：

     ```bash
     cd /home/nvidia/ros2_project
     git pull
     ```

------

### **总结：如何处理挂载问题**

| **方法**                        | **特点**                                           | **适用场景**                               |
| ------------------------------- | -------------------------------------------------- | ------------------------------------------ |
| **方案 1：重新挂载新路径**      | 简单直接，但需要手动传输代码和配置挂载路径。       | 代码不经常变动，或迁移后开发本地化的场景。 |
| **方案 2：打包进 Docker 镜像**  | 无需挂载，但每次修改代码需要重新构建镜像。         | 代码较稳定，且对开发实时性要求较低的场景。 |
| **方案 3：使用网络共享文件**    | 本地和远程实时同步，开发效率高，但依赖网络连接。   | 开发频繁、需要实时同步的场景。             |
| **方案 4：使用版本控制（Git）** | 代码版本管理清晰，可轻松同步到多个平台，推荐使用。 | 推荐：适用于任何开发场景。                 |

### **推荐做法**

- **开发阶段：** 使用挂载（方案 1 或 3），实现快速调试。
- **部署阶段：** 将代码打包到 Docker 镜像中（方案 2）。
- **协作开发：** 使用 Git 管理代码版本（方案 4），确保不同平台和开发者之间的同步。
