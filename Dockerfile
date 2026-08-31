# 使用官方ROS2 Humble镜像作为基础（支持ARM64）
FROM ros:humble-ros-base

# 设置环境变量，避免apt-get安装过程中的交互式提示
ENV DEBIAN_FRONTEND=noninteractive

# 更新软件源并安装基础开发工具
RUN apt-get update && apt-get install -y \
    # 编译工具链
    build-essential \
    cmake \
    git \
    wget \
    vim \
    # Python相关
    python3-pip \
    python3-dev \
    python3-venv \
    # colcon构建工具
    python3-colcon-common-extensions \
    # 其他常用工具
    curl \
    && rm -rf /var/lib/apt/lists/*

# 安装MuJoCo（Python版本）
RUN pip3 install mujoco

# 设置ROS2环境变量（每次进入容器自动生效）
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 设置工作目录
WORKDIR /workspace

# 恢复交互式提示
ENV DEBIAN_FRONTEND=