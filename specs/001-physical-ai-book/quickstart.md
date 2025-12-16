# Quickstart Guide: Physical AI & Humanoid Robotics Book

**Date**: December 15, 2025  
**Feature**: 001-physical-ai-book

## Overview

This guide provides a rapid pathway for readers to set up their development environment and begin working with the concepts and code examples from the Physical AI & Humanoid Robotics book. Follow these steps to get started with the book's content in 15-30 minutes.

## Prerequisites

Before starting with the book content, ensure you have:

- **Operating System**: Ubuntu 22.04 LTS (recommended) or Windows 10/11 with WSL2
- **Hardware**: Minimum 8GB RAM, 50GB free disk space (16GB+ RAM recommended for simulation)
- **Internet Connection**: Required for initial package installation
- **Basic Knowledge**: Python programming experience, familiarity with command line

## System Setup

### Option 1: Native Ubuntu (Recommended)

1. **Install Ubuntu 22.04 LTS** (fresh installation or dual boot)
   - Download from https://ubuntu.com/download/desktop
   - Minimum 8GB RAM, 50GB disk space

2. **Update system packages**:
   ```bash
   sudo apt update && sudo apt upgrade -y
   ```

3. **Install Python and basic tools**:
   ```bash
   sudo apt install python3 python3-pip python3-venv build-essential curl wget git
   ```

### Option 2: Windows with WSL2

1. **Enable WSL2**:
   ```powershell
   wsl --install
   # Or if WSL is already installed:
   wsl --set-default-version 2
   ```

2. **Install Ubuntu 22.04 from Microsoft Store**:
   - Search for "Ubuntu 22.04 LTS" in Microsoft Store
   - Launch and create your user account

3. **Proceed with the Ubuntu installation steps above**

## ROS 2 Installation

### Install ROS 2 Humble Hawksbill

1. **Set up locale**:
   ```bash
   locale-gen en_US.UTF-8
   export LANG=en_US.UTF-8
   ```

2. **Set up sources**:
   ```bash
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

3. **Install ROS 2 packages**:
   ```bash
   sudo apt update
   sudo apt install -y ros-humble-desktop ros-humble-ros-base
   sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
   ```

4. **Initialize rosdep**:
   ```bash
   sudo rosdep init
   rosdep update
   ```

5. **Source ROS 2 environment**:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## Workspace Setup

1. **Create a ROS 2 workspace**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

2. **Create an additional source directory for book examples**:
   ```bash
   mkdir -p ~/book_examples/src
   cd ~/book_examples
   colcon build
   ```

## Python Environment Setup

1. **Create a virtual environment for book work**:
   ```bash
   python3 -m venv ~/book_env
   source ~/book_env/bin/activate
   ```

2. **Install required Python packages**:
   ```bash
   pip install --upgrade pip
   pip install rclpy transforms3d numpy matplotlib scipy
   pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
   pip install openai langchain-community langchain-openai
   pip install SpeechRecognition pyaudio whisper
   ```

3. **(Optional) Add virtual environment activation to shell**:
   ```bash
   echo "alias bookenv='source ~/book_env/bin/activate'" >> ~/.bashrc
   ```

## Simulation Environment Setup

### Install Gazebo

1. **Install Gazebo Classic**:
   ```bash
   sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-gazebo-ros2-control-demos
   ```

2. **Test Gazebo installation**:
   ```bash
   gazebo
   ```

### Install RViz2
```bash
sudo apt install -y ros-humble-rviz2
```

## First Example: Hello World Publisher

1. **Create your first package**:
   ```bash
   cd ~/book_examples/src
   ros2 pkg create --build-type ament_python hello_publisher
   cd hello_publisher
   ```

2. **Create a simple publisher** in `hello_publisher/hello_publisher/publisher_member_function.py`:
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class MinimalPublisher(Node):
       def __init__(self):
           super().__init__('minimal_publisher')
           self.publisher_ = self.create_publisher(String, 'topic', 10)
           timer_period = 0.5  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.i = 0

       def timer_callback(self):
           msg = String()
           msg.data = f'Hello World: {self.i}' % self.i
           self.publisher_.publish(msg)
           self.get_logger().info(f'Publishing: "{msg.data}"')
           self.i += 1

   def main(args=None):
       rclpy.init(args=args)
       minimal_publisher = MinimalPublisher()
       rclpy.spin(minimal_publisher)
       minimal_publisher.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. **Test your publisher**:
   ```bash
   cd ~/book_examples
   colcon build --packages-select hello_publisher
   source install/setup.bash
   ros2 run hello_publisher publisher_member_function
   ```

4. **In another terminal, verify messages**:
   ```bash
   source ~/book_examples/install/setup.bash
   ros2 topic echo /topic std_msgs/msg/String
   ```

## Book Code Examples Setup

1. **Clone the book's code repository** (when available):
   ```bash
   git clone https://github.com/[author]/physical-ai-book-examples.git ~/book_code_examples
   cd ~/book_code_examples
   ```

2. **Set up the examples workspace**:
   ```bash
   cd ~/book_code_examples
   colcon build
   source install/setup.bash
   ```

## Verification Checklist

Complete this checklist to ensure your environment is properly set up:

- [ ] ROS 2 Humble is installed and sourced
- [ ] `ros2 topic list` runs without errors
- [ ] Gazebo launches successfully
- [ ] RViz2 launches successfully
- [ ] Python virtual environment is working
- [ ] Basic publisher example runs
- [ ] System has adequate RAM for simulation (8GB+)

## Troubleshooting

### Common Issues

**ROS 2 Environment Not Sourced**
- Problem: `ros2: command not found`
- Solution: `source /opt/ros/humble/setup.bash`

**Gazebo Fails to Launch**
- Problem: Display or graphics errors
- Solution: Ensure X11 forwarding if using SSH, or install proper graphics drivers

**Python Package Installation Fails**
- Problem: Permission or dependency errors
- Solution: Ensure you're in the virtual environment (`source ~/book_env/bin/activate`)

**Workspace Building Fails**
- Problem: Missing dependencies
- Solution: Run `rosdep install --from-paths src --ignore-src -r -y` in workspace

## Next Steps

After completing this quickstart guide, you're ready to:

1. **Begin with Chapter 1** of the book
2. **Explore the ROS 2 tutorials** at https://docs.ros.org/en/humble/Tutorials.html
3. **Set up your first simulation** from Chapter 3
4. **Join the community forum** for questions and support

Your development environment is now properly configured to work through all examples and exercises in the Physical AI & Humanoid Robotics book. All code examples are designed to work in this environment, and the book provides detailed explanations for each step of the learning process.