# Butler Bot 🤖

A ROS2-based autonomous restaurant service robot designed to handle food delivery tasks between kitchen, tables, and home positions with intelligent order management and timeout handling.

## Overview

Butler Bot is an intelligent service robot that automates food delivery in restaurant environments. The robot efficiently manages single and multiple orders, handles confirmation timeouts, supports order cancellations, and ensures reliable service delivery through sophisticated state management.

## Demo video and pics drive link
```bash
https://drive.google.com/drive/folders/1jYyIpZtXCbAJ1UdxkCdwueSHOY1JRr3Q?usp=sharing
```

## Features

### Core Functionality
- **Autonomous Navigation**: Seamless movement between home, kitchen, and table positions
- **Order Management**: Handles single and multiple order scenarios
- **Timeout Handling**: Intelligent waiting with configurable timeouts for confirmations
- **Order Cancellation**: Dynamic order cancellation support during task execution
- **Multi-Table Delivery**: Optimized routing for multiple table deliveries
- **Web Interface**: Browser-based control and monitoring interface

### Implementation Details
This project consists of several key components that work together to create a complete restaurant service robot system:

## 1. Custom Gazebo Simulation Environment

- Built a custom simulation environment in Gazebo to replicate a restaurant floor layout

## 2. ROS2-Supported Flask Web Server

- Developed a Flask-based web server with integrated ROS2 communication
- Features a complete web UI that simulates the restaurant interface
- Allows real-time order management and robot status monitoring through the browser

## 3. Integrated Launch System

- Created a comprehensive ROS2 package with a custom launch file
- Single command launch file (custom_bringup_launch.py) starts all required components
- Eliminates the need for multiple terminal sessions and complex startup procedures
- Ensures proper initialization order and dependency management

## 4. Control Node Bridge

- Built a dedicated control node that serves as the communication bridge
- Interfaces between the Flask web server and Navigation2 (Nav2) stack
- Implements core control logic for order processing and task management
- Handles state transitions and coordinates robot movements between locations

## 5. Custom Behavior Tree System

- Developed a custom behavior tree executor node for advanced task planning
- Created custom behavior tree nodes tailored for restaurant service scenarios
- Implements complex decision-making logic for handling the 7 different operational scenarios
- this is still incomplete
  

## Supported Scenarios

### 1. Basic Delivery
Robot moves from home → kitchen → table → home with no confirmations required.

### 2. Confirmation with Timeout
Robot waits for confirmations at kitchen/table and returns home after timeout if no response.

### 3. Kitchen and Table Confirmations
- Kitchen timeout: Return home immediately
- Table timeout after kitchen pickup: Return to kitchen first, then home

### 4. Order Cancellation
- Cancelled while going to table: Return to kitchen, then home
- Cancelled while going to kitchen: Return home directly

### 5. Multiple Order Delivery
Collect all orders from kitchen and deliver to multiple tables in sequence.

### 6. Multi-Table with Timeout
Skip unresponsive tables, continue delivery sequence, return to kitchen before going home.

### 7. Multi-Table with Cancellation
Skip cancelled orders, deliver remaining orders, return to kitchen before going home.

## Installation

### Prerequisites
- ROS2 - humble
- NAV2
- python flask

### Setup
1. **Clone the repository**
   ```bash
   cd ~/your_ros2_workspace/src
   git clone <repository-url> butler_bot
   ```

2. **Build the package**
   ```bash
   cd ~/your_ros2_workspace
   colcon build --packages-select butler_bot
   ```

3. **Source the workspace**
   ```bash
   source install/setup.bash
   ```

## Usage

### Starting the System
```bash
ros2 launch butler_bot custom_bringup_launch.py
```

### Web Interface
1. Open your browser and navigate to: `http://localhost:5000/`
2. Use the web interface to:
   - Submit new orders with table numbers
   - Monitor robot status
   - Cancel active orders





