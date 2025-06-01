# Butler Bot 🤖

A ROS2-based autonomous restaurant service robot designed to handle food delivery tasks between kitchen, tables, and home positions with intelligent order management and timeout handling.

## Overview

Butler Bot is an intelligent service robot that automates food delivery in restaurant environments. The robot efficiently manages single and multiple orders, handles confirmation timeouts, supports order cancellations, and ensures reliable service delivery through sophisticated state management.

## Features

### Core Functionality
- **Autonomous Navigation**: Seamless movement between home, kitchen, and table positions
- **Order Management**: Handles single and multiple order scenarios
- **Timeout Handling**: Intelligent waiting with configurable timeouts for confirmations
- **Order Cancellation**: Dynamic order cancellation support during task execution
- **Multi-Table Delivery**: Optimized routing for multiple table deliveries
- **Web Interface**: Browser-based control and monitoring interface


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





