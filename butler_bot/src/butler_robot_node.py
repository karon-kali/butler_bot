#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import time
from enum import Enum
from dataclasses import dataclass
from typing import List, Optional, Dict, Any
import json


class RobotState(Enum):
    IDLE = "idle"
    MOVING_TO_KITCHEN = "moving_to_kitchen"
    AT_KITCHEN = "at_kitchen"
    MOVING_TO_TABLE = "moving_to_table"
    AT_TABLE = "at_table"
    RETURNING_HOME = "returning_home"
    WAITING_CONFIRMATION = "waiting_confirmation"


@dataclass
class Order:
    table_id: str
    status: str = "pending"  # pending, confirmed, delivered, cancelled
    timestamp: float = 0.0


class ButlerRobotNode(Node):
    def __init__(self):
        super().__init__('butler_robot_node')
        
        # Callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()
        
        # Predefined coordinates
        self.coordinates = {
            'home': {'x': 0.22, 'y': 0.15, 'z': 0.0, 'w': 1.57},
            'kitchen': {'x': 0.02, 'y': 1.0, 'z': 0.0, 'w': 0.71},
            'table:1': {'x': -2.43, 'y': 0.01, 'z': 0.0, 'w': 1.0},
            'table:2': {'x': -2.40, 'y': 1.56, 'z': 0.0, 'w': 1.0},
            'table:3': {'x': -2.29, 'y': 3.11, 'z': 0.0, 'w': 1.0}
        }
        
        # Robot state management
        self.current_state = RobotState.IDLE
        self.current_location = 'home'
        self.active_orders: List[Order] = []
        self.current_order_index = 0
        self.task_cancelled = False
        self.waiting_for_confirmation = False
        self.confirmation_timeout = 30.0  # 30 seconds timeout
        self.confirmation_start_time = 0.0
        
        # Threading locks
        self.state_lock = threading.Lock()
        self.orders_lock = threading.Lock()
        
        # ROS2 subscribers and publishers
        self.butler_subscriber = self.create_subscription(
            String,
            '/butler_bot',
            self.butler_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/butler_status',
            10
        )
        
        # Navigation action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # Timers
        self.status_timer = self.create_timer(
            1.0,
            self.status_check_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Butler Robot Node initialized')
        self.publish_status("Butler robot ready and waiting for orders")

    def butler_callback(self, msg):
        """Handle incoming butler commands"""
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received command: {command}')
        
        with self.state_lock:
            if command == "table done":
                self.handle_table_done()
            elif command == "kitchen done":
                self.handle_kitchen_done()
            elif command == "go to base":
                self.handle_go_to_base()
            elif command == "go to kitchen":
                self.handle_go_to_kitchen()
            elif command == "cancel goal":
                self.handle_cancel_goal()
            elif command.startswith("go to table:"):
                table_id = command.split(":")[-1]
                self.handle_table_order(table_id)
            else:
                self.get_logger().warn(f'Unknown command: {command}')

    def handle_table_order(self, table_id: str):
        """Handle new table order"""
        if f'table:{table_id}' not in self.coordinates:
            self.get_logger().error(f'Invalid table ID: {table_id}')
            return
            
        with self.orders_lock:
            # Check if this table already has a pending order
            existing_order = next((order for order in self.active_orders 
                                 if order.table_id == table_id and order.status == "pending"), None)
            
            if not existing_order:
                new_order = Order(table_id=table_id, timestamp=time.time())
                self.active_orders.append(new_order)
                self.get_logger().info(f'Added order for table {table_id}')
        
        # Start processing if robot is idle
        if self.current_state == RobotState.IDLE:
            self.start_order_processing()

    def handle_table_done(self):
        """Handle table done confirmation"""
        if self.current_state == RobotState.AT_TABLE and self.waiting_for_confirmation:
            self.waiting_for_confirmation = False
            self.mark_current_order_delivered()
            self.move_to_next_order_or_return_home()

    def handle_kitchen_done(self):
        """Handle kitchen done confirmation"""
        if self.current_state == RobotState.AT_KITCHEN and self.waiting_for_confirmation:
            self.waiting_for_confirmation = False
            self.confirm_kitchen_pickup()
            self.move_to_first_table()

    def handle_go_to_base(self):
        """Handle manual return to base command"""
        self.clear_all_orders()
        self.navigate_to_location('home')

    def handle_go_to_kitchen(self):
        """Handle manual go to kitchen command"""
        self.navigate_to_location('kitchen')

    def handle_cancel_goal(self):
        """Handle goal cancellation"""
        self.task_cancelled = True
        self.cancel_navigation()
        
        if self.current_state == RobotState.MOVING_TO_TABLE:
            # Return to kitchen first, then home
            self.navigate_to_location('kitchen')
            self.current_state = RobotState.RETURNING_HOME
        elif self.current_state == RobotState.MOVING_TO_KITCHEN:
            # Return directly to home
            self.navigate_to_location('home')
        else:
            self.navigate_to_location('home')

    def start_order_processing(self):
        """Start processing the order queue"""
        with self.orders_lock:
            if not self.active_orders:
                return
                
            self.current_order_index = 0
            self.task_cancelled = False
            
        self.get_logger().info(f'Starting order processing with {len(self.active_orders)} orders')
        self.navigate_to_location('kitchen')

    def navigate_to_location(self, location: str):
        """Navigate to specified location"""
        if location not in self.coordinates:
            self.get_logger().error(f'Unknown location: {location}')
            return False
            
        coords = self.coordinates[location]
        
        # Update state based on destination
        if location == 'kitchen':
            self.current_state = RobotState.MOVING_TO_KITCHEN
        elif location.startswith('table:'):
            self.current_state = RobotState.MOVING_TO_TABLE
        elif location == 'home':
            self.current_state = RobotState.RETURNING_HOME
            
        self.publish_status(f"Navigating to {location}")
        
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = coords['x']
        goal_msg.pose.pose.position.y = coords['y']
        goal_msg.pose.pose.position.z = coords['z']
        goal_msg.pose.pose.orientation.w = coords['w']
        
        # Send navigation goal
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda fut: self.navigation_response_callback(fut, location))
        
        return True

    def navigation_response_callback(self, future, destination: str):
        """Handle navigation response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Navigation to {destination} rejected')
            return
            
        self.get_logger().info(f'Navigation to {destination} accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self.navigation_result_callback(fut, destination))

    def navigation_result_callback(self, future, destination: str):
        """Handle navigation result"""
        result = future.result().result
        
        if self.task_cancelled:
            self.get_logger().info('Task was cancelled, handling cancellation logic')
            return
            
        self.current_location = destination
        self.get_logger().info(f'Arrived at {destination}')
        
        if destination == 'kitchen':
            self.handle_arrival_at_kitchen()
        elif destination.startswith('table:'):
            self.handle_arrival_at_table(destination)
        elif destination == 'home':
            self.handle_arrival_at_home()

    def handle_arrival_at_kitchen(self):
        """Handle arrival at kitchen"""
        self.current_state = RobotState.AT_KITCHEN
        self.waiting_for_confirmation = True
        self.confirmation_start_time = time.time()
        self.publish_status("Arrived at kitchen, waiting for food pickup confirmation")

    def handle_arrival_at_table(self, table_location: str):
        """Handle arrival at table"""
        self.current_state = RobotState.AT_TABLE
        self.waiting_for_confirmation = True
        self.confirmation_start_time = time.time()
        table_id = table_location.split(':')[-1]
        self.publish_status(f"Arrived at table {table_id}, waiting for delivery confirmation")

    def handle_arrival_at_home(self):
        """Handle arrival at home"""
        self.current_state = RobotState.IDLE
        self.waiting_for_confirmation = False
        self.publish_status("Returned to home position, ready for new orders")
        self.clear_completed_orders()

    def confirm_kitchen_pickup(self):
        """Confirm food pickup from kitchen"""
        with self.orders_lock:
            for order in self.active_orders:
                if order.status == "pending":
                    order.status = "confirmed"
        self.get_logger().info('Food pickup confirmed from kitchen')

    def move_to_first_table(self):
        """Move to the first pending table"""
        with self.orders_lock:
            pending_orders = [order for order in self.active_orders if order.status == "confirmed"]
            
        if pending_orders:
            self.current_order_index = 0
            table_location = f'table:{pending_orders[0].table_id}'
            self.navigate_to_location(table_location)
        else:
            self.navigate_to_location('home')

    def mark_current_order_delivered(self):
        """Mark current order as delivered"""
        with self.orders_lock:
            confirmed_orders = [order for order in self.active_orders if order.status == "confirmed"]
            if self.current_order_index < len(confirmed_orders):
                confirmed_orders[self.current_order_index].status = "delivered"
                table_id = confirmed_orders[self.current_order_index].table_id
                self.get_logger().info(f'Order for table {table_id} marked as delivered')

    def move_to_next_order_or_return_home(self):
        """Move to next order or return home if all orders are complete"""
        with self.orders_lock:
            confirmed_orders = [order for order in self.active_orders if order.status == "confirmed"]
            
        self.current_order_index += 1
        
        if self.current_order_index < len(confirmed_orders):
            # Move to next table
            next_table = f'table:{confirmed_orders[self.current_order_index].table_id}'
            self.navigate_to_location(next_table)
        else:
            # Check if any orders were undelivered
            with self.orders_lock:
                undelivered_orders = [order for order in self.active_orders 
                                    if order.status in ["confirmed", "pending"]]
            
            if undelivered_orders:
                # Return to kitchen first for undelivered orders
                self.navigate_to_location('kitchen')
            else:
                # All orders delivered, return home
                self.navigate_to_location('home')

    def status_check_callback(self):
        """Periodic status check for timeouts and order management"""
        if self.waiting_for_confirmation:
            elapsed_time = time.time() - self.confirmation_start_time
            
            if elapsed_time > self.confirmation_timeout:
                self.handle_confirmation_timeout()

    def handle_confirmation_timeout(self):
        """Handle confirmation timeout"""
        self.waiting_for_confirmation = False
        
        if self.current_state == RobotState.AT_KITCHEN:
            self.get_logger().warn('Kitchen confirmation timeout, returning home')
            self.navigate_to_location('home')
            
        elif self.current_state == RobotState.AT_TABLE:
            self.get_logger().warn('Table confirmation timeout, moving to next order or returning via kitchen')
            
            # Skip current table and move to next or return via kitchen
            with self.orders_lock:
                confirmed_orders = [order for order in self.active_orders if order.status == "confirmed"]
                
            self.current_order_index += 1
            
            if self.current_order_index < len(confirmed_orders):
                # Move to next table
                next_table = f'table:{confirmed_orders[self.current_order_index].table_id}'
                self.navigate_to_location(next_table)
            else:
                # No more tables, return via kitchen
                self.navigate_to_location('kitchen')

    def clear_all_orders(self):
        """Clear all active orders"""
        with self.orders_lock:
            self.active_orders.clear()
        self.current_order_index = 0
        self.task_cancelled = False
        self.waiting_for_confirmation = False

    def clear_completed_orders(self):
        """Clear completed orders"""
        with self.orders_lock:
            self.active_orders = [order for order in self.active_orders 
                                if order.status not in ["delivered", "cancelled"]]

    def cancel_navigation(self):
        """Cancel current navigation goal"""
        # Implementation would depend on your navigation stack
        # This is a placeholder for the cancellation logic
        self.get_logger().info('Cancelling current navigation goal')

    def publish_status(self, message: str):
        """Publish status message"""
        status_msg = String()
        status_msg.data = f"[{self.current_state.value}] {message}"
        self.status_publisher.publish(status_msg)
        self.get_logger().info(status_msg.data)


def main(args=None):
    rclpy.init(args=args)
    
    butler_robot = ButlerRobotNode()
    
    # Use MultiThreadedExecutor for concurrent operations
    executor = MultiThreadedExecutor()
    executor.add_node(butler_robot)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        butler_robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()