#!/usr/bin/env python3

from flask import Flask, render_template, request, jsonify
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

class RestaurantROSNode(Node):
    def __init__(self):
        super().__init__('restaurant_server_node')
        
        # Publisher for butler goal commands
        self.butler_goal_pub = self.create_publisher(String, '/butler_bot', 10)
        
        self.get_logger().info('Restaurant ROS2 node initialized')
    
    def publish_butler_goal(self, message):
        """Publish message to /butler_bot topic"""
        msg = String()
        msg.data = message
        self.butler_goal_pub.publish(msg)
        self.get_logger().info(f'Published to /butler_bot: {message}')

# Initialize Flask app
app = Flask(__name__)
ros_node = None

def init_ros():
    """Initialize ROS2 node in separate thread"""
    global ros_node
    rclpy.init()
    ros_node = RestaurantROSNode()
    
    # Spin in separate thread
    def spin_node():
        rclpy.spin(ros_node)
    
    ros_thread = threading.Thread(target=spin_node, daemon=True)
    ros_thread.start()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/done', methods=['POST'])
def table_done():
    """Handle done button press - publishes 'go to base' to /butler_goal"""
    if ros_node:
        ros_node.publish_butler_goal('table done')
        return jsonify({'success': True, 'message': 'Going to base'})
    
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

@app.route('/api/go_to_base', methods=['POST'])
def go_to_base():
    """Handle go to base button - publishes 'go to base' to /butler_goal"""
    if ros_node:
        ros_node.publish_butler_goal('go to base')
        return jsonify({'success': True, 'message': 'Going to base'})
    
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

@app.route('/api/go_to_kitchen', methods=['POST'])
def go_to_kitchen():
    """Handle go to kitchen button - publishes 'go to kitchen' to /butler_goal"""
    if ros_node:
        ros_node.publish_butler_goal('go to kitchen')
        return jsonify({'success': True, 'message': 'Going to kitchen'})
    
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

@app.route('/api/go_to_table', methods=['POST'])
def go_to_table():
    """Handle go to table button - publishes 'go to table:number' to /butler_goal"""
    data = request.json
    table_number = data.get('table_number')
    
    if ros_node and table_number:
        try:
            # Validate table number
            table_num = int(table_number)
            ros_node.publish_butler_goal(f'go to table:{table_num}')
            return jsonify({'success': True, 'message': f'Going to table {table_num}'})
        except ValueError:
            return jsonify({'success': False, 'message': 'Invalid table number'})
    
    return jsonify({'success': False, 'message': 'Table number required'})

@app.route('/api/cancel', methods=['POST'])
def cancel_goal():
    """Handle cancel button - publishes 'cancel goal' to /butler_goal"""
    if ros_node:
        ros_node.publish_butler_goal('cancel goal')
        return jsonify({'success': True, 'message': 'Goal cancelled'})
    
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

@app.route('/api/kitchen_done', methods=['POST'])
def kitchen_done():
    """Handle Kitchen Done button - publishes 'kitchen done' to /butler_bot"""
    if ros_node:
        ros_node.publish_butler_goal('kitchen done')
        return jsonify({'success': True, 'message': 'Kitchen marked as done'})
    return jsonify({'success': False, 'message': 'ROS node not initialized'})


# CSS Route
@app.route('/static/style.css')
def serve_css():
    css_content = """
    * {
        margin: 0;
        padding: 0;
        box-sizing: border-box;
    }

    body {
        font-family: 'Arial', sans-serif;
        background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
        min-height: 100vh;
        padding: 20px;
    }

    .container {
        max-width: 1200px;
        margin: 0 auto;
        background: rgba(255, 255, 255, 0.95);
        border-radius: 20px;
        padding: 30px;
        box-shadow: 0 20px 40px rgba(0,0,0,0.1);
    }

    h1 {
        text-align: center;
        color: #333;
        margin-bottom: 30px;
        font-size: 2.5em;
        text-shadow: 2px 2px 4px rgba(0,0,0,0.1);
    }

    .sections {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
        gap: 25px;
        margin-top: 20px;
    }

    .section {
        background: white;
        border-radius: 15px;
        padding: 25px;
        box-shadow: 0 10px 25px rgba(0,0,0,0.1);
        border: 3px solid transparent;
        transition: all 0.3s ease;
        text-align: center;
    }

    .section:hover {
        transform: translateY(-5px);
        box-shadow: 0 15px 35px rgba(0,0,0,0.15);
    }

    .table-section {
        border-color: #4CAF50;
    }

    .kitchen-section {
        border-color: #FF9800;
    }

    .section h2 {
        color: #333;
        margin-bottom: 30px;
        font-size: 1.8em;
    }

    .btn {
        width: 100%;
        padding: 15px 20px;
        border: none;
        border-radius: 8px;
        font-size: 18px;
        font-weight: bold;
        cursor: pointer;
        transition: all 0.3s ease;
        text-transform: uppercase;
        letter-spacing: 1px;
        margin-bottom: 15px;
    }

    .btn:hover {
        transform: translateY(-2px);
        box-shadow: 0 5px 15px rgba(0,0,0,0.2);
    }

    .btn-done {
        background: linear-gradient(45deg, #4CAF50, #45a049);
        color: white;
        font-size: 24px;
        padding: 20px;
    }

    .btn-done:hover {
        background: linear-gradient(45deg, #45a049, #3d8b40);
    }

    .btn-base {
        background: linear-gradient(45deg, #FF9800, #F57C00);
        color: white;
    }

    .btn-base:hover {
        background: linear-gradient(45deg, #F57C00, #E65100);
    }

    .btn-kitchen {
        background: linear-gradient(45deg, #9C27B0, #7B1FA2);
        color: white;
    }

    .btn-kitchen:hover {
        background: linear-gradient(45deg, #7B1FA2, #6A1B9A);
    }

    .btn-table {
        background: linear-gradient(45deg, #2196F3, #1976D2);
        color: white;
    }

    .btn-table:hover {
        background: linear-gradient(45deg, #1976D2, #1565C0);
    }

    .btn-cancel {
        background: linear-gradient(45deg, #F44336, #D32F2F);
        color: white;
    }

    .btn-cancel:hover {
        background: linear-gradient(45deg, #D32F2F, #C62828);
    }

    .table-input-group {
        margin-bottom: 20px;
    }

    .table-input-group label {
        display: block;
        margin-bottom: 10px;
        font-weight: bold;
        color: #333;
        font-size: 16px;
    }

    .table-number-input {
        width: 100%;
        padding: 12px;
        border: 2px solid #ddd;
        border-radius: 8px;
        font-size: 18px;
        text-align: center;
        margin-bottom: 15px;
    }

    .table-number-input:focus {
        outline: none;
        border-color: #2196F3;
        box-shadow: 0 0 10px rgba(33, 150, 243, 0.3);
    }

    .message {
        position: fixed;
        top: 20px;
        right: 20px;
        padding: 15px 20px;
        border-radius: 5px;
        color: white;
        z-index: 1000;
        transition: opacity 0.3s ease;
        font-weight: bold;
    }

    .message.success {
        background: #4CAF50;
    }

    .message.error {
        background: #F44336;
    }

    .table-icon {
        font-size: 4em;
        margin-bottom: 20px;
        color: #4CAF50;
    }

    .kitchen-icon {
        font-size: 4em;
        margin-bottom: 20px;
        color: #FF9800;
    }
    """
    return css_content, 200, {'Content-Type': 'text/css'}

if __name__ == '__main__':
    # Create templates directory and index.html
    import os
    os.makedirs('templates', exist_ok=True)
    
    html_content = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Restaurant Management System</title>
    <link rel="stylesheet" href="/static/style.css">
</head>
<body>
    <div class="container">
        <h1>🍽️ Restaurant Management System</h1>
        
        <div class="sections">
            <!-- Table 1 -->
            <div class="section table-section">
                <div class="table-icon">🍽️</div>
                <h2>Table 1</h2>
                <button class="btn btn-done" onclick="markDone('Table 1')">✓ Done</button>
            </div>

            <!-- Table 2 -->
            <div class="section table-section">
                <div class="table-icon">🍽️</div>
                <h2>Table 2</h2>
                <button class="btn btn-done" onclick="markDone('Table 2')">✓ Done</button>
            </div>

            <!-- Table 3 -->
            <div class="section table-section">
                <div class="table-icon">🍽️</div>
                <h2>Table 3</h2>
                <button class="btn btn-done" onclick="markDone('Table 3')">✓ Done</button>
            </div>

            <!-- Kitchen -->
            <div class="section kitchen-section">
                <div class="kitchen-icon">👨‍🍳</div>
                <h2>Kitchen</h2>
                
                <button class="btn btn-base" onclick="goToBase()">🏠 Go to Base</button>
                
                <button class="btn btn-kitchen" onclick="goToKitchen()">👨‍🍳 Go to Kitchen</button>

                <button class="btn btn-done" onclick="markKitchenDone()">✓ Kitchen Done</button>
                
                <div class="table-input-group">
                    <label for="tableNumber">Table Number:</label>
                    <input type="number" id="tableNumber" class="table-number-input" 
                           placeholder="Enter table number" min="1" max="999">
                </div>
                
                <button class="btn btn-table" onclick="goToTable()">🍽️ Go to Table</button>
                
                <button class="btn btn-cancel" onclick="cancelGoal()">❌ Cancel</button>
            </div>
        </div>
    </div>

    <script>
        function markDone(tableName) {
            fetch('/api/done', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                }
            })
            .then(response => response.json())
            .then(data => {
                showMessage(`${tableName} - ${data.message}`, data.success ? 'success' : 'error');
            })
            .catch(error => {
                console.error('Error:', error);
                showMessage('Error processing done request', 'error');
            });
        }

        function goToBase() {
            fetch('/api/go_to_base', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                }
            })
            .then(response => response.json())
            .then(data => {
                showMessage(data.message, data.success ? 'success' : 'error');
            })
            .catch(error => {
                console.error('Error:', error);
                showMessage('Error sending go to base command', 'error');
            });
        }

        function goToKitchen() {
            fetch('/api/go_to_kitchen', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                }
            })
            .then(response => response.json())
            .then(data => {
                showMessage(data.message, data.success ? 'success' : 'error');
            })
            .catch(error => {
                console.error('Error:', error);
                showMessage('Error sending go to kitchen command', 'error');
            });
        }

        function goToTable() {
            const tableNumber = document.getElementById('tableNumber').value;
            
            if (!tableNumber) {
                showMessage('Please enter a table number', 'error');
                return;
            }

            fetch('/api/go_to_table', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ table_number: tableNumber })
            })
            .then(response => response.json())
            .then(data => {
                showMessage(data.message, data.success ? 'success' : 'error');
                if (data.success) {
                    document.getElementById('tableNumber').value = '';
                }
            })
            .catch(error => {
                console.error('Error:', error);
                showMessage('Error sending go to table command', 'error');
            });
        }

        function cancelGoal() {
            fetch('/api/cancel', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                }
            })
            .then(response => response.json())
            .then(data => {
                showMessage(data.message, data.success ? 'success' : 'error');
            })
            .catch(error => {
                console.error('Error:', error);
                showMessage('Error sending cancel command', 'error');
            });
        }

        function showMessage(text, type) {
            const messageDiv = document.createElement('div');
            messageDiv.className = `message ${type}`;
            messageDiv.textContent = text;
            document.body.appendChild(messageDiv);
            
            setTimeout(() => {
                messageDiv.style.opacity = '0';
                setTimeout(() => {
                    document.body.removeChild(messageDiv);
                }, 300);
            }, 3000);
        }

        function markKitchenDone() {
            fetch('/api/kitchen_done', { method: 'POST' })
                .then(res => res.json())
                .then(data => showMessage(data.message, data.success ? 'success' : 'error'))  // or alert(data.message)
                .catch(err => showMessage('Error: ' + err, 'error'));
        }


        // Allow Enter key to submit table number
        document.getElementById('tableNumber').addEventListener('keypress', function(e) {
            if (e.key === 'Enter') {
                goToTable();
            }
        });
    </script>
</body>
</html>"""
    
    with open('templates/index.html', 'w') as f:
        f.write(html_content)
    
    # Initialize ROS2
    init_ros()
    
    # Run Flask app
    try:
        app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
    finally:
        if ros_node:
            ros_node.destroy_node()
        rclpy.shutdown()