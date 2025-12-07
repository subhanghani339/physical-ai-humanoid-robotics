---
sidebar_position: 2
---

# ROS 2 Fundamentals (Weeks 3-5)

## ROS 2 Architecture Overview

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot applications. Its distributed nature allows for components to run on different machines and communicate seamlessly.

Key architectural concepts:

*   **Nodes**: Executables that perform computation (e.g., a node for reading sensor data, a node for controlling motors).
*   **Topics**: A publish/subscribe mechanism for nodes to exchange data asynchronously. Nodes publish messages to topics, and other nodes subscribe to those topics to receive the messages.
*   **Services**: A request/reply mechanism for synchronous communication between nodes. A client node sends a request to a service server node and waits for a response.
*   **Actions**: A long-running goal-oriented communication mechanism, built on topics and services, used for tasks that take time to complete (e.g., navigating to a goal, performing a complex manipulation).

## Nodes, Topics, Services, and Actions Explained

### Nodes

Nodes are the fundamental building blocks of a ROS 2 system. Each node should ideally be responsible for a single, well-defined task. This modularity allows for easier debugging, development, and reuse of components.

### Topics

Topics are named buses over which nodes exchange messages. Messages are simple data structures. When a node publishes a message to a topic, all nodes subscribed to that topic receive a copy of the message.

### Services

Services enable nodes to make RPC-like function calls to other nodes. This is useful for tasks that require an immediate response, such as querying a sensor for its current reading or triggering a specific action that completes quickly.

### Actions

Actions are designed for tasks that might take a significant amount of time to complete and where the client needs feedback on the progress of the goal. An action client sends a goal to an action server, which then provides continuous feedback and a final result. The client can also preempt a goal.

## Code Example: Simple Publisher Node in Python

This node will publish a simple "Hello, ROS 2!" message to a topic.

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
        msg.data = f'Hello, ROS 2!: {self.i}'
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

## Code Example: Simple Subscriber Node in Python

This node will subscribe to the topic from the publisher and print the received messages.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Explanation of Launch Files

Launch files in ROS 2 are XML or Python files used to start and configure multiple ROS 2 nodes and other processes simultaneously. They simplify the process of setting up complex robot systems by allowing you to define the startup behavior of all components in a single file.

Example `my_robot_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='minimal_publisher',
            name='my_publisher',
            output='screen'
        ),
        Node(
            package='my_package',
            executable='minimal_subscriber',
            name='my_subscriber',
            output='screen'
        ),
    ])
```

## Practical Exercise: Build a Temperature Monitoring System

**Goal**: Create two ROS 2 nodes in Python:

1.  A `temperature_sensor_node` that publishes simulated temperature readings (e.g., random numbers) to a topic named `/temperature` every second.
2.  A `temperature_logger_node` that subscribes to `/temperature` and logs any temperature readings above a certain threshold (e.g., 25.0 degrees Celsius).

**Steps**:

1.  Create a new ROS 2 package.
2.  Implement the `temperature_sensor_node.py`.
3.  Implement the `temperature_logger_node.py`.
4.  Create a launch file to start both nodes simultaneously.
5.  Run your system and observe the output.
