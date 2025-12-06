---
sidebar_position: 2
---

# ROS 2 Fundamentals (Weeks 3-5)

## ROS 2 Architecture Overview

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot applications. Its distributed architecture allows for modular development and deployment across multiple machines.

Key architectural concepts:

*   **Nodes**: Executable processes that perform computation (e.g., a node for reading sensor data, a node for controlling motors).
*   **Topics**: A mechanism for nodes to asynchronously send data (messages) to each other. Nodes publish messages to topics, and other nodes subscribe to those topics.
*   **Services**: A mechanism for nodes to synchronously request and receive responses from each other, similar to a function call.
*   **Actions**: A higher-level abstraction for long-running, goal-oriented tasks. Actions provide feedback and allow for preemption.
*   **Parameters**: Configuration values that can be set for nodes at runtime.

## Nodes, Topics, Services, and Actions Explained

### Nodes

A node is typically responsible for a single module of functionality (e.g., camera driver, motor controller, path planner). Multiple nodes can run concurrently and communicate with each other.

### Topics

Topics enable a publish-subscribe communication pattern. One node publishes information to a topic, and any number of other nodes can subscribe to that topic to receive the information.

### Services

Services are used for request-response communication. A client node sends a request to a service server node and waits for a response. This is suitable for tasks that require an immediate result.

### Actions

Actions are designed for tasks that might take a long time to complete and where feedback on progress is important. An action client sends a goal to an action server, which then provides continuous feedback while processing the goal, and eventually sends a result.

## Code Example: Simple Publisher Node in Python

This example demonstrates a basic ROS 2 publisher node that publishes a string message to a topic.

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
        msg.data = f'Hello ROS 2: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Code Example: Simple Subscriber Node in Python

This example shows a basic ROS 2 subscriber node that listens for messages on a topic.

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

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Explanation of Launch Files

Launch files in ROS 2 are XML or Python files used to start and configure multiple ROS 2 nodes and other processes simultaneously. They simplify the deployment of complex robotic systems.

Example (Python launch file):

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='minimal_publisher',
            name='my_publisher',
            output='screen',
        ),
        Node(
            package='my_package',
            executable='minimal_subscriber',
            name='my_subscriber',
            output='screen',
        ),
    ])
```

## Practical Exercise: Build a Temperature Monitoring System

**Goal**: Create a simple ROS 2 system that simulates a temperature sensor and displays the readings.

**Steps**:

1.  **Create a publisher node**: This node will simulate a temperature sensor, publishing random temperature values (e.g., between 20-30 degrees Celsius) to a `temperature_topic` every second.
2.  **Create a subscriber node**: This node will subscribe to the `temperature_topic` and print the received temperature readings to the console.
3.  **Create a launch file**: Configure a launch file to start both the publisher and subscriber nodes simultaneously.
4.  **Test**: Run your launch file and observe the temperature readings in the subscriber's console output.
