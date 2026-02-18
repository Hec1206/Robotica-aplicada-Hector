# activity ROS2 #

This project implements **ROS 2 publisher–subscriber architecture** like activity one but we are adding a service 

A constant number is published periodically, received by a second node, accumulated in a counter, and then republished with the updated value like activity one but with one exception if the service recieve a True de counter reset.

## Node Description

### 1. `robot1_publisher` Node

This node does not have any changes it gives a constant integer value periodically.

- **Published Topic:** `/robot_pub1`
- **Message Type:** `example_interfaces/msg/Int64`
- **Publishing Rate:** 1 message per second
- **Functionality:**
  - Always publishes the value `1`
  - Logs each published value to the terminal

---

### 2. `number_counter` Node

This node subscribes to the published number, accumulates it, and republishes the result and in this version we add a serve whenever it gets a True it resets the counting.

- **Subscribed Topic:** `/robot_pub1`
- **Published Topic:** `/robot_pub2`
- **Message Type:** `example_interfaces/msg/Int64 from std_srvs.srv import SetBool`
- **Functionality:**
  - Maintains an internal counter
  - Adds each received number to the counter
  - Publishes the updated counter value immediately from the subscriber callback
  -has a services whenever it gets a true resets the counter

the part of the service to add to the code 

```basch
        self.server_= self.create_service(SetBool, #Service TYPE
                                          "reset_counter", #service Name
                                          self.reset_callback
                                          )
        self.get_logger().info("Service Server is ready")

    def reset_callback(self, request, response):
        if request.data:
            self.counter = 0
            self.get_logger().info("Counter reset!")
            response.success = True
            response.message = "Counter reset"
        else:
            response.success = True
            response.message = "Reset not executed"

        return response
```
the Client its going to be the terminal so we need to add this line in ubuntu so the serve has a client and it can reset the counter

# How to Call the Service
To reset the counter we need to add this line
```basch
ros2 service call /reset_counter std_srvs/srv/SetBool "{data: true}"
```
---
## System Architecture

- **Nodes**
  - `robot1_publisher`
  - `number counter_code`

- **Topics**
  - `/robot_pub1`
  - `/robot_pub2`

- **Service**
    -`/reset_counter`
---

## Node Implementation

### robot1_publisher Node Code

```python
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

# Node that publishes a constant number periodically
class numPublisher(Node):
    def __init__(self):
        super().__init__("robot1_publisher")

        self.publisher_ = self.create_publisher(Int64, "/robot_pub1", 10)
        self.create_timer(1.0, self.number)

        self.number = 1
        self.get_logger().info("Sending number")

    def number(self):
        msg = Int64()
        msg.data = self.number
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    my_publisher_node = numPublisher()
    rclpy.spin(my_publisher_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

---
### number_counter Node Code

```python
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from std_srvs.srv import SetBool


class myNode_function(Node):
    
    def __init__(self):
        super().__init__('number_counter')
        self.counter = 0
        self.get_logger().info('beep Boop R2D2 is operational.')
        self.subscriber = self.create_subscription(Int64, "/robot_pub1", self.callback_receive_info,10) #
        self.publisher = self.create_publisher(Int64, "/robot_pub2", 10)


        self.server_= self.create_service(SetBool, #Service TYPE
                                          "reset_counter", #service Name
                                          self.reset_callback
                                          )
        self.get_logger().info("Service Server is ready")

    def reset_callback(self, request, response):
        if request.data:
            self.counter = 0
            self.get_logger().info("Counter reset!")
            response.success = True
            response.message = "Counter reset"
        else:
            response.success = True
            response.message = "Reset not executed"

        return response
    
    def callback_receive_info(self, msg: Int64):
        # Add the received number to the counter
        self.counter += msg.data
        # Create a message with the updated counter value
        out_msg = Int64()
        out_msg.data = self.counter
        # Log received and accumulated values
        self.get_logger().info(f"Received: {msg.data} | Counter: {self.counter}" )
         # Publish the updated counter
        self.publisher.publish(out_msg)
 
def main(args=None):
    rclpy.init(args=args)
    second_code =myNode_function()
    rclpy.spin(second_code)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

---
## Results

![](imgs2/service.png)

---