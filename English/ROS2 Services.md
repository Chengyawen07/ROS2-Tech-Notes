## 1. **What is a Service in ROS 2?**

A **Service** in ROS 2 is a communication mechanism based on the **Request-Response Model**.

- **Client:** Sends a request to the server and waits for a response.
- **Server:** Processes the client's request and returns a result.

In simple terms, a service is like a **food ordering system**:

1. The client (customer) sends a request (places an order).
2. The server (restaurant) processes the request and returns a response (serves the food).

------

### **Difference Between a Service and a Topic**

| Feature                   | Service                                                     | Topic                                                       |
| ------------------------- | ----------------------------------------------------------- | ----------------------------------------------------------- |
| **Communication Model**   | Request-Response Model                                      | Publish-Subscribe Model                                     |
| **Data Flow**             | Bi-directional (Request -> Response)                        | Uni-directional (Publish -> Subscribe)                      |
| **Use Case**              | One-time tasks, e.g., querying a status or sending commands | Continuous data flow, e.g., sensor data or state broadcasts |
| **Sync/Async**            | Typically synchronous (wait for response)                   | Typically asynchronous (no waiting)                         |
| **Real-Time Suitability** | Suitable for immediate feedback tasks                       | Suitable for frequent, stateless data flow                  |

------

### **How to Implement a Service and Client in ROS 2?**

Below is an example in Python demonstrating how to implement a service and a client.

------

#### **Service Code (Server)**

**Functionality:** Implements a service that receives two integers and returns their sum.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # Using a standard service interface

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')  # Node name
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)  # Create service
        self.get_logger().info('Service is ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b  # Calculate the sum of the request parameters
        self.get_logger().info(f'Received request: a={request.a}, b={request.b}, sum={response.sum}')
        return response  # Return the response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)  # Keep the node running
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Steps to Run:**

1. Save the code as `add_two_ints_server.py`.

2. Run the server:

   ```bash
   ros2 run <package_name> add_two_ints_server
   ```

------

#### **Client Code**

**Functionality:** Creates a client that sends two integers to the service and receives their sum.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # Using a standard service interface

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')  # Node name
        self.client = self.create_client(AddTwoInts, 'add_two_ints')  # Create client
        while not self.client.wait_for_service(timeout_sec=1.0):  # Wait for the service to be available
            self.get_logger().info('Waiting for service...')
        self.get_logger().info('Service is available.')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.client.call_async(request)  # Asynchronous request
        return self.future

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    a, b = 5, 10  # The two integers to send
    future = node.send_request(a, b)

    rclpy.spin_until_future_complete(node, future)  # Wait for the response
    if future.result() is not None:
        response = future.result()
        node.get_logger().info(f'Response: {response.sum}')
    else:
        node.get_logger().error('Service call failed.')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Steps to Run:**

1. Save the code as `add_two_ints_client.py`.

2. Run the client:

   ```bash
   ros2 run <package_name> add_two_ints_client
   ```

------

#### **Interaction Between Server and Client**

When the service and client run simultaneously:

- The client sends two integers (e.g., `a=5, b=10`) to the service.
- The service calculates their sum and returns the result (e.g., `sum=15`).

**Client Output:**

```
[INFO] [<timestamp>] [add_two_ints_client]: Response: 15
```

**Server Output:**

```
[INFO] [<timestamp>] [add_two_ints_server]: Received request: a=5, b=10, sum=15
```

------

### **Summary**

- **Service:** Used for tasks requiring immediate feedback, such as querying a status or issuing commands.
- **Topic:** Better suited for continuous, stateless data streams, such as sensor readings or system broadcasts.
- **Choosing Between Them:** The choice depends on the task’s real-time requirements and data characteristics.
