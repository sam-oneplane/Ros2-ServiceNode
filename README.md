# Ros2-RequestNode
ROS2 - Http req. for Ros Topic interacton Node Design

## Ros2 version 
- ros humble

## Overview
The `ros_rest_relay` node bridges ROS2 and REST API, allowing HTTP GET and POST requests to interact with dynamic ROS topics.

## Components
- package name : `ros_rest_interface`
- node name    : `ros_rest_relay`

### ROS2 Node
- Dynamically subscribes to and publishes on ROS topics based on API requests.

### Node API
- `GET /{topic_name}`: Returns the latest message from the specified topic.
- `POST /{topic_name}`: Publishes a message to the specified topic.

## Implementation Details
- `ros_rest_relay` node runs a simple HTTP server using the `Crow` library.
- JSON parsing and serialization handled by `jsoncpp`.
- Topics are extracted from the URL and handled dynamically.

## Usage
- Build the Ros2 package:  
`cd /path/to/your/ros2_ws`
`colcon build --packages-select ros_rest_interface`
- Source Ros2 worksspace : 
`source /path/to/your/ros2_ws/install/setup.bash`   
- Start the node:
`ros2 run ros_rest_interface ros_rest_relay`
- Access via REST API at `http://localhost:8080`

## Testing
- Python script `test_ros_rest_interface.py` provides tests for the REST API.

## Docker
- Build :  `docker build -t ros_rest_interface:latest .`
- Run :  `docker run -it --rm --network host ros_rest_interface:latest`
