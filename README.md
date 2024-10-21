# TurtleBot3 Web Navigation

This repository contains a ROS2 package that allows you to control a simulated TurtleBot3 using a web interface. The system uses FastAPI to handle communication between the web application and ROS2, with a ROS2 Bridge script interfacing with Nav2 to set goals and start navigation.

## System Design Overview

The system consists of four main components:

1. **Web Application (Frontend)**
2. **FastAPI Backend**
3. **ROS2 Bridge**
4. **TurtleBot3 Simulation**

### Web Application (Frontend)

- A simple HTML/JavaScript application
- Allows users to input X, Y, theta values for the 2D Pose Goal
- Provides buttons to set navigation goal and start navigation
- Communicates with the FastAPI backend

### FastAPI Backend

- Handles HTTP requests from the web application
- Processes the navigation goals
- Communicates with the ROS2 Bridge

### ROS2 Bridge

- A Python script that interfaces between FastAPI and ROS2
- Uses rclpy to create ROS2 nodes and interact with Nav2

### TurtleBot3 Simulation

- Uses the default turtlebot3_world
- Controlled by Nav2

## Repository Structure

```
ros2_ws/
└── src/
    └── turtlebot3_web_nav/
        ├── turtlebot3_web_nav/
        │   ├── __init__.py
        │   └── ros2_bridge.py
        ├── web_app/
        │   ├── index.html
        │   ├── style.css
        │   └── script.js
        ├── backend/
        │   ├── main.py
        │   └── requirements.txt
        ├── launch/
        │   └── web_nav_launch.py
        ├── resource/
        │   └── turtlebot3_web_nav
        ├── test/
        │   └── test_copyright.py
        ├── setup.py
        ├── setup.cfg
        └── package.xml
```

## Implementation Details

### 1. ROS2 Bridge (`ros2_bridge.py`)

This script creates a ROS2 node that interfaces with Nav2. It provides functionality to:

- Set navigation goals
- Start navigation
- Handle goal responses and results

### 2. FastAPI Backend (`main.py`)

The FastAPI application handles HTTP requests from the web interface. It provides two main endpoints:

- `/set_goal`: Sets a new navigation goal
- `/start_navigation`: Initiates the navigation process

### 3. Web Application (`index.html`, `script.js`)

The web interface allows users to:

- Input X, Y, and theta values for the navigation goal
- Set the navigation goal
- Start the navigation process

### 4. Package Setup (`setup.py`)

This file configures the ROS2 package, including:

- Package name and version
- Dependencies
- Entry points for ROS2 nodes

## Usage

1. Clone this repository into your ROS2 workspace.
2. Build the package using `colcon build`.
3. Source your ROS2 workspace.
4. Launch the TurtleBot3 simulation (not included in this package).
5. Run the ROS2 bridge node.
6. Start the FastAPI backend.
7. Open the web application in a browser.

## Dependencies

- ROS2 (tested with Humble)
- Nav2
- FastAPI
- Uvicorn
- TurtleBot3 packages

## Contributing

Contributions to improve the functionality or documentation of this package are welcome. Please feel free to submit issues or pull requests.

## License

[Insert your chosen license here]

## Acknowledgments

This project was inspired by the need for an easy-to-use web interface for controlling TurtleBot3 in ROS2 simulations.
