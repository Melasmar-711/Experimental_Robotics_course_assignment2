# Experimental Robotics Assignment 2: PlanSys2 Exploration & Visual Servoing

This ROS 2 package implements an autonomous robot mission using **PlanSys2** (PDDL planning), **Nav2**, and **OpenCV**. The robot explores a set of waypoints to detect ArUco markers, dynamically updates its knowledge base, and generates a second plan to revisit and center on the markers in a specific order (ascending ID).

## 🚀 Project Overview

The mission is divided into two distinct phases managed by a central controller:

1.  **Phase 1: Exploration**
    * The robot navigates to pre-defined waypoints (`wp1` through `wp4`).
    * At each waypoint, it executes a **Search** action (rotating 360° to scan the environment).
    * A clipped camera view is set to 8 m to prevent detection of other arucos at different waypoints
    * Detected ArUco markers are stored in the PlanSys2 knowledge base using PDDL predicates.

2.  **Phase 2: Ordered Processing**
    * Once exploration is complete, the controller retrieves all found markers.
    * It sorts them by ID (lowest to highest).
    * It generates a new PDDL plan to visit the markers in this strict order.
    * The robot navigates to the marker's location and performs **Visual Servoing** to align the camera center with the marker center.

## 🛠️ Tech Stack

* **ROS 2** (Jazzy/Humble)
* **PlanSys2**: For PDDL-based high-level planning and dispatching.
* **Nav2**: For autonomous navigation and path planning.
* **OpenCV & ArUco**: For marker detection and image processing.
* **C++**: Implementation of action nodes and controller.

## 📂 Architecture

### 1. Controller Node (`get_plan_and_execute`)
The "brain" of the operation. It initializes the PlanSys2 clients, requests the initial exploration plan, monitors execution, and handles the logic transition to Phase 2. It bypasses the standard Executor Client to allow for reactive, callback-driven plan execution.

### 2. Action Nodes
These nodes implement the PDDL actions using `plansys2::ActionExecutorClient`:

* **`move_action_node`**:
    * Maps PDDL waypoints (e.g., `wp1`) to physical coordinates.
    * Sends goals to the **Nav2** stack using the `MapsToPose` action.
    * Implements robust failure handling and lazy initialization.
* **`search_waypoint_node`**:
    * Rotates the robot using `cmd_vel` to scan the area.
    * Uses **OpenCV** to detect ArUco markers.
    * Updates the PDDL Problem Instance with `(marker_at id wp)` predicates upon detection.
    * Displays a live camera feed with detection overlays.
* **`process_action_node`**:
    * Requires the robot to be at the correct waypoint (enforced by PDDL preconditions).
    * Uses a **P-Controller** (Visual Servoing) to rotate the robot until the marker is centered in the image frame.
    * Locks onto the target (GREEN circle) for 5 seconds before finishing.

## 📋 PDDL Domain

The domain defines three durative actions:
1.  **`move`**: Moves the robot between waypoints.
2.  **`search_waypoint`**: Rotates to find markers.
3.  **`process_marker`**: Centers on a marker. Enforces processing order using the `(next_id ?prev ?curr)` predicate chain constructed dynamically by the controller.

## ⚙️ Installation & Build

1.  **Prerequisites**:
    Ensure you have ROS 2, PlanSys2, Nav2, and OpenCV installed.
    ```bash
    sudo apt install ros-$ROS_DISTRO-plansys2-*
    sudo apt install ros-$ROS_DISTRO-nav2-*
    sudo apt install ros-$ROS_DISTRO-opencv-*
    ```

2.  **Clone the Repository**:
    ```bash
    mkdir -p ros2_ws/src
    cd ros2_ws/src
    git clone https://github.com/Melasmar-711/Experimental_Robotics_course_assignment1.git
    ```

3.  **Build**:
    ```bash
    cd ..
    source install/setup.bash
    ```

## ▶️ Usage

### 1. Launch the Simulation & Nodes

in the first terminal run and wait for a few seconds for everthing to run smoothly. 
because the rviz configuration sets the **fixed frame to map** which is not loaded yet.

`ros2 launch assign2 spawn_robot.launch.py `

in the second terminal launch the localiztion node and wait untill the mao is loaded.

`ros2 launch ros2_navigation localization.launch.py`

in a third terminal launch the navigation node .

`ros2 launch ros2_navigation navigation.launch.py`

in a fourth terminal after you make sure the action server of the navigation is up

`ros2 launch plansys_interface distributed_actions_3.launch.py`

in a fifth terminal run 

`ros2 run  plansys_interface get_plan_and_execute`



## Results

### video
[![Demo](https://img.youtube.com/vi/Gk8OgA8lAYM/0.jpg)](https://youtu.be/Gk8OgA8lAYM)
