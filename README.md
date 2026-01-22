# Autonomous Marker Search and Inspection Robot

An autonomous ROS2 project utilizing **PlanSys2** for high-level mission planning and **Nav2** for autonomous navigation. The robot executes a two-phase mission: searching specific waypoints for ArUco markers and subsequently revisiting them to perform inspection tasks (taking photos).

## 📋 Project Overview

This project implements a modular architecture composed of three main ROS2 packages working in tandem to move from high-level symbolic planning down to low-level simulation control.

### The Three Pillars
1.  **`plansys_interface`**: The **Brain**. Manages the PDDL domain, mission states, and coordinates the execution of high-level actions (Move, Search, Picture) by interfacing with PlanSys2.
2.  **`ros2_navigation`**: The **Navigator**. Contains the configuration for the Nav2 stack, including costmap parameters, AMCL localization, and environment maps.
3.  **`robot_model_and_simulation`**: The **Body & World**. Provides the robot's physical description (URDF/Xacro), sensor plugins (Lidar/Camera), and the Gazebo simulation environment.

---

## 🏗️ System Architecture

### 1. Mission Logic (`plansys_interface`)
* **`action_manager_node`**: Orchestrates the mission. It transitions the system from **Phase 1 (Discovery)** to **Phase 2 (Inspection)** by dynamically updating the PDDL goal once markers are identified.
* **Action Executors**:
    * `move_action_node`: A PlanSys2 wrapper that sends goals to the Nav2 stack.
    * `search_action_node`: Commands the robot to rotate at waypoints and scan for ArUco IDs.
    * `picture_action_node`: Uses visual servoing to center the robot on a marker and "capture" it.

### 2. PDDL Domain Logic
The high-level logic is defined in `domain.pddl` using the following actions:

| Action | Description |
| :--- | :--- |
| **`move`** | Navigates the robot between waypoints or markers. |
| **`search_waypoint`** | Triggers a 360° rotation to detect markers (Phase 1). |
| **`take_picture`** | Approaches and captures an image of a specific marker (Phase 2). |

---

## ⚙️ Prerequisites & Dependencies

* **OS:** Ubuntu 20.04/22.04
* **ROS2 Distribution:** Humble / Galactic / Jazzy
* **Core Dependencies:**
    * `plansys2`
    * `navigation2` & `nav2_bringup`
    * `opencv` & `cv_bridge`
    * `gazebo_ros_pkgs`

```bash
# Install dependencies
sudo apt update
sudo apt install ros-$ROS_DISTRO-plansys2-* ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-vision-opencv

## 🚀 Installation & Build

1.  **Create and Initialize Workspace:**
    ```bash
    mkdir -p ~/erl_ws/src
    cd ~/erl_ws/src
    ```

2.  **Clone the Repository:**
    ```bash
    # Clone the three packages into your src directory
    git clone <your-repository-url> .
    ```

3.  **Install Dependencies:**
    Use `rosdep` to automatically install all required system dependencies for the packages:
    ```bash
    cd ~/erl_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

4.  **Build the Project:**
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

---

## ▶️ Running the Project

To execute the full autonomous mission, follow these steps in order using separate terminals.



### 1. Launch the Simulation
This brings up the Gazebo environment, the robot model (URDF), and the necessary sensor plugins.
```bash
ros2 launch robot_model_and_simulation simulation.launch.py

### 2. Launch the Localization
This starts the AMCL localization with the specified parameters and environment map.
```bash
ros2 launch ros2_navigation localization.launch.py

### 3. Launch the Navigator
This initializes the Nav2 stack and starts the action manager node.
```bash
ros2 launch ros2_navigation navigator.launch.py

### 4. Launch the Mission
This sends the PDDL goal to the action manager node, which coordinates the execution of high-level actions.
```bash
ros2 launch plansys_interface mission.launch.py

## Project Structure

The project is structured as follows:

```
Experimental_Robotics_course_assignment2/
├── plansys_interface/
│   ├── action_manager_node.py       # The Brain. Manages the PDDL domain, mission states, and coordinates the execution of high-level actions (Move, Search, Picture) by interfacing with PlanSys2.
│   ├── search_action_node.py        # The Navigator. Contains the configuration for the Nav2 stack, including costmap parameters, AMCL localization, and environment maps.
│   ├── picture_action_node.py       # The Body & World. Provides the robot's physical description (URDF/Xacro), sensor plugins (Lidar/Camera), and the Gazebo simulation environment.
│   └── action_manager_node.py
├── ros2_navigation/
│   ├── navigator.py                 # The Navigator. Contains the configuration for the Nav2 stack, including costmap parameters, AMCL localization, and environment maps.
│   └── localization.py              # The Body & World. Provides the robot's physical description (URDF/Xacro), sensor plugins (Lidar/Camera), and the Gazebo simulation environment.
└── robot_model_and_simulation/
    ├── robot_model_and_simulation.py  # The Body & World. Provides the robot's physical description (URDF/Xacro), sensor plugins (Lidar/Camera), and the Gazebo simulation environment.
    └── simulation.py                  # The Body & World. Provides the robot's physical description (URDF/Xacro), sensor plugins (Lidar/Camera), and the Gazebo simulation environment.
```

---