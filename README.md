# Experimental Robotics Assignment 2: PlanSys2 Exploration & Visual Servoing

This ROS 2 package implements an autonomous robot mission using **PlanSys2** (PDDL planning), **Nav2**, and **OpenCV**. The robot explores a set of waypoints to detect ArUco markers, dynamically updates its knowledge base, and generates a second plan to revisit and center on the markers in a specific order (ascending ID).

## 🚀 Project Overview

The mission is divided into two distinct phases managed by a central controller:

1.  **Phase 1: Exploration**
    * The robot navigates to pre-defined waypoints (`wp1` through `wp4`).
    * At each waypoint, it executes a **Search** action (rotating 360° to scan the environment).
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

