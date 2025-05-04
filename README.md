# RAS545_FinalProject2025_-spring-25
# Project: Smart and Accurate Path Planning with the Mycobot Pro 600: Solving Mazes

## 1. Overview

This project demonstrates a system for enabling the MyCobot Pro 600 collaborative robot arm to autonomously solve a 4x4 rectangular maze. It integrates computer vision, robotic simulation (digital twin), path planning, and real-time control to achieve accurate motion planning and execution. The core idea is to capture the maze using a camera, solve it using software, validate the path in a digital twin simulation, and then execute the path on the physical robot.

## 2. Methodology

The project follows a structured workflow involving several key methods:

1.  **Kinematic Analysis:**
    * A kinematic diagram of the MyCobot Pro 600 was created to represent its structure and joint relationships.
    * Homogeneous Transformation Matrices (HTMs) were derived manually from the diagram and verified in MATLAB to model the robot's forward kinematics ($^{0}H_{6}$).

2.  **Robot Modeling & Simulation (Digital Twin):**
    * A digital twin of the MyCobot Pro 600 was created in MATLAB using the Robotics System Toolbox (either via URDF import or programmatically using `rigidBodyTree`).
    * Forward Kinematics (FK) was used within the simulation to verify end-effector poses corresponding to specific joint angles.
    * The digital twin was used extensively for path validation *before* physical execution.

3.  **Maze Acquisition & Processing:**
    * A live video feed was captured using OpenCV.
    * A frame containing the maze was captured and automatically processed:
        * Converted to grayscale (`rgb2gray`).
        * Binarized (`imbinarize`).
        * The largest connected component (maze board) was identified (`regionprops`).
        * The board was cropped and rotated to a standard orientation.

4.  **Maze Solving:**
    * An AI-Kit camera captured the maze image.
    * An image-processing pipeline extracted maze walls and corridors.
    * Entry (red blob) and exit (green blob) points were identified.
    * Dijkstra's algorithm was used to find the optimal (shortest) path between the entry and exit points.
    * The pixel-based path was translated into real-world coordinates relative to the robot's workspace using camera calibration data.

5.  **Path Planning & Inverse Kinematics:**
    * The solution path was initially represented as a series of straight-line waypoints.
    * This path was "densified" by adding intermediate target points to ensure smooth, continuous motion.
    * A MATLAB-based inverse kinematics (IK) solver calculated the required joint angles for the robot to reach each target point along the densified path.

6.  **Simulation Validation:**
    * The complete sequence of joint angles (the trajectory) was simulated using the digital twin in MATLAB/Simscape Multibody.
    * This step verified reachability and checked for potential collisions before running on the physical hardware.

7.  **Motion Execution:**
    * The validated sequence of joint angles was streamed from the computer to the physical MyCobot Pro 600.
    * Communication was handled using TCP/IP sockets, managed by a Python script.

8.  **Comparative Analysis & Final Validation:**
    * The actual end-effector coordinates of the physical robot during the run were logged.
    * These coordinates were compared against the positions predicted by the forward kinematics in the digital twin model.
    * Any significant deviations were investigated to ensure close alignment between the simulation and the real-world execution.

## 3. Key Technologies & Tools

* **Hardware:** MyCobot Pro 600, AI-Kit Camera
* **Software:**
    * Python: For overall control, OpenCV, socket communication.
    * OpenCV: For image acquisition and processing.
    * MATLAB: For robot modeling (Robotics System Toolbox), forward/inverse kinematics, simulation (Simscape Multibody), path planning.
* **Algorithms:** Dijkstra's Algorithm (maze solving).
* **Communication:** TCP/IP Sockets.

## 4. Execution Workflow Summary

1.  Capture and process the maze image using OpenCV.
2.  Solve the maze using Dijkstra's algorithm to get waypoints.
3.  Translate waypoints to robot coordinates and densify the path.
4.  Calculate required joint angles using MATLAB's IK solver.
5.  Validate the full trajectory in the MATLAB digital twin.
6.  Stream joint angles to the MyCobot Pro 600 via TCP using Python.
7.  Execute the path on the physical robot.
8.  Compare real vs. simulated execution for validation.

## 5. References

* [MyCobot Pro 600 Documentation](https://docs.elephantrobotics.com/docs/gitbook-en/2-serialproduct/2.3-myCobot_Pro_600/2.3-myCobot_Pro_600.html)
* [Environment Building for ROS1 (MyCobot Pro 600)](https://docs.elephantrobotics.com/docs/pro600-en/12-ApplicationBaseROS/12.1-ROS1/12.1.2-EnvironmentBuilding.html)
* Project Google Drive Link (Contains video): [Link from Document](https://drive.google.com/drive/folders/1z-Yw3Gj1HG1zU_HUEZpT56SMKt-uV438?usp=drive_link)



