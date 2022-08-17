## My Robotic Arm
<!--<h4>Object picking and Placing with a 3-DOF <br> Articulated Robotic Manipulator<br>using ROS</h4>-->

<p align="center">
<b><i>Object picking and Placing with 3 DOF Articulated Manipulator using ROS and Simulated in CoppeliaSim (Open Dynamics Engine)</i></b>
</p>

<p align="center">
<img src="figures/sim_env.png" alt="" width="51%"><img src="figures/picked_obj_demo.png" alt="" width="47.1%">
</p>

<p align="center">
<b>Muhammad Muneeb ur Rehman</b>
<br>
<a href="mailto:muneeburrehman197@gmail.com" target="_top">muneeburrehman197@gmail.com</a>
</p>

------------

## Abbreviations

- **ARC:** Amazon Robotics Challenge
- **ROS:** Robot Operating System
- **DOF:** Degrees of Freedom
- **EE:** End Effector

### Working of Autonomous Robotic Arm Pick and Place Work Flow in Simulation

<video src="./assets/output1.mp4"></video>

# 1. Introduction

This project is inspired by [Amazon Robotics Challenge](https://www.amazonrobotics.com/#/roboticschallenge), where decision making, for autonomous picking and stowing, picking objects  off shelves and putting them in shipping boxes, still remains a challenging task.

As per ARC rules, robotic arm must be capable of these:

- Object Recognition
- Pose (position + Orientation) Recognition
- Pick Planning
- Compliant Manipulation
- Motion Planning
- Error Detection and Recovery

In this project, all above mentioned conditions are satisfied, except, there are some limitations. 

As it is 3 DOF robotic arm, only three spatial position coordinates are possible (we cannot control the orientation of manipulator EE). Also error detection is a part of project, but error recovery is not yet implemented, although it is a part of future work.

### Objective

Commercially viable autonomous picking and placing in unstructured (dynamic) environment still remains a difficult challenge. The objective of this project is to demonstrate the autonomous capability of 3 DOF articulated manipulator in simulation to pick an object and placing it in appropriate box.

Within the context of this project, a single pick and place cycle can be divided into following tasks:

1. Identify the target object on table using 2D Vision sensor (RGB Camera)
2. Classify and Localize the Object using YOLOv3 (supervised machine learning object detection algorithm)
3. Perform Inverse Kinematics (to calculate joint angles)
4. Plan the clean movement toward the object to pick
5. Efficiently pick the object using suction gripper
6. Plan and perform a clean movement towards a drop site
7. Drop an object in appropriate box

### Relevance

The object detection using computer vision methods, such as edge detection, color gradient etc. is usually used for many applications. This method is suitable for just structured environments where conditions remain same with time. For dynamic and unstructured environments, traditional computer vision becomes irrelevant to implement.

Machine learning classification model is widely used for object detection. Machine learning enables machines to behave intelligently in dynamics scenarios and take decisions. Object detection performs two task: Object Classification and Object Localization.

Object Classification refers to detecting the object in workspace and classifying into predefined class. For example, in the below image, person, vase, chair, laptop, football etc. are predefined classes of objects that our machine learning algorithm learns to predict after training a model.

![](/demo-mount/FYP/Design Project Part 1 Progress Report/yolo_detection.jpg)

We have trained our machine learning model for detecting various shapes, predominantly, square, circle, and triangle etc. Machine learning model can be trained to predict any set of objects as per the need of application, this proves the robustness of system.

### Applications

Robotic manipulators have become ubiquitous in almost every industry; from food, beverage, shipping and packaging to manufacturing. Almost all robotic applications face the fundamental challenge of decision making, how to perform a certain task? If we can incorporate the artificial intelligence and machine learning into our robotic systems in such a way to completely automate the task, it will be massive industrial disruption.

This project is mainly focused on applications of robotics in retail stores and in distribution centers.
