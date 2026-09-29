# Agrirover: Tomato Detection and 3D Localization on a Mobile Manipulator

Semester 7 Project-I, B.Tech Mechatronics and Automation, VIT Chennai (report title: *Agribot*)


Agrirover is a mobile manipulator platform for tomato harvesting. It combines a custom-trained YOLOv8
detector running on a Jetson Orin Nano, 3D localization from RealSense D435 depth, a 5-DOF arm driven
through ROS 2 and an Arduino Uno, and a compliant TPU end-effector.

<p align="center">
  <img src="https://github.com/user-attachments/assets/c936b987-c656-48c5-a04b-7ece26a0c295" height="300" />
  <img src="https://github.com/user-attachments/assets/6f1701e5-93db-48ba-9f86-268d900140a4" height="300" />
</p>

## Demo

Live tomato detection and 3D localization on the Jetson Orin Nano, followed by the end-effector picking
motion.

https://github.com/user-attachments/assets/91c9d1a5-2fdd-4766-8c50-547cba2d2344

## Results

| Metric | Value |
|---|---|
| Detection precision / recall (ripe) | 0.90 / 0.93 |
| Detection precision / recall (unripe) | 0.94 / 0.91 |
| F1 (ripe / unripe) | 0.91 / 0.92 |
| Test set | 784 annotated samples, overall accuracy 0.92 |
| 3D localization error, 0.15–0.50 m | RMSE 2.03 cm, MAE 0.98 cm |
| Goal-pose frame-to-frame jitter | < 1 mm (X, Y), 0.23 mm (Z) [static target: confirm] |
| Inference time, 480x640 | [5.5–9.3 ms, TensorRT: confirm] |
| Pipeline rate | 28 FPS |

Depth error was measured against [ground-truth method: e.g. tape measure at N distances].

## Perception

A ROS 2 node (`agrirover_perception`) that:

- runs a custom-trained YOLOv8n model on RealSense D435 color frames on a Jetson Orin Nano
  (Ubuntu 22.04, JetPack 6.2, ROS 2 Humble)
- classifies tomatoes as ripe or unripe
- reads depth at each detection's bounding-box center and deprojects it to a 3D point with the camera
  intrinsics
- transforms points from the camera frame to the robot base frame with a calibrated 4x4 transform
- publishes goal poses (`tomato/goal_pose`), RViz markers, a detection-status signal and an annotated
  image stream

## Manipulation

- 5-DOF arm with 3D-printed linkages, MG996R servos and a 500 mm reach
- Software chain: camera node, then IK node, then serial node, then Arduino Uno PWM servo control
- Compliant three-claw TPU end-effector; a worm-gear redesign that moves all three claws synchronously was
  designed in SolidWorks

## Robot description

URDF for the arm, camera mount and mobile base, with bringup launch files in `agrirover_bringup`.

## Roadmap

- MoveIt 2 motion planning between perception output and arm control
- Perception-driven pick-and-place
- Multi-frame tracking
- V-SLAM / LiDAR SLAM field navigation

## Repository structure

```
agrirover/                    Core package
agrirover_bringup/            Launch files, system startup
agrirover_description/        URDF and robot description
agrirover_manipulation/       IK node, serial node, end-effector logic
agrirover_perception/         Detection and 3D localization node
yolov8n_tomato.pt             Trained detection model
Datasets and Misc.zip         Training data and supporting files
```
