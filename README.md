# Agrirover

### Autonomous Mobile Fruit-Picking Robot

Semester 7 Project-I (B.Tech Mechatronics and Automation, VIT Chennai)

Agrirover is an autonomous mobile manipulator built to identify, localize, and harvest tomatoes in unstructured farm environments. The system combines vision-based perception, an articulated manipulator, and a mobile base, with the long-term goal of fully autonomous in-field navigation and picking.

This was my first ground-up robotics systems project — integrating perception, manipulation, and (planned) navigation into a single pipeline — and it's where I ran into the problems that later shaped how I approached [Project Tvastr](#).

---

## System Overview

- **Perception:** Real-time fruit detection and localization using a custom-trained YOLOv8 model (`yolov8n_tomato.pt`), running on a Jetson Orin Nano for on-edge inference.
- **Manipulation:** Articulated end-effector designed to adapt to fruit shape/size for low-damage picking.
- **Navigation:** Planned V-SLAM-based traversal through field rows (not implemented — see Status).
- **Compute:** Edge inference on Jetson Orin Nano.

## Status

This project was built under a fixed academic timeline and was paused, not completed, when I moved on to my final-year thesis project (obstacle-aware path planning for a SCARA manipulator, IGCAR Kalpakkam). Status below is accurate as of hand-off:

| Subsystem | Status | Notes |
|---|---|---|
| **Perception** | ✅ Working | Real-time tomato detection and localization performing reliably. [Demo video](#) shows the pipeline running live. |
| **Manipulation (pick-and-place)** | ⚠️ Partial | End-effector and grasp mechanism functioned, but closed-loop pick-and-place did not work end-to-end. Demoed using hardcoded joint commands rather than perception-driven motion planning (see below). |
| **Navigation** | ❌ Not started | V-SLAM-based field navigation was scoped but never implemented. |

### Why pick-and-place didn't close the loop

The blocker was on the **motion planning / inverse kinematics** side, not perception or hardware. Detection reliably output a fruit location, but converting that into a valid, collision-free joint trajectory for the arm to reach and grasp it didn't work reliably — the IK solutions either weren't converging consistently or weren't producing motion that aligned the gripper properly with the detected target. Given the project timeline, I demoed the perception pipeline live and used scripted joint commands to show the picking motion in isolation, rather than a fully closed perception → planning → control loop.

This is the main thing I'd revisit first: a proper motion planning layer (e.g. MoveIt 2) between perception output and arm control, with explicit handling for IK failure cases instead of assuming a solution always exists.

## Repository Structure

```
agrirover/
├── agrirover/                  # Core package
├── agrirover_bringup/          # Launch files, system startup
├── agrirover_description/      # URDF / robot description
├── agrirover_manipulation/     # Arm control, end-effector logic
├── agrirover_perception/       # Detection + localization pipeline
├── yolov8n_tomato.pt           # Trained detection model
└── Datasets and Misc.zip       # Training data and supporting files
```

## Future Work

If revisited, the priority order would be:

1. **Motion planning layer** — integrate MoveIt 2 (or equivalent) for IK and collision-aware trajectory generation, replacing the scripted joint commands.
2. **Closed-loop pick-and-place** — wire perception output directly into the planning layer so grasp targets are generated from live detections, not hardcoded.
3. **Navigation** — implement V-SLAM for field traversal, the originally scoped but unstarted subsystem.
4. **Farm management integration** — data logging for yield tracking and harvesting decisions.

## Why This Project Stopped Here

Agrirover was a semester-long academic project, and the timeline ended before the manipulation and navigation subsystems could be fully closed out. Rather than leave it half-documented, this README reflects exactly where it landed: a working, validated perception system, a partially working manipulation pipeline, and a navigation stack that was scoped but not started.

The IK/motion-planning gap here was a direct input into how I approached Project Tvastr afterward.
