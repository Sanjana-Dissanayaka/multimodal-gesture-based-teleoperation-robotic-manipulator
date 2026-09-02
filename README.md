# Multimodal Gesture-Based Teleoperation Framework for Low-Cost Robotic Manipulators

A low-cost gesture-based robotic teleoperation framework designed to provide an intuitive human–robot interaction interface using a wearable sensor glove.

The current system combines an **MPU6050 inertial measurement unit (IMU)** and **flex sensors** with calibration, filtering, gesture recognition, VPython hand visualization, and inverse-kinematics-based robotic manipulator control.

The framework is being extended toward a **hybrid multimodal teleoperation system** by integrating computer-vision-based hand tracking and physical robotic manipulator validation.

---

## Project Overview

Conventional robotic manipulator teleoperation interfaces such as joysticks and teach pendants can be less intuitive when operators need to perform natural hand-based manipulation.

Gesture-based interfaces provide a more natural method of communicating human intent to a robotic system. However, systems that depend on only one sensing method may experience problems such as:

* Sensor drift
* Measurement noise
* Camera occlusion
* Lighting variation
* Limited gesture information
* High implementation cost
* Reduced teleoperation accuracy

This project investigates a low-cost teleoperation framework that initially uses a wearable **IMU + flex-sensor glove** and is designed to later combine wearable sensing with computer vision.

---

## Overall Objective

Develop and validate a low-cost hybrid teleoperation system that integrates an **IMU + flex-sensor glove** with computer-vision-based hand tracking to achieve robust, accurate, and intelligent robotic manipulator control.

---

## System Architecture
<img width="660" height="333" alt="Screenshot 2026-09-02 102238" src="https://github.com/user-attachments/assets/62be03dd-69fc-470f-a8f6-e1de5f6c68d2" />

The current teleoperation pipeline can be represented as:

```text
Human Hand
    │
    ▼
IMU + Flex Sensor Glove
    │
    ▼
Microcontroller / Data Acquisition
    │
    ▼
Calibration + Filtering
    │
    ▼
Gesture Recognition + Orientation Estimation
    │
    ├──────────────► VPython Virtual Hand
    │
    ▼
Gesture / Orientation Command Mapping
    │
    ▼
Inverse Kinematics
    │
    ▼
Robotic Manipulator Simulation
    │
    ▼
Sorting / Pick-and-Place Tasks
```

The planned multimodal extension is:

```text
Wearable Sensor Glove ──────┐
                            │
                            ▼
                    Hybrid Sensor Fusion
                            ▲
                            │
RGB Camera + MediaPipe ─────┘
                            │
                            ▼
                Robotic Manipulator Control
```

---

## Current Implementation

### 1. Wearable Sensing Layer

The wearable glove provides information about hand orientation and finger movement.

**MPU6050 IMU**

Used to estimate:

* Roll
* Pitch
* Yaw

**Flex Sensors**

Used to detect:

* Finger bending
* Open-hand gestures
* Closed-hand gestures
* Finger-count gestures
* Object-selection commands

---

## 2. Sensor Acquisition and Stabilization

Raw sensor data can contain noise, offsets, and small fluctuations.

The system therefore applies:

* Startup neutral calibration
* Flex-sensor calibration
* IMU calibration
* Deadband processing
* First-order smoothing
* Gesture stabilization
* Threshold-based gesture classification

These techniques improve the stability of commands before they are transmitted to the robotic manipulator.

---

## 3. Flex-Sensor Gesture Validation

The current gesture-validation criteria include:

| Condition           | Approximate Threshold |
| ------------------- | --------------------: |
| Straight finger     |                 ≤ 25° |
| Bent finger         |                 ≥ 60° |
| Stable gesture hold |                 0.5 s |

A gesture must remain sufficiently stable before it is accepted as a control command.

This helps prevent accidental commands caused by temporary sensor fluctuations.

---

## 4. Gesture Mapping

Finger gestures are mapped to object-selection and manipulation commands.

| Gesture   | Command              |
| --------- | -------------------- |
| 1 finger  | Select RED object    |
| 2 fingers | Select BLUE object   |
| 3 fingers | Select GREEN object  |
| 4 fingers | Select YELLOW object |
| Fist      | PICK                 |
| Open palm | PLACE / RELEASE      |

This allows the operator to select and manipulate objects using natural hand gestures.

---

## 5. IMU Orientation Mapping

The MPU6050 provides orientation information for controlling the manipulator end effector.

| Human-Hand Motion | Robot Command      |
| ----------------- | ------------------ |
| Roll              | End-effector roll  |
| Pitch             | End-effector pitch |
| Yaw               | End-effector yaw   |

Control limits are applied to prevent unrealistic or unstable manipulator commands.

### Current Safety Limits

| Motion       | Limit |
| ------------ | ----: |
| Roll         |  ±35° |
| Pitch        |  ±35° |
| Yaw          |  ±60° |
| Command rate | 25 Hz |

---

## 6. Virtual Hand Validation

Before controlling the robotic manipulator, sensor information is visualized using **VPython**.

The virtual-hand environment is used to validate:

* Finger bending
* Hand orientation
* Open-palm detection
* Fist detection
* Finger-count gestures
* Sensor calibration
* Gesture stability

This provides an intermediate validation stage between the physical glove and robotic manipulator.

---

## 7. Robotic Manipulator Simulation

Robotic manipulation is investigated using simulation environments such as:

* PyBullet
* CoppeliaSim

The simulation stage includes:

* Manipulator kinematics
* Forward kinematics
* Inverse kinematics
* End-effector orientation control
* Gesture-to-command mapping
* Object selection
* Pick-and-place operations
* Object sorting
* Contact-based manipulation experiments

---

## Inverse Kinematics

The desired end-effector pose generated from the gesture interface is converted into manipulator joint commands using inverse kinematics.

The general control process is:

```text
Human Gesture
      ↓
Target End-Effector Pose
      ↓
Inverse Kinematics
      ↓
Required Joint Angles
      ↓
Manipulator Motion
```

Inverse kinematics allows intuitive hand commands to be translated into feasible robotic manipulator movements.

---

## Hardware

The project hardware platform includes or investigates:

* MPU6050 IMU
* Flex sensors
* Wearable glove
* Microcontroller
* Custom PCB
* CH340G USB communication interface
* Flex-sensor interface circuitry
* Sensor wiring and connectors
* Robotic manipulator for final physical testing
* RGB camera for the planned vision stage

---

## Software and Development Tools

### Current Tools

* Python
* VPython
* PyBullet
* CoppeliaSim
* Arduino IDE
* KiCad
* Git
* GitHub

### Planned Vision Tools

* OpenCV
* MediaPipe

### Analysis and Evaluation

* Python
* MATLAB

---

## Major Development Phases

### Phase 1 – Wearable Hardware Development

* System architecture development
* PCB design
* Embedded firmware
* Hardware integration

### Phase 2 – Sensor Acquisition and Calibration

* MPU6050 calibration
* Flex-sensor calibration
* Noise filtering
* Gesture stabilization
* Wearable glove fabrication

### Phase 3 – Virtual Hand and Manipulator Validation

* VPython visualization
* Gesture mapping
* Teleoperation data acquisition
* Manipulator simulation
* Inverse-kinematics control

### Phase 4 – Vision-Based Teleoperation

Planned development includes:

* MediaPipe hand tracking
* Vision gesture recognition
* Vision-only manipulator control
* Performance comparison with the sensor glove

### Phase 5 – Hybrid Sensor Fusion

The future multimodal framework will combine:

```text
IMU
 +
Flex Sensors
 +
Computer Vision
        ↓
Hybrid Gesture Estimation
        ↓
Robotic Manipulator Control
```

The aim is to improve robustness against problems such as sensor drift and camera occlusion.

### Phase 6 – Physical Robot Integration

The final stage is intended to include:

* Physical robotic manipulator integration
* Wireless master–slave communication
* Servo-control optimization
* Real-time physical teleoperation
* Pick-and-place experiments
* Robustness testing
* Latency measurement
* Trajectory analysis
* Statistical evaluation

---

## Key Features

* Low-cost wearable gesture interface
* MPU6050-based hand orientation estimation
* Flex-sensor finger-motion measurement
* Sensor calibration
* Deadband processing
* Signal smoothing
* Stable gesture detection
* VPython virtual-hand visualization
* Gesture-labelled object selection
* Inverse-kinematics-based manipulator control
* PyBullet/CoppeliaSim simulation
* Pick-and-place control
* Object sorting
* Extendable multimodal architecture
* Planned MediaPipe computer-vision integration
* Designed for physical robotic-manipulator validation

---

## Research Contribution

The current framework provides a practical foundation for an intuitive and cost-effective human–robot teleoperation interface.

The main research direction is the development of a multimodal system that combines the advantages of wearable sensors and computer vision.

Potential benefits include:

* Improved teleoperation robustness
* Reduced dependence on conventional joysticks
* Natural human–robot interaction
* Improved resistance to camera occlusion
* Compensation for wearable-sensor drift
* Low implementation cost
* Flexible gesture-to-robot mapping

---

---
 ## Project Status

### Completed / Demonstrated

* Wearable gesture-glove prototype
* MPU6050 sensor acquisition
* Flex-sensor acquisition
* Sensor calibration
* Signal smoothing and deadband
* Gesture recognition
* VPython virtual-hand visualization
* Gesture-to-command mapping
* Manipulator simulation
* Inverse-kinematics-based control
* Gesture-controlled sorting and pick-and-place demonstrations

### Next Development Stage

* RGB-camera integration
* MediaPipe hand tracking
* Vision-only teleoperation
* Comparison between glove and vision systems
* Hybrid sensor fusion
* Physical robotic manipulator integration
* Experimental performance evaluation

---

## Future Improvements

* MediaPipe hand-landmark tracking
* Adaptive multimodal sensor fusion
* Vision and wearable-sensor synchronization
* Automatic sensor-drift compensation
* Kalman or complementary filtering
* Improved gesture-classification algorithms
* Wireless glove communication
* Physical 6-DoF manipulator integration
* End-effector camera feedback
* Collision avoidance
* Workspace constraints
* Trajectory smoothing
* Latency optimization
* Gesture confusion-matrix analysis
* Quantitative trajectory-error analysis
* User-based teleoperation experiments

---

## Contributors

### Lowe W.N.D.

**Registration No.: 2022/E/172**

Primary areas include:

* Embedded electronics
* PCB development
* IMU and flex-sensor calibration
* Python development
* Sensor fusion
* Teleoperation implementation
* Experimental evaluation

### Dissanayaka W.A.M.S.K.

**Registration No.: 2022/E/174**

Primary areas include:

* VPython visualization
* PyBullet simulation
* Computer vision
* MediaPipe implementation
* Mechanical fabrication
* Sensor-fusion development
* Experimental evaluation
* Documentation

---

## Supervisors

**Supervisor:** Mr. M. Barathraj
**Co-Supervisor:** Mr. P. Ravivarman

Department of Electrical & Electronic Engineering
Faculty of Engineering
University of Jaffna

---

## License

An open-source license has not yet been specified for this research project.

Because the repository contains collaborative university research work, an appropriate license should be selected only after agreement among the project contributors and relevant supervisors.

---

## Acknowledgements

This project is being developed as part of the undergraduate research work of the **Department of Electrical & Electronic Engineering, Faculty of Engineering, University of Jaffna**.

The authors acknowledge the guidance and support provided by the project supervisor and co-supervisor throughout the development of the gesture-based robotic teleoperation framework.

---

**Project:** Gesture-Based Teleoperation of a Robotic Manipulator
**Research Direction:** Low-Cost Hybrid Gesture-Based Teleoperation Using Wearable Sensor Fusion and Computer Vision

