# 🧠🤖 Vision-Driven Soft Manipulation

**Integrating Soft Robotics and Computer Vision for Intelligent Environment Interaction**

<img width="8269" height="1894" alt="FrameXDSS-1" src="https://github.com/user-attachments/assets/f53abc78-4211-4f3d-8ae6-f53387186975" />


---

### 📌 Abstract

This project presents a modular robotic framework that integrates:
- Visual SLAM (RTAB-Map)
- Autonomous path planning (ROS Navigation Stack)
- Soft robotic manipulation
- Object detection enhanced by **Vision-Language Models (VLMs)**, including **GPT-4.1 Vision**

Key achievements:
- 98.6% IoU with DINO-DETR  
- GPT-4.1 Vision detected 41 objects (vs. 6 by Detectron2, 16 by MobileNetV4)  
- Effective detection in **low-light** and **unseen-object** scenarios

---

### 🔧 System Overview

- **Hardware**: Quadruped robot with RGB-D camera and soft gripper  
- **Software Stack**: ROS (Navigation, RTAB-Map), Detectron2, GPT-4.1 Vision (via API)  
- **Languages**: Python, C++ (ROS)

---

### 🔍 Key Components

#### 🗺️ Visual SLAM
- RTAB-Map with RGB-D camera only (no wheel encoders, no IMU)
- Planar motion enforced (x, y, yaw) to improve mapping and localization
- 2D occupancy grid + 3D point cloud generation

#### 🧭 Path Planning
- Global path via **Dijkstra** or **A\***
- Local path dynamically updated for real-time obstacle avoidance (via ROS Navigation Stack)
- Goal selection via RViz interface

#### 🧠 Object Detection
- Initial detection via **Detectron2**
- Confidence < 50% → verified or corrected via **GPT-4.1 Vision**
- Robust under:
  - Low light
  - Occlusions
  - Out-of-distribution objects

---

### 📊 Experimental Results

| Model        | IoU (%) | F1 Score | Precision | Recall | Inference Time |
|--------------|---------|----------|-----------|--------|----------------|
| **DINO-DETR**     | 98.6    | **45.45** | 90.91     | 30.3   | 1.94 s         |
| YOLOv8x      | 91.6    | 41.03    | 57.14     | 32.0   | **1.27 s**      |
| Detectron2   | 91.5    | 39.02    | **92.7**   | 24.24  | 1.88 s         |
| YOLOv11x     | 97.3    | 29.27    | 75.0      | 18.18  | 1.32 s         |

#### ✅ GPT-4.1 Vision vs Others

| Model           | Objects Detected |
|------------------|------------------|
| GPT-4.1 Vision   | **41**           |
| MobileNetV4      | 16               |
| Detectron2       | 6                |

#### 💡 Low-Light Detection Example

- Traditional models failed
- GPT-4.1 Vision identified: *monitors, person, tripod, plant*, etc.
- Prompt used: _"List all the objects and the things seen in the picture."_

---

### 🛠️ System Architecture

```text
[RGB-D Camera]
       |
[RTAB-Map SLAM] --> [2D Occupancy Grid] + [3D Point Cloud]
       |
[ROS Navigation Stack] --> [Global & Local Path Planning]
       |
[Detectron2 Detection]
       |
 [GPT-4.1 Vision API] <-- if confidence < 50%
       |
[Soft Manipulator] --> Grasps final object


#### License

Created by: Ibrahim Alsarraj

This project is licensed under the MIT License.


#### Contact

For inquiries and feedback, please contact me on Instagram @d.a.v.i.c.h.i
