# 🐢 TurtleBot3 Custom GUI Tutorials (ROS 2 + PyQt 5)

This repository contains a **series of seven hands-on tutorials** that guide you through building a complete, custom **graphical user interface (GUI)** for controlling and visualizing a TurtleBot3 using **ROS 2** and **PyQt 5**.

By the end of these tutorials, you will have a fully functional **SLAM visualization tool** with tele-operation controls, live plotting, map rendering, and TF-aligned robot tracking — all implemented from scratch in Python 3.

---

## 🚀 Overview

| Tutorial | Title | Key Topics |
|-----------|--------|------------|
| 01 | PyQt App Skeleton | Canvas, event loop, pan & zoom, grid |
| 02 | Tele-op to `/cmd_vel` | ROS 2 + Qt threading, buttons, keyboard input |
| 03 | ODOM Visualization | Subscribing to `/odom`, drawing robot + path |
| 04 | Live Speed Plot | Real-time plotting (pyqtgraph) |
| 05 | Pause & Save | Exporting plots + canvas, non-native dialogs |
| 06 | Sprite Robot | Replacing rectangle with `turtlebot.png` sprite |
| 07 | SLAM GUI (Final) | Map overlay, TF (map ← base_link), full integration |

---

## 🧩 Common Stack

| Tool / Library | Purpose |
|----------------|----------|
| **ROS 2 (Humble +)** | Robot middleware providing `/cmd_vel`, `/odom`, `/map`, and TF transforms. |
| **rclpy** | Python client library for ROS 2 nodes, publishers, subscribers, timers, and parameters. |
| **PyQt 5** | GUI framework (widgets, layouts, event loop, painting, dialogs). |
| **pyqtgraph** | Lightweight plotting library for real-time speed charts. |
| **tf2_ros** | For fetching `map ← base_link` transform from the TF tree. |
| **TurtleBot3** | Mobile robot platform (simulated in Gazebo or physical). |

---

## ⚙️ Environment Setup

Make sure your ROS 2 and Python environment are ready:

```bash
# ROS 2 Humble or newer
source /opt/ros/humble/setup.bash

# Python & dependencies
sudo apt install -y python3-pyqt5 python3-pyqtgraph ros-humble-tf2-ros

# python version=3.10  | Linux ubuntu 22.04  Jammy | ROS2 Humble  | Gazebo  | turtlbot3 burger packages  
# For TurtleBot3 simulations
sudo apt install -y ros-humble-turtlebot3* ros-humble-slam-toolbox
export TURTLEBOT3_MODEL=burger
```

---

## 📚 Tutorials

### 🧱 01 — PyQt App Skeleton
**Goal:**  
Understand how a basic PyQt5 application works — including the **event loop**, **canvas**, and **custom painting**.

**What you’ll learn**
- Creating a `QApplication` and `QMainWindow`
- Building layouts using `QSplitter` and `QVBoxLayout`
- Writing a custom `Canvas` widget with:
  - Pan (mouse drag)
  - Zoom (mouse wheel)
  - World↔View coordinate transforms
  - Grid & axes drawing via `QPainter`

**APIs used**
- `QWidget.paintEvent`
- `QPainter`, `QPen`, `QColor`
- `mousePressEvent`, `wheelEvent`

**ROS used:** ❌ (GUI only)  
**PyQt used:** ✅

---

### 🕹️ 02 — Tele-op to `/cmd_vel`
**Goal:**  
Send velocity commands to the robot using GUI buttons and keyboard keys.

**What you’ll learn**
- Running **ROS 2 nodes in background threads** so the Qt event loop remains responsive.
- Publishing `geometry_msgs/Twist` messages to `/cmd_vel`.
- Connecting `QPushButton.clicked` and `keyPressEvent` to actions.

**APIs used**
- `rclpy.Node.create_publisher`
- `rclpy.create_timer`
- `threading.Thread`
- Qt `Signal` → `Slot` connections

**ROS used:** ✅ (`/cmd_vel`)  
**PyQt used:** ✅  

---

### 🧭 03 — ODOM Visualization
**Goal:**  
Visualize the robot’s position and orientation using `/odom`.

**What you’ll learn**
- Subscribing to `nav_msgs/Odometry`
- Extracting yaw from a quaternion
- Drawing:
  - Rectangle robot footprint
  - Heading arrow
  - Breadcrumb path (stored points)
- Auto-centering the canvas on the robot (follow mode)

**APIs used**
- `rclpy.Node.create_subscription`
- `QPainter.drawPolygon`, `QPainter.drawLine`

**ROS used:** ✅ (`/cmd_vel`, `/odom`)  
**PyQt used:** ✅  

---

### 📈 04 — Live Speed Plot
**Goal:**  
Add a live **speed-versus-time plot** of the robot’s linear velocity.

**What you’ll learn**
- Integrating `pyqtgraph` with PyQt.
- Maintaining a fixed-length buffer (`collections.deque`) of data.
- Updating plots periodically via `QTimer`.
- Multi-threaded GUI + ROS integration.

**APIs used**
- `pyqtgraph.PlotWidget`
- `QTimer.timeout.connect`

**ROS used:** ✅ (`/odom`, `/cmd_vel`)  
**PyQt used:** ✅  

---

### 💾 05 — Pause + Save Plot + Save Canvas
**Goal:**  
Export GUI visuals and plots to files and pause live updates.

**What you’ll learn**
- Using `QFileDialog` safely on Wayland with `DontUseNativeDialog`.
- Capturing widgets (`QWidget.grab()`) as images.
- Pausing/resuming data updates.
- Clean shutdown of threads and ROS nodes.

**APIs used**
- `QFileDialog`, `QMessageBox`
- `QTimer.singleShot`
- `QApplication.aboutToQuit`
- `rclpy.shutdown`

**ROS used:** ✅  
**PyQt used:** ✅  

---

### 🤖 06 — Sprite Robot (PNG)
**Goal:**  
Replace the rectangle with a **robot sprite image** for better visualization.

**What you’ll learn**
- Loading images (`QPixmap`) and transforming them (`QTransform`).
- Scaling meters → pixels to maintain physical proportions.
- Implementing a **rotation cache** (quantized heading) for smoother performance.
- Retaining all 05 features (tele-op, HUD, plot, save, shutdown).

**APIs used**
- `QPixmap.scaled`, `QPixmap.transformed`
- `QTransform.rotate`

**ROS used:** ✅  
**PyQt used:** ✅  

> 💡 Tip: place a top-down `turtlebot.png` next to the script (front pointing upward).

---

### 🗺️ 07 — SLAM GUI (Final)
**Goal:**  
Integrate SLAM map visualization (from `/map`) and TF alignment.

**What you’ll learn**
- Subscribing to `nav_msgs/OccupancyGrid` and converting map data into a `QImage`.
- Drawing the map raster with correct scaling, rotation, and transparency.
- Querying TF (`map ← base_link` or `base_footprint`) using `tf2_ros.Buffer` and `TransformListener`.
- Accumulating the robot path directly in the **MAP** frame.
- Full-featured GUI combining all previous lessons.

**APIs used**
- `tf2_ros.Buffer.lookup_transform`
- `QImage.Format_RGBA8888`
- `QPainter.scale`, `QPainter.rotate`, `QPainter.drawImage`
- `rclpy.time.Time`

**ROS used:** ✅ (`/cmd_vel`, `/odom`, `/map`, TF)  
**PyQt used:** ✅  

**Typical launch sequence**
```bash
# Terminal 1 – Gazebo sim
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2 – SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# Terminal 3 – Run GUI
/usr/bin/python3 07_tb3_pyqt_gui_slam.py
```

---

## 🧠 Key Concepts Reinforced

- **Event-driven architecture:** Qt event loop (no while True loops).
- **Threading:** Running ROS spin() in a background thread for a responsive GUI.
- **Decoupling:** Separate drawing (canvas) from data (ROS node).
- **Transformations:** Mapping between world meters and canvas pixels.
- **Safety:** Clean shutdown and proper resource handling.

---

## 🧩 Directory Structure

```
.
├── 01_qt_skeleton.py
├── 02_teleop_cmdvel.py
├── 03_odom_rect_viz.py
├── 04_live_speed_plot.py
├── 05_save_pause.py
├── 06_sprite.py
├── 07_tb3_pyqt_gui_slam.py
├── turtlebot.png          # optional sprite
└── README.md
```

---

## 🛠️ Troubleshooting

| Problem | Fix |
|----------|-----|
| No window appears | Run with `/usr/bin/python3` (system Python with PyQt5). |
| App freezes on “Save” | Wayland issue → we already force `DontUseNativeDialog=True`. |
| Map offset / rotated | Ensure you use `p.drawImage(0, 0, img)` (top-left anchor) after `p.scale(s, -s)`. |
| Path not drawn | Make sure `last = pt` updates every iteration in `_path()` loop. |
| TF unavailable | Check TF tree: `ros2 run tf2_tools view_frames` or `ros2 run tf2_ros tf2_echo map base_link`. |

---

## 📜 License
This tutorial is supplementary material for coure ROBO 202 Software Development for Robotics
This tutorial set is for educational and research use.  
You’re free to reuse or modify it for your robotics or GUI development classes.

---

## ✍️ Author & Acknowledgements

Developed for the **TurtleBot3 Custom Data Visualization & SLAM GUI** lecture.
Under the course of ROBO 202 Software Development for Robotics.
Made by **Abdulrahman Hamdy Ahmad (Abdulrahman Ahmad)**.  
Combines core concepts from **ROS 2**, **PyQt 5**, **rclpy**, and **tf2_ros**.  
Special thanks to the open-source ROS community and the TurtleBot3 developers.
