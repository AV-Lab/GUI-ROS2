#!/usr/bin/env python3
"""
02 — Tele‑op publisher to /cmd_vel
Goal:
  - Start a ROS 2 node inside a PyQt app (spin in a background thread).
  - Publish Twist to /cmd_vel at 50 Hz.
  - Buttons & WASD/arrow keys to set velocities.

Run:
  source /opt/ros/humble/setup.bash
  /usr/bin/python3 02_teleop_cmdvel.py
"""

import math, threading
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPainter, QPen, QColor
from PyQt5.QtWidgets import *
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

# --- Canvas reused from step 01 ------------

class Canvas(QWidget):
    """Simple world space canvas with pan+zoom and a metric grid."""
    def __init__(self):
        super().__init__()
        self.scale = 80.0     # pixels per meter
        self.offset_x = 0.0   # view offset in pixels
        self.offset_y = 0.0
        self._dragging = False
        self._last_mouse = None

    # -------- coordinate transforms (world meters <-> view pixels) ----------
    def world_to_view(self, x, y):
        return x * self.scale + self.offset_x, -y * self.scale + self.offset_y

    def view_to_world(self, vx, vy):
        return (vx - self.offset_x) / self.scale, -(vy - self.offset_y) / self.scale

    # ---------------- interactions: wheel zoom / mouse drag ------------------
    def wheelEvent(self, e):
        factor = 1.15 if e.angleDelta().y() > 0 else 1/1.15
        # Update scale but keep the world point under the mouse fixed on screen
        mx, my = e.x(), e.y()
        wx, wy = self.view_to_world(mx, my)
        self.scale = max(10.0, min(600.0, self.scale * factor))
        self.offset_x = mx - wx * self.scale
        self.offset_y = my + wy * self.scale
        self.update()

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._dragging = True
            self._last_mouse = e.pos()

    def mouseMoveEvent(self, e):
        if self._dragging:
            d = e.pos() - self._last_mouse
            self.offset_x += d.x()
            self.offset_y += d.y()
            self._last_mouse = e.pos()
            self.update()

    def mouseReleaseEvent(self, _):  # end drag
        self._dragging = False
        self._last_mouse = None

    def center_on(self, x, y):
        """Center the view on a world point (x,y) in meters."""
        w = self.width() or 800
        h = self.height() or 600
        cx = x * self.scale
        cy = -y * self.scale
        self.offset_x = w/2 - cx
        self.offset_y = h/2 - cy
        self.update()

    # -------------------------------- painting --------------------------------
    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QColor('white'))

        # Visible world-rect in meters (for grid culling)
        w = self.width() or 800
        h = self.height() or 600
        x_min, y_max = self.view_to_world(0, 0)
        x_max, y_min = self.view_to_world(w, h)

        # Grid spacing ~>=30px on screen (in meters)
        spacing = 1.0
        while spacing * self.scale < 30:
            spacing *= 2.0

        # Draw grid
        p.setPen(QPen(QColor(230, 230, 230)))
        xx = math.floor(x_min / spacing) * spacing
        while xx <= x_max:
            x1, y1 = self.world_to_view(xx, y_min)
            x2, y2 = self.world_to_view(xx, y_max)
            p.drawLine(QPointF(x1, y1), QPointF(x2, y2))
            xx += spacing

        yy = math.floor(y_min / spacing) * spacing
        while yy <= y_max:
            x1, y1 = self.world_to_view(x_min, yy)
            x2, y2 = self.world_to_view(x_max, yy)
            p.drawLine(QPointF(x1, y1), QPointF(x2, y2))
            yy += spacing

        # Axes
        p.setPen(QPen(QColor(220, 51, 51), 2))  # X (red)
        x1, y1 = self.world_to_view(-1e6, 0)
        x2, y2 = self.world_to_view(1e6, 0)
        p.drawLine(QPointF(x1, y1), QPointF(x2, y2))
        p.setPen(QPen(QColor(51, 102, 220), 2))  # Y (blue)
        x1, y1 = self.world_to_view(0, -1e6)
        x2, y2 = self.world_to_view(0, 1e6)
        p.drawLine(QPointF(x1, y1), QPointF(x2, y2))


# --- ROS Node that ONLY publishes /cmd_vel at a fixed rate --------------------
class TeleopNode(Node):
    def __init__(self):
        super().__init__('tb3_teleop_gui')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.des_v = 0.0  # desired linear x
        self.des_w = 0.0  # desired angular z
        # Timer at 50 Hz so Gazebo/robot never times out
        self.create_timer(0.02, self._publish)

    def _publish(self):
        msg = Twist()
        msg.linear.x = float(self.des_v)
        msg.angular.z = float(self.des_w)
        self.pub.publish(msg)


def spin_ros(node: Node):
    """Spin ROS in a background thread (Qt event loop stays responsive)."""
    rclpy.spin(node)

# --- Main window with buttons and keyboard control --------------------------
class Main(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("02 — Tele‑op to /cmd_vel")
        self.resize(950, 650)

        # 1) Bring up ROS and start spinning in the background
        rclpy.init()
        self.node = TeleopNode()
        threading.Thread(target=spin_ros, args=(self.node,), daemon=True).start()

        # 2) Build UI
        splitter = QSplitter(Qt.Horizontal, self); self.setCentralWidget(splitter)
        left = QWidget(); L = QVBoxLayout(left)

        # Speed sliders (set max speeds we will apply when clicking the buttons)
        self.max_lin = 0.22; self.max_ang = 1.80
        sp = QGroupBox("Speeds"); spL = QVBoxLayout(sp)
        # Linear
        row1 = QHBoxLayout(); self.lbl_lin = QLabel(f"{self.max_lin:.2f} m/s")
        s1 = QSlider(Qt.Horizontal); s1.setRange(0,100); s1.setValue(int(self.max_lin*100))
        s1.valueChanged.connect(lambda v:(setattr(self,'max_lin',v/100.0), self.lbl_lin.setText(f"{self.max_lin:.2f} m/s")))
        row1.addWidget(QLabel("Max Linear")); row1.addWidget(self.lbl_lin, 1, Qt.AlignRight)
        spL.addLayout(row1); spL.addWidget(s1)
        # Angular
        row2 = QHBoxLayout(); self.lbl_ang = QLabel(f"{self.max_ang:.2f} rad/s")
        s2 = QSlider(Qt.Horizontal); s2.setRange(0,300); s2.setValue(int(self.max_ang/3.0*300))
        s2.valueChanged.connect(lambda v:(setattr(self,'max_ang', (v/300.0)*3.0), self.lbl_ang.setText(f"{self.max_ang:.2f} rad/s")))
        row2.addWidget(QLabel("Max Angular")); row2.addWidget(self.lbl_ang, 1, Qt.AlignRight)
        spL.addLayout(row2); spL.addWidget(s2)
        L.addWidget(sp)

        # Teleop buttons (set desired speeds; ROS timer publishes continuously)
        tele = QGroupBox("Tele‑op"); grid = QGridLayout(tele)
        def cmd(v,w): self.node.des_v=v; self.node.des_w=w
        bF=QPushButton("Forward"); bB=QPushButton("Back"); bL=QPushButton("Left"); bR=QPushButton("Right"); bS=QPushButton("Stop")
        bF.clicked.connect(lambda:cmd(self.max_lin,0)); bB.clicked.connect(lambda:cmd(-self.max_lin,0))
        bL.clicked.connect(lambda:cmd(0,self.max_ang)); bR.clicked.connect(lambda:cmd(0,-self.max_ang))
        bS.clicked.connect(lambda:cmd(0,0))
        grid.addWidget(bF,0,1); grid.addWidget(bL,1,0); grid.addWidget(bS,1,1); grid.addWidget(bR,1,2); grid.addWidget(bB,2,1)
        L.addWidget(tele); L.addStretch(1)
        splitter.addWidget(left)

        # Right: the drawing canvas (just a grid in this step)
        self.canvas = Canvas(); splitter.addWidget(self.canvas); splitter.setStretchFactor(1,1)

        # Keyboard shortcuts (WASD/Arrows/Space)
        self.setFocusPolicy(Qt.StrongFocus)

    def keyPressEvent(self, e):
        k = e.key()
        if   k in (Qt.Key_W, Qt.Key_Up):    self.node.des_v=self.max_lin;  self.node.des_w=0
        elif k in (Qt.Key_S, Qt.Key_Down):  self.node.des_v=-self.max_lin; self.node.des_w=0
        elif k in (Qt.Key_A, Qt.Key_Left):  self.node.des_v=0; self.node.des_w=self.max_ang
        elif k in (Qt.Key_D, Qt.Key_Right): self.node.des_v=0; self.node.des_w=-self.max_ang
        elif k == Qt.Key_Space:             self.node.des_v=0; self.node.des_w=0
        else:
            super().keyPressEvent(e)
        
    # Catch window close: perform clean shutdown *before* letting Qt exit
    def closeEvent(self, event):
        self._clean_shutdown()
        event.accept()
        
    def _clean_shutdown(self):
        """Idempotent: safe to call multiple times."""
        if getattr(self, "_shutting_down", False):
            return
        self._shutting_down = True
        try:
            # stop the robot
            self.node.des_v = 0.0; self.node.des_w = 0.0
            time.sleep(0.05)

            # ask rclpy.spin() to return, then join the thread
            if rclpy.ok():
                rclpy.shutdown()

            if hasattr(self, "ros_thread") and self.ros_thread.is_alive():
                self.ros_thread.join(timeout=2.0)

            # destroy the node (after spin stops is fine)
            try:
                self.node.destroy_node()
            except Exception:
                pass
        except Exception:
            pass

if __name__ == "__main__":
    app = QApplication([])
    win = Main(); win.show()
    app.exec_()
