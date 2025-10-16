#!/usr/bin/env python3
"""
03 — ODOM Visualization (PyQt5)

What this app does:
  • Starts a PyQt window with a canvas (pan/zoom grid & axes).
  • Subscribes to /odom to get robot pose and velocity.
  • Draws a rectangle footprint + heading and a breadcrumb path.
  • Tele-op via buttons and WASD/Arrow keys (publishes /cmd_vel at 50 Hz).
  • Safe shutdown on close (no Ubuntu "app crashed" dialog).

Run:
  source /opt/ros/humble/setup.bash
  /usr/bin/python3 03_odom_rect_viz.py

Dependencies:
  sudo apt install -y python3-pyqt5
"""

import math
import threading
import time

from PyQt5.QtCore import Qt, QPointF, QTimer
from PyQt5.QtGui import QPainter, QPen, QColor
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QSplitter, QVBoxLayout, QHBoxLayout,
    QGridLayout, QPushButton, QGroupBox, QLabel
)

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


# ========================= CANVAS (Base from Step-01, extended for Step-03) =========================
class Canvas(QWidget):
    """
    Canvas v1 (Step-01) with Step-03 additions:
      - world<->view transforms, pan/zoom, grid & axes
      - state provider hook (x,y,yaw,path) in ODOM frame
      - draw path + rectangle robot + heading
      - optional follow mode to keep robot centered
    """
    def __init__(self):
        super().__init__()
        # World↔View: meters ↔ pixels
        self.scale = 80.0
        self.offset_x = 0.0
        self.offset_y = 0.0

        # Interactions
        self._dragging = False
        self._last_mouse = None

        # Feature toggles
        self.show_grid = True
        self.follow = True

        # Data hooks (provided by the main window)
        self._state_provider = None  # fn() -> (x, y, yaw, path_list)

    # --------- hooks to connect ROS state ----------
    def set_state_provider(self, fn):
        """fn() -> (x, y, yaw, path[list[(x,y)]]) in ODOM frame."""
        self._state_provider = fn

    # ---------- coordinate transforms ----------
    def world_to_view(self, x, y):
        return x * self.scale + self.offset_x, -y * self.scale + self.offset_y

    def view_to_world(self, vx, vy):
        return (vx - self.offset_x) / self.scale, -(vy - self.offset_y) / self.scale

    # ---------- pan / zoom ----------
    def wheelEvent(self, e):
        factor = 1.15 if e.angleDelta().y() > 0 else 1/1.15
        mx, my = e.x(), e.y()
        wx, wy = self.view_to_world(mx, my)              # keep mouse-world point fixed
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

    def mouseReleaseEvent(self, _):
        self._dragging = False
        self._last_mouse = None

    def center_on(self, x, y):
        """Center view on world (x,y) meters."""
        w = self.width() or 800
        h = self.height() or 600
        cx = x * self.scale
        cy = -y * self.scale
        self.offset_x = w/2 - cx
        self.offset_y = h/2 - cy
        self.update()

    # ---------- painting ----------
    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QColor('white'))

        # (1) GRID
        if self.show_grid:
            self._draw_grid(p)

        # (2) AXES
        self._draw_axes(p)

        # (3) PATH + ROBOT (ODOM frame in this tutorial)
        if self._state_provider:
            x, y, yaw, path = self._state_provider()
            if self.follow:
                self.center_on(x, y)
            self._draw_path(p, path)
            self._draw_robot_rect(p, x, y, yaw)

    # ---------- helpers ----------
    def _draw_grid(self, p: QPainter):
        w = self.width() or 800
        h = self.height() or 600
        x_min, y_max = self.view_to_world(0, 0)
        x_max, y_min = self.view_to_world(w, h)

        spacing = 1.0
        while spacing * self.scale < 30:
            spacing *= 2.0

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

    def _draw_axes(self, p: QPainter):
        p.setPen(QPen(QColor(220, 51, 51), 2))  # X
        x1, y1 = self.world_to_view(-1e6, 0)
        x2, y2 = self.world_to_view( 1e6, 0)
        p.drawLine(QPointF(x1, y1), QPointF(x2, y2))
        p.setPen(QPen(QColor(51, 102, 220), 2)) # Y
        x1, y1 = self.world_to_view(0, -1e6)
        x2, y2 = self.world_to_view(0,  1e6)
        p.drawLine(QPointF(x1, y1), QPointF(x2, y2))

    def _draw_path(self, p: QPainter, path):
        if not path or len(path) < 2:
            return
        pen = QPen(QColor(0, 160, 0)); pen.setWidth(2)
        p.setPen(pen)
        last = None
        for px, py in path:
            vx, vy = self.world_to_view(px, py)
            pt = QPointF(vx, vy)
            if last is not None:
                p.drawLine(last, pt)
            last = pt

    def _draw_robot_rect(self, p: QPainter, x, y, yaw, L=0.18, W=0.14):
        """Draw a rectangle footprint and a heading line. TB3 approx size."""
        corners = [(-W/2,-L/2),(W/2,-L/2),(W/2,L/2),(-W/2,L/2)]
        c, s = math.cos(yaw), math.sin(yaw)
        poly = []
        for dx, dy in corners:
            gx = x + dx*c - dy*s
            gy = y + dx*s + dy*c
            vx, vy = self.world_to_view(gx, gy)
            poly.append(QPointF(vx, vy))
        p.setPen(QPen(Qt.black, 1))
        p.setBrush(QColor(135, 206, 235))
        p.drawPolygon(*poly)

        # Heading (front)
        fx = x + (L/2) * math.cos(yaw)
        fy = y + (L/2) * math.sin(yaw)
        cfx, cfy = self.world_to_view(fx, fy)
        cx, cy = self.world_to_view(x, y)
        p.setPen(QPen(QColor(255, 165, 0), 2))
        p.drawLine(QPointF(cx, cy), QPointF(cfx, cfy))


# ================================ ROS NODE ====================================
def yaw_from_quat(x, y, z, w):
    """Extract yaw (Z) from quaternion (ROS ZYX convention)."""
    return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))


class OdomTeleopNode(Node):
    """
    - Publishes /cmd_vel at 50 Hz (using desired v,w set by UI).
    - Subscribes to /odom to track pose and velocity.
    - Accumulates a breadcrumb path (downsampled).
    """
    def __init__(self):
        super().__init__('tb3_odom_gui')

        # Publisher: tele-op
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.des_v = 0.0
        self.des_w = 0.0
        self.create_timer(0.02, self._publish_cmd)  # 50 Hz

        # Subscriber: odometry
        self.create_subscription(Odometry, '/odom', self._odom_cb, 50)

        # Current state from /odom
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v_lin = 0.0
        self.v_ang = 0.0

        # Path (downsample to ~3 cm)
        self.path = []
        self._last_xy = None
        self._min_spacing = 0.03
        self._max_pts = 3000

        # TB3 footprint (meters) — used by Canvas for proportions (docs only here)
        self.length = 0.18
        self.width = 0.14

    def _publish_cmd(self):
        msg = Twist()
        msg.linear.x = float(self.des_v)
        msg.angular.z = float(self.des_w)
        self.pub.publish(msg)

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.v_lin = msg.twist.twist.linear.x
        self.v_ang = msg.twist.twist.angular.z

        if (self._last_xy is None or
            (self.x - self._last_xy[0])**2 + (self.y - self._last_xy[1])**2 >= self._min_spacing**2):
            self.path.append((self.x, self.y))
            if len(self.path) > self._max_pts:
                self.path.pop(0)
            self._last_xy = (self.x, self.y)


def spin_ros(node: Node):
    """Spin rclpy in a background thread. Returns when rclpy.shutdown() is called."""
    rclpy.spin(node)


# ================================= MAIN WINDOW ================================
class Main(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("03 — ODOM Visualization")
        self.resize(1100, 740)

        # Start ROS (init here so we can cleanly shutdown later)
        rclpy.init()
        self.node = OdomTeleopNode()
        self.ros_thread = threading.Thread(target=spin_ros, args=(self.node,), daemon=False)
        self.ros_thread.start()

        # UI layout
        splitter = QSplitter(Qt.Horizontal, self)
        self.setCentralWidget(splitter)

        # ---------- Left panel: Tele-op + HUD ----------
        left = QWidget(); L = QVBoxLayout(left)

        tele = QGroupBox("Tele-op")
        grid = QGridLayout(tele)
        def set_cmd(v, w):
            self.node.des_v = v
            self.node.des_w = w
        bF = QPushButton("Forward")
        bB = QPushButton("Back")
        bL = QPushButton("Left")
        bR = QPushButton("Right")
        bS = QPushButton("Stop")
        bF.clicked.connect(lambda: set_cmd(0.22, 0.0))
        bB.clicked.connect(lambda: set_cmd(-0.22, 0.0))
        bL.clicked.connect(lambda: set_cmd(0.0, 1.8))
        bR.clicked.connect(lambda: set_cmd(0.0, -1.8))
        bS.clicked.connect(lambda: set_cmd(0.0, 0.0))
        grid.addWidget(bF, 0, 1)
        grid.addWidget(bL, 1, 0)
        grid.addWidget(bS, 1, 1)
        grid.addWidget(bR, 1, 2)
        grid.addWidget(bB, 2, 1)
        L.addWidget(tele)

        hud = QGroupBox("Robot State (/odom)")
        hL = QVBoxLayout(hud)
        self.lbl_pose = QLabel("Pose: x=0.00  y=0.00  yaw=0.0°")
        self.lbl_vel  = QLabel("Vel : lin=0.00 m/s  ang=0.00 rad/s")
        hL.addWidget(self.lbl_pose)
        hL.addWidget(self.lbl_vel)
        L.addWidget(hud)

        L.addStretch(1)
        splitter.addWidget(left)

        # ---------- Right panel: Canvas ----------
        self.canvas = Canvas()
        # Provide ODOM state to the canvas
        self.canvas.set_state_provider(lambda: (self.node.x, self.node.y, self.node.yaw, list(self.node.path)))
        splitter.addWidget(self.canvas)
        splitter.setStretchFactor(1, 1)

        # Keep keyboard focus for tele-op keys
        self.setFocusPolicy(Qt.StrongFocus)

        # GUI refresh timer (~10 Hz)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(100)

        # Ensure clean shutdown if app quits by WM/menu
        QApplication.instance().aboutToQuit.connect(self._clean_shutdown)

    # Tele-op keys
    def keyPressEvent(self, e):
        k = e.key()
        if   k in (Qt.Key_W, Qt.Key_Up):    self.node.des_v = 0.22; self.node.des_w = 0.0
        elif k in (Qt.Key_S, Qt.Key_Down):  self.node.des_v = -0.22; self.node.des_w = 0.0
        elif k in (Qt.Key_A, Qt.Key_Left):  self.node.des_v = 0.0;  self.node.des_w = 1.8
        elif k in (Qt.Key_D, Qt.Key_Right): self.node.des_v = 0.0;  self.node.des_w = -1.8
        elif k == Qt.Key_Space:             self.node.des_v = 0.0;  self.node.des_w = 0.0
        else:
            super().keyPressEvent(e)

    # GUI timer: repaint + HUD
    def _tick(self):
        self.canvas.update()
        self.lbl_pose.setText(
            f"Pose: x={self.node.x:.2f}  y={self.node.y:.2f}  yaw={math.degrees(self.node.yaw):.1f}°"
        )
        self.lbl_vel.setText(
            f"Vel : lin={self.node.v_lin:.2f} m/s  ang={self.node.v_ang:.2f} rad/s"
        )

    # Handle window close (safe shutdown)
    def closeEvent(self, event):
        self._clean_shutdown()
        event.accept()

    def _clean_shutdown(self):
        """Idempotent clean shutdown for ROS + thread."""
        if getattr(self, "_shutting_down", False):
            return
        self._shutting_down = True
        try:
            # stop the robot
            self.node.des_v = 0.0
            self.node.des_w = 0.0
            time.sleep(0.05)

            # request rclpy.spin() to return, then join thread
            if rclpy.ok():
                rclpy.shutdown()
            if hasattr(self, "ros_thread") and self.ros_thread.is_alive():
                self.ros_thread.join(timeout=2.0)

            # destroy node
            try:
                self.node.destroy_node()
            except Exception:
                pass
        except Exception:
            pass


# =================================== BOOTSTRAP =================================
if __name__ == "__main__":
    # Better scaling on HiDPI displays
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    app = QApplication([])
    win = Main()
    win.show()
    app.exec_()
