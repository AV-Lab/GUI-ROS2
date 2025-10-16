#!/usr/bin/env python3
# TurtleBot3 Teleop + Visualization (PyQt5) + SLAM map overlay + live plot
# Everything is drawn in the MAP frame.
# Robot pose and path come from TF: (map <- base_link) or (map <- base_footprint).
#
# Deps:
#   sudo apt install -y python3-pyqt5 python3-pyqtgraph ros-humble-tf2-ros
#
# Run:
#   source /opt/ros/humble/setup.bash
#   export TURTLEBOT3_MODEL=burger
#   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
#   ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
#   /usr/bin/python3 tb3_pyqt_gui_slam.py

import math
import os
import time
import threading
from collections import deque

from PyQt5.QtCore import Qt, QTimer, QPointF
from PyQt5.QtGui import QPainter, QPen, QColor, QPixmap, QTransform, QImage
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QSplitter, QGroupBox, QVBoxLayout,
    QHBoxLayout, QLabel, QPushButton, QSlider, QGridLayout, QCheckBox,
    QSizePolicy, QFileDialog, QMessageBox
)

# Plot
try:
    import pyqtgraph as pg
    HAVE_PG = True
except Exception:
    HAVE_PG = False

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException

ROBOT_IMAGE_PATH = "turtlebot.png"  # optional top-down sprite (front up)


def yaw_from_quaternion(x, y, z, w) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ======================= ROS Node (MAP frame) =======================
class TB3RosNode(Node):
    def __init__(self):
        super().__init__('tb3_pyqt_slam_node')

        # publishers/subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 50)           # for velocities
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)        # map image

        # publish commands at 50 Hz
        self.des_lin = 0.0
        self.des_ang = 0.0
        self.create_timer(0.02, self._publish_cmd)

        # velocities (from /odom)
        self.v_lin = 0.0
        self.v_ang = 0.0

        # TB3 dims (m)
        self.length = 0.18
        self.width  = 0.14

        # MAP pose (from TF map <- base_link or base_footprint)
        self.mx = 0.0
        self.my = 0.0
        self.myaw = 0.0
        self.tf_ok = False

        # PATH in MAP frame
        self.path_map = []
        self._last_path_xy = None
        self._min_spacing = 0.03
        self._max_pts = 3000

        # SLAM map storage
        self.map_width = 0
        self.map_height = 0
        self.map_res = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_origin_yaw = 0.0
        self.map_data = b""
        self.map_version = 0

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # update base pose in MAP via TF (20 Hz)
        self.create_timer(0.05, self._update_base_map)

        # lock for GUI reads
        self._lock = threading.Lock()

    # --- callbacks ---
    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self.v_lin = msg.twist.twist.linear.x
            self.v_ang = msg.twist.twist.angular.z

    def _map_cb(self, msg: OccupancyGrid):
        info = msg.info
        origin = info.origin
        yaw0 = yaw_from_quaternion(origin.orientation.x, origin.orientation.y,
                                   origin.orientation.z, origin.orientation.w)
        # int8 -> bytes 0..255 (with -1 -> 255)
        raw = bytes([(v + 256) % 256 for v in msg.data])
        with self._lock:
            self.map_width = info.width
            self.map_height = info.height
            self.map_res = info.resolution
            self.map_origin_x = origin.position.x
            self.map_origin_y = origin.position.y
            self.map_origin_yaw = yaw0
            self.map_data = raw
            self.map_version += 1

    def _publish_cmd(self):
        msg = Twist()
        msg.linear.x = float(self.des_lin)
        msg.angular.z = float(self.des_ang)
        self.cmd_pub.publish(msg)

    def _update_base_map(self):
        # Prefer base_link; fallback to base_footprint
        frames = ['base_link', 'base_footprint']
        got = False
        for child in frames:
            try:
                t = self.tf_buffer.lookup_transform('map', child, Time())
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                q = t.transform.rotation
                tyaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
                with self._lock:
                    self.mx = tx
                    self.my = ty
                    self.myaw = tyaw
                    self.tf_ok = True
                got = True
                break
            except TransformException:
                continue
        if not got:
            with self._lock:
                self.tf_ok = False

        # Append to MAP path if moved enough
        if got:
            with self._lock:
                if self._last_path_xy is None:
                    self.path_map.append((self.mx, self.my))
                    self._last_path_xy = (self.mx, self.my)
                else:
                    dx = self.mx - self._last_path_xy[0]
                    dy = self.my - self._last_path_xy[1]
                    if (dx*dx + dy*dy) >= (self._min_spacing**2):
                        self.path_map.append((self.mx, self.my))
                        if len(self.path_map) > self._max_pts:
                            self.path_map.pop(0)
                        self._last_path_xy = (self.mx, self.my)

    # --- getters for GUI ---
    def get_state_map(self):
        with self._lock:
            return (self.mx, self.my, self.myaw, self.v_lin, self.v_ang, list(self.path_map), self.tf_ok)

    def get_map_snapshot(self):
        with self._lock:
            return dict(
                version=self.map_version,
                width=self.map_width,
                height=self.map_height,
                res=self.map_res,
                origin_x=self.map_origin_x,
                origin_y=self.map_origin_y,
                origin_yaw=self.map_origin_yaw,
                data=self.map_data
            )


# ======================= Plot Widget =======================
class SpeedPlot(QWidget):
    """Live plot of |linear x| over time (last ~60s)."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.layout = QVBoxLayout(self); self.setLayout(self.layout)
        self.window_s = 60.0
        self.ts = deque(); self.vs = deque()
        self.paused = False

        if HAVE_PG:
            pg.setConfigOptions(antialias=True)
            self.plot = pg.PlotWidget()
            self.plot.setBackground('w')
            self.plot.showGrid(x=True, y=True, alpha=0.3)
            self.plot.setLabel('bottom', 'time', units='s')
            self.plot.setLabel('left', 'speed', units='m/s')
            self.curve = self.plot.plot([], [], pen=pg.mkPen(width=2))
            self.layout.addWidget(self.plot)
            self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        else:
            lbl = QLabel("pyqtgraph not installed.\nInstall: sudo apt install python3-pyqtgraph")
            lbl.setStyleSheet("color: #a33;")
            self.layout.addWidget(lbl)
            self.plot = None
            self.curve = None

    def add_sample(self, v_abs: float):
        if self.paused: return
        now = time.monotonic()
        self.ts.append(now)
        self.vs.append(v_abs)
        cutoff = now - self.window_s
        while self.ts and self.ts[0] < cutoff:
            self.ts.popleft(); self.vs.popleft()

    def refresh(self):
        if not HAVE_PG or not self.ts: return
        t0 = self.ts[0]
        x = [t - t0 for t in self.ts]
        y = list(self.vs)
        self.curve.setData(x, y)
        vmax = max(0.2, max(y) * 1.2)
        self.plot.setYRange(0.0, vmax, padding=0.02)
        if x:
            x_max = max(x[-1], self.window_s)
            self.plot.setXRange(max(0.0, x_max - self.window_s), x_max, padding=0.0)

    def toggle_pause(self):
        self.paused = not self.paused
        return self.paused

    def save_png(self, parent=None):
        if not HAVE_PG or self.plot is None:
            QMessageBox.warning(self, "Save Plot", "Plotting backend not available (pyqtgraph missing).")
            return
        dlg = QFileDialog(parent or self, "Save Plot")
        dlg.setAcceptMode(QFileDialog.AcceptSave)
        dlg.setNameFilters(["PNG Files (*.png)"])
        dlg.setDefaultSuffix("png")
        dlg.setOption(QFileDialog.DontUseNativeDialog, True)
        if dlg.exec_() == QFileDialog.Accepted:
            path = dlg.selectedFiles()[0]
            pix = self.plot.grab()
            if not pix.save(path):
                QMessageBox.critical(self, "Save Plot", "Failed to save plot image.")


# ======================= Canvas (draws in MAP frame) =======================
class Canvas(QWidget):
    def __init__(self, node: TB3RosNode, parent=None):
        super().__init__(parent)
        self.node = node
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self.scale = 80.0  # px per meter
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.follow = True
        self.show_grid = True
        self.show_map = True

        # sprite cache
        self.sprite = QPixmap()
        self.sprite_loaded = False
        if os.path.exists(ROBOT_IMAGE_PATH):
            sp = QPixmap(ROBOT_IMAGE_PATH)
            if not sp.isNull():
                self.sprite = sp
                self.sprite_loaded = True
        self._last_sprite_key = None  # (px_w, px_h, yaw_deg_q)
        self._sprite_cache = None

        # map cache
        self._map_version_seen = -1
        self._map_image = None
        self._map_rgba = None

        # panning
        self._dragging = False
        self._last_mouse = None

    # transforms (MAP world <-> view)
    def world_to_view(self, x, y):
        vx = x * self.scale + self.offset_x
        vy = -y * self.scale + self.offset_y
        return vx, vy

    def view_to_world(self, vx, vy):
        x = (vx - self.offset_x) / self.scale
        y = -(vy - self.offset_y) / self.scale
        return x, y

    def wheelEvent(self, e):
        angle = e.angleDelta().y()
        factor = 1.15 if angle > 0 else 1 / 1.15
        self.scale = max(10.0, min(600.0, self.scale * factor))
        # keep mouse world point fixed
        mx, my = e.x(), e.y()
        wx, wy = self.view_to_world(mx, my)
        self.offset_x = mx - wx * self.scale
        self.offset_y = my + wy * self.scale
        self.update()

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._dragging = True
            self._last_mouse = e.pos()
            self.follow = False
            self.update()

    def mouseMoveEvent(self, e):
        if self._dragging and self._last_mouse is not None:
            d = e.pos() - self._last_mouse
            self.offset_x += d.x()
            self.offset_y += d.y()
            self._last_mouse = e.pos()
            self.update()

    def mouseReleaseEvent(self, e):
        self._dragging = False
        self._last_mouse = None

    def center_on(self, x, y):
        w = self.width() or 800
        h = self.height() or 600
        cx = x * self.scale
        cy = -y * self.scale
        self.offset_x = w / 2 - cx
        self.offset_y = h / 2 - cy
        self.update()

    def save_view_png(self, parent=None):
        dlg = QFileDialog(parent or self, "Save View")
        dlg.setAcceptMode(QFileDialog.AcceptSave)
        dlg.setNameFilters(["PNG Files (*.png)", "JPEG Files (*.jpg *.jpeg)"])
        dlg.setDefaultSuffix("png")
        dlg.setOption(QFileDialog.DontUseNativeDialog, True)
        if dlg.exec_() == QFileDialog.Accepted:
            path = dlg.selectedFiles()[0]
            pix = self.grab()
            if not pix.save(path):
                QMessageBox.critical(self, "Save View", "Failed to save the view image.")

    # --- build QImage from OccupancyGrid snapshot ---
    def _ensure_map_image(self):
        snap = self.node.get_map_snapshot()
        if snap["version"] == 0 or snap["data"] == b"":
            return None, snap
        if snap["version"] == self._map_version_seen and self._map_image is not None:
            return self._map_image, snap

        w = snap["width"]; h = snap["height"]
        data = snap["data"]  # bytes in 0..255, with 255 meaning unknown
        rgba = bytearray(w * h * 4)
        for i in range(w * h):
            v = data[i]
            if v == 255:   # unknown (-1)
                rgba[4*i+0] = 255; rgba[4*i+1] = 255; rgba[4*i+2] = 255; rgba[4*i+3] = 0
            else:
                gray = 255 - int(255 * (v / 100.0))  # free(0)=255, occ(100)=0
                rgba[4*i+0] = gray; rgba[4*i+1] = gray; rgba[4*i+2] = gray; rgba[4*i+3] = 220

        self._map_rgba = bytes(rgba)
        img = QImage(self._map_rgba, w, h, QImage.Format_RGBA8888)
        self._map_image = img
        self._map_version_seen = snap["version"]
        return self._map_image, snap

    def paintEvent(self, ev):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        w = self.width()
        h = self.height()
        painter.fillRect(0, 0, w, h, QColor('white'))

        # Pose/path in MAP frame
        mx, my, myaw, v_lin, v_ang, path_map, tf_ok = self.node.get_state_map()

        # auto-follow
        if self.follow and tf_ok:
            self.center_on(mx, my)

        # draw MAP (OccupancyGrid) first
        if self.show_map:
            img, snap = self._ensure_map_image()
            if img is not None:
                vx0, vy0 = self.world_to_view(snap["origin_x"], snap["origin_y"])
                painter.save()
                painter.translate(vx0, vy0)
                painter.rotate(-math.degrees(snap["origin_yaw"]))  # screen y-down
                s = self.scale * snap["res"]
                painter.scale(s, -s)
                painter.setOpacity(1.0)
                painter.drawImage(0, 0, img)  # top-left at origin (corrects the vertical shift)

                painter.restore()

        # grid
        if self.show_grid:
            self._draw_grid(painter, w, h)

        # path (MAP)
        if len(path_map) >= 2:
            pen = QPen(QColor(0, 160, 0)); pen.setWidth(2)
            painter.setPen(pen)
            last_pt = None
            for px, py in path_map:
                vxp, vyp = self.world_to_view(px, py)
                pt = QPointF(vxp, vyp)
                if last_pt is not None:
                    painter.drawLine(last_pt, pt)
                last_pt = pt

        # robot (MAP)
        if tf_ok:
            self._draw_robot(painter, mx, my, myaw)

        # axes
        painter.setPen(QPen(QColor(220, 51, 51), 2))
        x1, y1 = self.world_to_view(-1e6, 0); x2, y2 = self.world_to_view(1e6, 0)
        painter.drawLine(QPointF(x1, y1), QPointF(x2, y2))
        painter.setPen(QPen(QColor(51, 102, 220), 2))
        x1, y1 = self.world_to_view(0, -1e6); x2, y2 = self.world_to_view(0, 1e6)
        painter.drawLine(QPointF(x1, y1), QPointF(x2, y2))

    def _draw_grid(self, painter: QPainter, w, h):
        x_min, y_max = self.view_to_world(0, 0)
        x_max, y_min = self.view_to_world(w, h)
        spacing = 1.0
        while spacing * self.scale < 30:
            spacing *= 2.0
        painter.setPen(QPen(QColor(230, 230, 230)))
        # verticals
        x0 = math.floor(x_min / spacing) * spacing
        xx = x0
        while xx <= x_max:
            vx1, vy1 = self.world_to_view(xx, y_min)
            vx2, vy2 = self.world_to_view(xx, y_max)
            painter.drawLine(QPointF(vx1, vy1), QPointF(vx2, vy2))
            xx += spacing
        # horizontals
        y0 = math.floor(y_min / spacing) * spacing
        yy = y0
        while yy <= y_max:
            vx1, vy1 = self.world_to_view(x_min, yy)
            vx2, vy2 = self.world_to_view(x_max, yy)
            painter.drawLine(QPointF(vx1, vy1), QPointF(vx2, vy2))
            yy += spacing

    def _draw_robot(self, painter: QPainter, x, y, yaw):
        L = self.node.length
        W = self.node.width
        vx, vy = self.world_to_view(x, y)

        if self.sprite_loaded:
            px_w = max(8, int(W * self.scale))
            px_h = max(8, int(L * self.scale))
            yaw_deg_q = int(round((-math.degrees(yaw)) / 2.0))  # quantize 2°
            key = (px_w, px_h, yaw_deg_q)
            if self._last_sprite_key != key:
                base = self.sprite.scaled(px_w, px_h, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                rot = base.transformed(QTransform().rotate(yaw_deg_q * 2), Qt.SmoothTransformation)
                self._sprite_cache = rot
                self._last_sprite_key = key

            if self._sprite_cache is not None:
                half_w = self._sprite_cache.width() / 2
                half_h = self._sprite_cache.height() / 2
                painter.drawPixmap(int(vx - half_w), int(vy - half_h), self._sprite_cache)

            fx = x + (L/2) * math.cos(yaw)
            fy = y + (L/2) * math.sin(yaw)
            cfx, cfy = self.world_to_view(fx, fy)
            painter.setPen(QPen(QColor(255, 165, 0), 2))
            painter.drawLine(QPointF(vx, vy), QPointF(cfx, cfy))
        else:
            pts = [(-W/2, -L/2), (W/2, -L/2), (W/2, L/2), (-W/2, L/2)]
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            qp = []
            for dx, dy in pts:
                gx = x + dx * cos_y - dy * sin_y
                gy = y + dx * sin_y + dy * cos_y
                qx, qy = self.world_to_view(gx, gy)
                qp.append(QPointF(qx, qy))
            painter.setPen(QPen(Qt.black, 1))
            painter.setBrush(QColor(135, 206, 235))
            painter.drawPolygon(*qp)

            fx = x + (L/2) * math.cos(yaw)
            fy = y + (L/2) * math.sin(yaw)
            cfx, cfy = self.world_to_view(fx, fy)
            painter.setPen(QPen(QColor(255, 165, 0), 2))
            painter.drawLine(QPointF(vx, vy), QPointF(cfx, cfy))


# ======================= Main Window =======================
class MainWindow(QMainWindow):
    def __init__(self, node: TB3RosNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("TurtleBot3 — Teleop + SLAM Map (PyQt5)")
        self.resize(1250, 800)

        splitter = QSplitter(Qt.Horizontal, self)
        self.setCentralWidget(splitter)

        # ---------- Left panel ----------
        left = QWidget(); left_layout = QVBoxLayout(left); left.setLayout(left_layout)
        splitter.addWidget(left)

        # Speeds
        speeds = QGroupBox("Speeds"); sp_l = QVBoxLayout(speeds); speeds.setLayout(sp_l)
        left_layout.addWidget(speeds)

        self.max_lin = 0.22
        self.max_ang = 1.80

        # Linear slider
        row_lin = QWidget(); row_lin_l = QHBoxLayout(row_lin); row_lin.setLayout(row_lin_l)
        row_lin_l.addWidget(QLabel("Max Linear (m/s)"))
        self.lbl_lin = QLabel(f"{self.max_lin:.2f} m/s"); row_lin_l.addWidget(self.lbl_lin, 1, Qt.AlignRight)
        sp_l.addWidget(row_lin)
        self.slider_lin = QSlider(Qt.Horizontal); self.slider_lin.setRange(0, 100); self.slider_lin.setValue(int(self.max_lin*100))
        self.slider_lin.valueChanged.connect(self._on_lin_changed)
        sp_l.addWidget(self.slider_lin)

        # Angular slider
        row_ang = QWidget(); row_ang_l = QHBoxLayout(row_ang); row_ang.setLayout(row_ang_l)
        row_ang_l.addWidget(QLabel("Max Angular (rad/s)"))
        self.lbl_ang = QLabel(f"{self.max_ang:.2f} rad/s"); row_ang_l.addWidget(self.lbl_ang, 1, Qt.AlignRight)
        sp_l.addWidget(row_ang)
        self.slider_ang = QSlider(Qt.Horizontal); self.slider_ang.setRange(0, 300); self.slider_ang.setValue(int(self.max_ang/3.0*300))
        self.slider_ang.valueChanged.connect(self._on_ang_changed)
        sp_l.addWidget(self.slider_ang)

        # Teleop
        tele = QGroupBox("Teleop"); tele_l = QVBoxLayout(tele); tele.setLayout(tele_l)
        left_layout.addWidget(tele)
        grid = QWidget(); grid_l = QGridLayout(grid); grid.setLayout(grid_l)
        btn_f = QPushButton("Forward"); btn_l = QPushButton("Left"); btn_s = QPushButton("Stop"); btn_r = QPushButton("Right"); btn_b = QPushButton("Back")
        btn_f.clicked.connect(self.cmd_forward); btn_l.clicked.connect(self.cmd_left); btn_s.clicked.connect(self.cmd_stop)
        btn_r.clicked.connect(self.cmd_right); btn_b.clicked.connect(self.cmd_back)
        grid_l.addWidget(btn_f, 0, 1)
        grid_l.addWidget(btn_l, 1, 0)
        grid_l.addWidget(btn_s, 1, 1)
        grid_l.addWidget(btn_r, 1, 2)
        grid_l.addWidget(btn_b, 2, 1)
        tele_l.addWidget(grid)

        # HUD
        hud = QGroupBox("Robot State (MAP TF)"); hud_l = QVBoxLayout(hud); hud.setLayout(hud_l)
        left_layout.addWidget(hud)
        self.lbl_pose_map = QLabel("Pose(map): x=0.00  y=0.00  yaw=0.0°  [TF: waiting]")
        self.lbl_vel  = QLabel("Vel:  lin=0.00 m/s  ang=0.00 rad/s")
        hud_l.addWidget(self.lbl_pose_map); hud_l.addWidget(self.lbl_vel)

        # Options
        opts = QGroupBox("View Options"); opts_l = QVBoxLayout(opts); opts.setLayout(opts_l)
        left_layout.addWidget(opts)
        self.chk_follow = QCheckBox("Follow Robot"); self.chk_follow.setChecked(True)
        self.chk_grid = QCheckBox("Show Grid"); self.chk_grid.setChecked(True)
        self.chk_map = QCheckBox("Show Map"); self.chk_map.setChecked(True)
        opts_l.addWidget(self.chk_follow); opts_l.addWidget(self.chk_grid); opts_l.addWidget(self.chk_map)

        # Speed plot + controls
        plot_box = QGroupBox("Speed (|linear x|, m/s)"); plot_l = QVBoxLayout(plot_box); plot_box.setLayout(plot_l)
        left_layout.addWidget(plot_box, 1)
        self.speed_plot = SpeedPlot()
        plot_l.addWidget(self.speed_plot)
        plot_btns = QWidget(); plot_btns_l = QHBoxLayout(plot_btns); plot_btns.setLayout(plot_btns_l)
        self.btn_pause_plot = QPushButton("Pause Plot"); self.btn_pause_plot.clicked.connect(self.on_toggle_pause_plot)
        self.btn_save_plot  = QPushButton("Save Plot");  self.btn_save_plot.clicked.connect(self.on_save_plot)
        plot_btns_l.addWidget(self.btn_pause_plot); plot_btns_l.addWidget(self.btn_save_plot)
        plot_l.addWidget(plot_btns)
        if not HAVE_PG:
            self.btn_pause_plot.setEnabled(False); self.btn_save_plot.setEnabled(False)

        left_layout.addStretch(0)

        # ---------- Right panel ----------
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel); right_panel.setLayout(right_layout)
        splitter.addWidget(right_panel)

        toolbar = QWidget(); tb_l = QHBoxLayout(toolbar); toolbar.setLayout(tb_l)
        self.btn_center  = QPushButton("Center on Robot"); self.btn_center.clicked.connect(self.center_on_robot)
        self.btn_zoom_in = QPushButton("Zoom In");         self.btn_zoom_in.clicked.connect(lambda: self.canvas_zoom(1.2))
        self.btn_zoom_out= QPushButton("Zoom Out");        self.btn_zoom_out.clicked.connect(lambda: self.canvas_zoom(1/1.2))
        self.btn_save_view = QPushButton("Save View");     self.btn_save_view.clicked.connect(self.on_save_view)
        tb_l.addWidget(self.btn_center); tb_l.addWidget(self.btn_zoom_in); tb_l.addWidget(self.btn_zoom_out)
        tb_l.addStretch(1); tb_l.addWidget(self.btn_save_view)
        right_layout.addWidget(toolbar)

        self.canvas = Canvas(self.node)
        right_layout.addWidget(self.canvas, 1)

        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

        self.setFocusPolicy(Qt.StrongFocus)

        # GUI timer
        self.gui_timer = QTimer(self)
        self.gui_timer.timeout.connect(self._on_timer)
        self.gui_timer.start(100)  # 10 Hz

    # sliders
    def _on_lin_changed(self, val):
        self.max_lin = val / 100.0
        self.lbl_lin.setText(f"{self.max_lin:.2f} m/s")

    def _on_ang_changed(self, val):
        self.max_ang = (val / 300.0) * 3.0
        self.lbl_ang.setText(f"{self.max_ang:.2f} rad/s")

    # teleop
    def cmd_forward(self): self.node.des_lin = self.max_lin; self.node.des_ang = 0.0
    def cmd_back(self):    self.node.des_lin = -self.max_lin; self.node.des_ang = 0.0
    def cmd_left(self):    self.node.des_lin = 0.0; self.node.des_ang = self.max_ang
    def cmd_right(self):   self.node.des_lin = 0.0; self.node.des_ang = -self.max_ang
    def cmd_stop(self):    self.node.des_lin = 0.0; self.node.des_ang = 0.0

    # keys
    def keyPressEvent(self, e):
        k = e.key()
        if   k in (Qt.Key_W, Qt.Key_Up):    self.cmd_forward()
        elif k in (Qt.Key_S, Qt.Key_Down):  self.cmd_back()
        elif k in (Qt.Key_A, Qt.Key_Left):  self.cmd_left()
        elif k in (Qt.Key_D, Qt.Key_Right): self.cmd_right()
        elif k == Qt.Key_Space:             self.cmd_stop()
        else:
            super().keyPressEvent(e)

    # timer
    def _on_timer(self):
        self.canvas.follow = self.chk_follow.isChecked()
        self.canvas.show_grid = self.chk_grid.isChecked()
        self.canvas.show_map = self.chk_map.isChecked()
        self.canvas.update()

        mx, my, myaw, v_lin, v_ang, _, tf_ok = self.node.get_state_map()
        if tf_ok:
            self.lbl_pose_map.setText(f"Pose(map): x={mx:.2f}  y={my:.2f}  yaw={math.degrees(myaw):.1f}°  [TF OK]")
        else:
            self.lbl_pose_map.setText("Pose(map): (TF unavailable)")
        self.lbl_vel.setText(f"Vel:  lin={v_lin:.2f} m/s  ang={v_ang:.2f} rad/s")

        self.speed_plot.add_sample(abs(v_lin))
        self.speed_plot.refresh()

    # canvas helpers
    def center_on_robot(self):
        mx, my, *_ = self.node.get_state_map()
        self.canvas.center_on(mx, my)

    def canvas_zoom(self, factor: float):
        w = self.canvas.width() or 800
        h = self.canvas.height() or 600
        mx, my = w/2, h/2
        wx, wy = self.canvas.view_to_world(mx, my)
        self.canvas.scale = max(10.0, min(600.0, self.canvas.scale * factor))
        self.canvas.offset_x = mx - wx * self.canvas.scale
        self.canvas.offset_y = my + wy * self.canvas.scale
        self.canvas.update()

    # plot buttons (non-native dialogs)
    def on_toggle_pause_plot(self):
        paused = self.speed_plot.toggle_pause()
        self.btn_pause_plot.setText("Resume Plot" if paused else "Pause Plot")

    def on_save_plot(self):
        QTimer.singleShot(0, lambda: self.speed_plot.save_png(self))

    def on_save_view(self):
        QTimer.singleShot(0, lambda: self.canvas.save_view_png(self))


# ======================= App bootstrap =======================
def spin_ros(node: TB3RosNode):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = TB3RosNode()

    t = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    t.start()

    app = QApplication([])
    win = MainWindow(node)

    def on_quit():
        node.des_lin = 0.0; node.des_ang = 0.0
        time.sleep(0.05)
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()

    app.aboutToQuit.connect(on_quit)
    win.show()
    app.exec_()

if __name__ == "__main__":
    main()
