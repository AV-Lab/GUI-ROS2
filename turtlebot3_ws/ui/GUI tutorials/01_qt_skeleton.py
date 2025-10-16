#!/usr/bin/env python3
"""
01 — PyQt Skeleton
Goal:
  - Create a PyQt window with a left toolbar and a right drawing canvas.
  - Implement world<->view transforms, mouse pan, wheel zoom, grid + axes.

How to run:
  /usr/bin/python3 01_qt_skeleton.py
"""

import math
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPainter, QPen, QColor
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QSplitter, QVBoxLayout,
    QHBoxLayout, QPushButton
)

# class Canvas(QWidget):
#     """Simple world space canvas with pan+zoom and a metric grid."""
#     def __init__(self):
#         super().__init__()
#         self.scale = 80.0     # pixels per meter
#         self.offset_x = 0.0   # view offset in pixels
#         self.offset_y = 0.0
#         self._dragging = False
#         self._last_mouse = None

#     # -------- coordinate transforms (world meters <-> view pixels) ----------
#     def world_to_view(self, x, y):
#         return x * self.scale + self.offset_x, -y * self.scale + self.offset_y

#     def view_to_world(self, vx, vy):
#         return (vx - self.offset_x) / self.scale, -(vy - self.offset_y) / self.scale

#     # ---------------- interactions: wheel zoom / mouse drag ------------------
#     def wheelEvent(self, e):
#         factor = 1.15 if e.angleDelta().y() > 0 else 1/1.15
#         # Update scale but keep the world point under the mouse fixed on screen
#         mx, my = e.x(), e.y()
#         wx, wy = self.view_to_world(mx, my)
#         self.scale = max(10.0, min(600.0, self.scale * factor))
#         self.offset_x = mx - wx * self.scale
#         self.offset_y = my + wy * self.scale
#         self.update()

#     def mousePressEvent(self, e):
#         if e.button() == Qt.LeftButton:
#             self._dragging = True
#             self._last_mouse = e.pos()

#     def mouseMoveEvent(self, e):
#         if self._dragging:
#             d = e.pos() - self._last_mouse
#             self.offset_x += d.x()
#             self.offset_y += d.y()
#             self._last_mouse = e.pos()
#             self.update()

#     def mouseReleaseEvent(self, _):  # end drag
#         self._dragging = False
#         self._last_mouse = None

#     def center_on(self, x, y):
#         """Center the view on a world point (x,y) in meters."""
#         w = self.width() or 800
#         h = self.height() or 600
#         cx = x * self.scale
#         cy = -y * self.scale
#         self.offset_x = w/2 - cx
#         self.offset_y = h/2 - cy
#         self.update()

#     # -------------------------------- painting --------------------------------
#     def paintEvent(self, _):
#         p = QPainter(self)
#         p.setRenderHint(QPainter.Antialiasing, True)
#         p.fillRect(self.rect(), QColor('white'))

#         # Visible world-rect in meters (for grid culling)
#         w = self.width() or 800
#         h = self.height() or 600
#         x_min, y_max = self.view_to_world(0, 0)
#         x_max, y_min = self.view_to_world(w, h)

#         # Grid spacing ~>=30px on screen (in meters)
#         spacing = 1.0
#         while spacing * self.scale < 30:
#             spacing *= 2.0

#         # Draw grid
#         p.setPen(QPen(QColor(230, 230, 230)))
#         xx = math.floor(x_min / spacing) * spacing
#         while xx <= x_max:
#             x1, y1 = self.world_to_view(xx, y_min)
#             x2, y2 = self.world_to_view(xx, y_max)
#             p.drawLine(QPointF(x1, y1), QPointF(x2, y2))
#             xx += spacing

#         yy = math.floor(y_min / spacing) * spacing
#         while yy <= y_max:
#             x1, y1 = self.world_to_view(x_min, yy)
#             x2, y2 = self.world_to_view(x_max, yy)
#             p.drawLine(QPointF(x1, y1), QPointF(x2, y2))
#             yy += spacing

#         # Axes
#         p.setPen(QPen(QColor(220, 51, 51), 2))  # X (red)
#         x1, y1 = self.world_to_view(-1e6, 0)
#         x2, y2 = self.world_to_view(1e6, 0)
#         p.drawLine(QPointF(x1, y1), QPointF(x2, y2))
#         p.setPen(QPen(QColor(51, 102, 220), 2))  # Y (blue)
#         x1, y1 = self.world_to_view(0, -1e6)
#         x2, y2 = self.world_to_view(0, 1e6)
#         p.drawLine(QPointF(x1, y1), QPointF(x2, y2))

class Canvas(QWidget):
    def __init__(self):
        super().__init__()
        # World↔View: meters ↔ pixels
        self.scale = 80.0        # px per meter
        self.offset_x = 0.0      # view offset in px
        self.offset_y = 0.0

        # Interactions
        self._dragging = False
        self._last_mouse = None

        # Feature toggles (used in later steps; harmless now)
        self.show_grid = True

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

        # (3) Hooks for later steps — no-ops now.
        #     In later tutorials you’ll paste bodies into these methods:
        #     self._draw_map(p), self._draw_path(p), self._draw_robot(p)
        #     and call them here in this order.
        # Example later:
        # self._draw_map(p)
        # self._draw_path(p)
        # self._draw_robot(p)

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

    # ---- placeholders you’ll FILL in later tutorials ----
    def _draw_map(self, p: QPainter):    pass
    def _draw_path(self, p: QPainter):   pass
    def _draw_robot(self, p: QPainter):  pass

class Main(QMainWindow):
    """Window with a left toolbar and the canvas on the right."""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("01 — PyQt Skeleton")
        self.resize(1000, 700)

        splitter = QSplitter(Qt.Horizontal, self)
        self.setCentralWidget(splitter)

        # Left: tiny toolbar
        left = QWidget(); l = QVBoxLayout(left)
        btn_center = QPushButton("Center (0,0)")
        btn_in     = QPushButton("Zoom In")
        btn_out    = QPushButton("Zoom Out")
        row = QHBoxLayout(); row.addWidget(btn_center); row.addWidget(btn_in); row.addWidget(btn_out)
        l.addLayout(row); l.addStretch(1)
        splitter.addWidget(left)

        # Right: canvas
        self.canvas = Canvas()
        splitter.addWidget(self.canvas)
        splitter.setStretchFactor(1, 1)

        # Wire toolbar
        btn_center.clicked.connect(lambda: self.canvas.center_on(0, 0))
        btn_in.clicked.connect(lambda: self._zoom(1.2))
        btn_out.clicked.connect(lambda: self._zoom(1/1.2))

    def _zoom(self, factor):
        """Zoom around the canvas center."""
        w = self.canvas.width() or 800
        h = self.canvas.height() or 600
        mx, my = w/2, h/2
        wx, wy = self.canvas.view_to_world(mx, my)
        self.canvas.scale = max(10.0, min(600.0, self.canvas.scale * factor))
        self.canvas.offset_x = mx - wx * self.canvas.scale
        self.canvas.offset_y = my + wy * self.canvas.scale
        self.canvas.update()

if __name__ == "__main__":
    app = QApplication([])
    win = Main()
    win.show()
    app.exec_()
