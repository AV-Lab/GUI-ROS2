#!/usr/bin/env python3
"""
05 — Pause + Save Plot + Save Canvas (PyQt5)

Builds on Tutorial 04:
- Adds "Pause Plot" toggle
- Adds "Save Plot" (PNG) and "Save View" (PNG/JPEG) with non-native dialogs
  (prevents Wayland portal freezes)
"""

import math, threading, time
from collections import deque
from PyQt5.QtCore import Qt, QPointF, QTimer
from PyQt5.QtGui import QPainter, QPen, QColor
from PyQt5.QtWidgets import *
try:
    import pyqtgraph as pg
    HAVE_PG = True
except Exception:
    HAVE_PG = False

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# ---------------- Canvas (Step-01 + Step-03) with Save View -------------------
class Canvas(QWidget):
    def __init__(self):
        super().__init__()
        self.scale=80.0; self.offset_x=0.0; self.offset_y=0.0
        self._dragging=False; self._last_mouse=None
        self.show_grid=True; self.follow=True
        self._state_provider=None
    def set_state_provider(self,fn): self._state_provider=fn
    def world_to_view(self,x,y): return x*self.scale+self.offset_x, -y*self.scale+self.offset_y
    def view_to_world(self,vx,vy): return (vx-self.offset_x)/self.scale, -(vy-self.offset_y)/self.scale
    def wheelEvent(self,e):
        f=1.15 if e.angleDelta().y()>0 else 1/1.15
        mx,my=e.x(),e.y(); wx,wy=self.view_to_world(mx,my)
        self.scale=max(10.0,min(600.0,self.scale*f))
        self.offset_x=mx - wx*self.scale; self.offset_y=my + wy*self.scale; self.update()
    def mousePressEvent(self,e):
        if e.button()==Qt.LeftButton: self._dragging=True; self._last_mouse=e.pos()
    def mouseMoveEvent(self,e):
        if self._dragging: d=e.pos()-self._last_mouse; self.offset_x+=d.x(); self.offset_y+=d.y(); self._last_mouse=e.pos(); self.update()
    def mouseReleaseEvent(self,_): self._dragging=False; self._last_mouse=None
    def center_on(self,x,y):
        w=self.width() or 800; h=self.height() or 600
        cx=x*self.scale; cy=-y*self.scale; self.offset_x=w/2-cx; self.offset_y=h/2-cy; self.update()
    def save_view_png(self,parent=None):
        dlg=QFileDialog(parent or self,"Save View"); dlg.setAcceptMode(QFileDialog.AcceptSave)
        dlg.setNameFilters(["PNG Files (*.png)","JPEG Files (*.jpg *.jpeg)"])
        dlg.setDefaultSuffix("png"); dlg.setOption(QFileDialog.DontUseNativeDialog, True)
        if dlg.exec_()==QFileDialog.Accepted:
            path=dlg.selectedFiles()[0]; pix=self.grab()
            if not pix.save(path): QMessageBox.critical(self,"Save View","Failed to save image.")
    def paintEvent(self,_):
        p=QPainter(self); p.setRenderHint(QPainter.Antialiasing,True)
        p.fillRect(self.rect(), QColor('white'))
        if self.show_grid: self._grid(p)
        self._axes(p)
        if self._state_provider:
            x,y,yaw,path=self._state_provider()
            if self.follow: self.center_on(x,y)
            self._path(p,path); self._robot(p,x,y,yaw)
    def _grid(self,p):
        w=self.width() or 800; h=self.height() or 600
        x_min,y_max=self.view_to_world(0,0); x_max,y_min=self.view_to_world(w,h)
        spacing=1.0
        while spacing*self.scale<30: spacing*=2.0
        p.setPen(QPen(QColor(230,230,230)))
        xx=math.floor(x_min/spacing)*spacing
        while xx<=x_max:
            x1,y1=self.world_to_view(xx,y_min); x2,y2=self.world_to_view(xx,y_max); p.drawLine(QPointF(x1,y1),QPointF(x2,y2)); xx+=spacing
        yy=math.floor(y_min/spacing)*spacing
        while yy<=y_max:
            x1,y1=self.world_to_view(x_min,yy); x2,y2=self.world_to_view(x_max,yy); p.drawLine(QPointF(x1,y1),QPointF(x2,y2)); yy+=spacing
    def _axes(self,p):
        p.setPen(QPen(QColor(220,51,51),2)); x1,y1=self.world_to_view(-1e6,0); x2,y2=self.world_to_view(1e6,0); p.drawLine(QPointF(x1,y1),QPointF(x2,y2))
        p.setPen(QPen(QColor(51,102,220),2)); x1,y1=self.world_to_view(0,-1e6); x2,y2=self.world_to_view(0,1e6); p.drawLine(QPointF(x1,y1),QPointF(x2,y2))
    def _path(self,p,path):
        if not path or len(path)<2: return
        pen=QPen(QColor(0,160,0)); pen.setWidth(2); p.setPen(pen)
        last=None
        for px,py in path:
            vx,vy=self.world_to_view(px,py); pt=QPointF(vx,vy)
            if last is not None: p.drawLine(last,pt)
            last=pt
    def _robot(self,p,x,y,yaw,L=0.18,W=0.14):
        pts=[(-W/2,-L/2),(W/2,-L/2),(W/2,L/2),(-W/2,L/2)]
        c,s=math.cos(yaw),math.sin(yaw); poly=[]
        for dx,dy in pts:
            gx=x+dx*c-dy*s; gy=y+dx*s+dy*c; vx,vy=self.world_to_view(gx,gy); poly.append(QPointF(vx,vy))
        p.setPen(QPen(Qt.black,1)); p.setBrush(QColor(135,206,235)); p.drawPolygon(*poly)
        fx=x+(L/2)*math.cos(yaw); fy=y+(L/2)*math.sin(yaw); cfx,cfy=self.world_to_view(fx,fy); cx,cy=self.world_to_view(x,y)
        p.setPen(QPen(QColor(255,165,0),2)); p.drawLine(QPointF(cx,cy), QPointF(cfx,cfy))

# ---------------- ROS Node ----------------
def yaw_from_quat(x,y,z,w): return math.atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z))
class OdomNode(Node):
    def __init__(self):
        super().__init__('tb3_save_gui')
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.create_subscription(Odometry,'/odom',self.cb,50)
        self.create_timer(0.02,self.tick)
        self.des_v=0.0; self.des_w=0.0
        self.x=0.0; self.y=0.0; self.yaw=0.0; self.v_lin=0.0; self.v_ang=0.0
        self.path=[]; self._last=None; self._min=0.03; self._max=3000
    def tick(self):
        m=Twist(); m.linear.x=float(self.des_v); m.angular.z=float(self.des_w); self.pub.publish(m)
    def cb(self,msg:Odometry):
        self.x=msg.pose.pose.position.x; self.y=msg.pose.pose.position.y
        q=msg.pose.pose.orientation; self.yaw=yaw_from_quat(q.x,q.y,q.z,q.w)
        self.v_lin=msg.twist.twist.linear.x; self.v_ang=msg.twist.twist.angular.z
        if self._last is None or (self.x-self._last[0])**2+(self.y-self._last[1])**2>=self._min**2:
            self.path.append((self.x,self.y)); self._last=(self.x,self.y)
            if len(self.path)>self._max: self.path.pop(0)
def spin_ros(n:Node): rclpy.spin(n)

# --------------- Plot with Pause/Save ---------------
class SpeedPlot(QWidget):
    def __init__(self, node: OdomNode):
        super().__init__()
        self.n=node; self.ts=deque(); self.vs=deque(); self.window_s=60.0; self.paused=False
        L=QVBoxLayout(self)
        if HAVE_PG:
            pg.setConfigOptions(antialias=True)
            self.plot=pg.PlotWidget(background='w'); self.plot.showGrid(x=True,y=True,alpha=0.3)
            self.plot.setLabel('bottom','time',units='s'); self.plot.setLabel('left','speed',units='m/s')
            self.curve=self.plot.plot([],[],pen=pg.mkPen(width=2)); L.addWidget(self.plot)
        else:
            L.addWidget(QLabel("pyqtgraph not installed.")); self.plot=None
    def step(self):
        if not HAVE_PG or self.paused: return
        now=time.monotonic(); self.ts.append(now); self.vs.append(abs(self.n.v_lin))
        cutoff=now-self.window_s
        while self.ts and self.ts[0]<cutoff: self.ts.popleft(); self.vs.popleft()
        if not self.ts: return
        t0=self.ts[0]; x=[t-t0 for t in self.ts]; y=list(self.vs)
        self.curve.setData(x,y); self.plot.setYRange(0.0, max(0.2, max(y)*1.2))
        x_max=max(x[-1], self.window_s); self.plot.setXRange(max(0.0, x_max-self.window_s), x_max)
    def toggle(self): self.paused=not self.paused
    def save_png(self, parent=None):
        if not HAVE_PG: return
        dlg=QFileDialog(parent or self,"Save Plot"); dlg.setAcceptMode(QFileDialog.AcceptSave)
        dlg.setNameFilters(["PNG Files (*.png)"]); dlg.setDefaultSuffix("png")
        dlg.setOption(QFileDialog.DontUseNativeDialog, True)
        if dlg.exec_()==QFileDialog.Accepted:
            path=dlg.selectedFiles()[0]; pix=self.plot.grab()
            if not pix.save(path): QMessageBox.critical(self,"Save Plot","Failed to save image.")

# ------------------------- Main -------------------------
class Main(QMainWindow):
    def __init__(self):
        super().__init__(); self.setWindowTitle("05 — Pause + Save Plot + Save View"); self.resize(1250,780)
        rclpy.init(); self.node=OdomNode(); self.ros_thread=threading.Thread(target=spin_ros, args=(self.node,), daemon=False); self.ros_thread.start()
        splitter=QSplitter(Qt.Horizontal,self); self.setCentralWidget(splitter)
        left=QWidget(); L=QVBoxLayout(left)
        tele=QGroupBox("Tele-op"); grid=QGridLayout(tele)
        mk=lambda v,w:(lambda:(setattr(self.node,'des_v',v), setattr(self.node,'des_w',w)))
        bF=QPushButton("Forward"); bB=QPushButton("Back"); bL=QPushButton("Left"); bR=QPushButton("Right"); bS=QPushButton("Stop")
        bF.clicked.connect(mk(0.22,0)); bB.clicked.connect(mk(-0.22,0)); bL.clicked.connect(mk(0,1.8)); bR.clicked.connect(mk(0,-1.8)); bS.clicked.connect(mk(0,0))
        grid.addWidget(bF,0,1); grid.addWidget(bL,1,0); grid.addWidget(bS,1,1); grid.addWidget(bR,1,2); grid.addWidget(bB,2,1)
        L.addWidget(tele)
        hud=QGroupBox("State (/odom)"); hL=QVBoxLayout(hud)
        self.lbl_pose=QLabel("Pose: x=0.00  y=0.00  yaw=0.0°"); self.lbl_vel=QLabel("Vel : lin=0.00 m/s  ang=0.00 rad/s")
        hL.addWidget(self.lbl_pose); hL.addWidget(self.lbl_vel); L.addWidget(hud)
        plot_box=QGroupBox("Speed (|linear x|, m/s)"); pL=QVBoxLayout(plot_box)
        self.plot=SpeedPlot(self.node); pL.addWidget(self.plot)
        row=QHBoxLayout(); self.btn_pause=QPushButton("Pause Plot"); self.btn_save=QPushButton("Save Plot")
        self.btn_pause.clicked.connect(lambda:(self.plot.toggle(), self.btn_pause.setText("Resume Plot" if self.plot.paused else "Pause Plot")))
        self.btn_save.clicked.connect(lambda: QTimer.singleShot(0, lambda: self.plot.save_png(self)))
        row.addWidget(self.btn_pause); row.addWidget(self.btn_save); pL.addLayout(row)
        L.addWidget(plot_box,1)
        save_view_btn=QPushButton("Save View (Canvas)"); save_view_btn.clicked.connect(lambda: QTimer.singleShot(0, lambda: self.canvas.save_view_png(self)))
        L.addWidget(save_view_btn); L.addStretch(0)
        splitter.addWidget(left)

        self.canvas=Canvas(); self.canvas.set_state_provider(lambda:(self.node.x,self.node.y,self.node.yaw,list(self.node.path)))
        splitter.addWidget(self.canvas); splitter.setStretchFactor(1,1)

        self.timer=QTimer(self); self.timer.timeout.connect(self._tick); self.timer.start(100)
        QApplication.instance().aboutToQuit.connect(self._clean_shutdown); self.setFocusPolicy(Qt.StrongFocus)

    def keyPressEvent(self,e):
        k=e.key()
        if   k in (Qt.Key_W,Qt.Key_Up):    self.node.des_v=0.22; self.node.des_w=0.0
        elif k in (Qt.Key_S,Qt.Key_Down):  self.node.des_v=-0.22; self.node.des_w=0.0
        elif k in (Qt.Key_A,Qt.Key_Left):  self.node.des_v=0.0;  self.node.des_w=1.8
        elif k in (Qt.Key_D,Qt.Key_Right): self.node.des_v=0.0;  self.node.des_w=-1.8
        elif k==Qt.Key_Space:              self.node.des_v=0.0;  self.node.des_w=0.0
        else: super().keyPressEvent(e)

    def _tick(self):
        self.canvas.update()
        self.lbl_pose.setText(f"Pose: x={self.node.x:.2f}  y={self.node.y:.2f}  yaw={math.degrees(self.node.yaw):.1f}°")
        self.lbl_vel.setText(f"Vel : lin={self.node.v_lin:.2f} m/s  ang={self.node.v_ang:.2f} rad/s")
        self.plot.step()

    def closeEvent(self,e): self._clean_shutdown(); e.accept()
    def _clean_shutdown(self):
        if getattr(self,'_sd',False): return
        self._sd=True
        try:
            self.node.des_v=0.0; self.node.des_w=0.0; time.sleep(0.05)
            if rclpy.ok(): rclpy.shutdown()
            if hasattr(self,'ros_thread') and self.ros_thread.is_alive(): self.ros_thread.join(timeout=2.0)
            try: self.node.destroy_node()
            except Exception: pass
        except Exception: pass

if __name__=="__main__":
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    app=QApplication([]); win=Main(); win.show(); app.exec_()
