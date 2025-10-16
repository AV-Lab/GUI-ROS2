#!/usr/bin/env python3
"""
07 — FINAL: Tele-op + SLAM map (MAP frame) + TF-aligned robot/path + plot + save

- Canvas world is MAP frame (meters)
- Draw OccupancyGrid in MAP using info.origin + resolution
- Query TF: map <- base_link (fallback: base_footprint)
- Accumulate path directly in MAP → perfect alignment
- Live speed plot (pause/save) + Save View
"""

import math, os, time, threading
from collections import deque
from PyQt5.QtCore import Qt, QPointF, QTimer
from PyQt5.QtGui import QPainter, QPen, QColor, QPixmap, QTransform, QImage
from PyQt5.QtWidgets import *
try:
    import pyqtgraph as pg
    HAVE_PG=True
except Exception:
    HAVE_PG=False

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException

ROBOT_IMAGE_PATH="turtlebot.png"

def yaw_from_q(x,y,z,w): return math.atan2(2*(w*z + x*y), 1-2*(y*y+z*z))

# ------------------------------ ROS Node --------------------------------------
class TB3Node(Node):
    def __init__(self):
        super().__init__('tb3_pyqt_slam_gui')
        # teleop
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.des_v=0.0; self.des_w=0.0
        self.create_timer(0.02, self._pub_cmd)
        # odom (for velocities only)
        self.v_lin=0.0; self.v_ang=0.0
        self.create_subscription(Odometry,'/odom', self._odom_cb, 50)
        # map
        self.map_w=0; self.map_h=0; self.map_res=0.0
        self.map_ox=0.0; self.map_oy=0.0; self.map_oyaw=0.0
        self.map_data=b""; self.map_ver=0
        self.create_subscription(OccupancyGrid,'/map', self._map_cb, 10)
        # tf
        self.tf_buf=Buffer(); self.tf_listener=TransformListener(self.tf_buf, self)
        self.mx=0.0; self.my=0.0; self.myaw=0.0; self.tf_ok=False
        # path in MAP
        self.path=[]; self._last=None; self._min_spacing=0.03; self._max_pts=3000
        # robot dims
        self.L=0.18; self.W=0.14
        self.create_timer(0.05, self._update_tf)  # 20 Hz

    def _pub_cmd(self):
        m=Twist(); m.linear.x=float(self.des_v); m.angular.z=float(self.des_w); self.pub.publish(m)
    def _odom_cb(self,msg:Odometry):
        self.v_lin=msg.twist.twist.linear.x; self.v_ang=msg.twist.twist.angular.z
    def _map_cb(self,msg:OccupancyGrid):
        info=msg.info; origin=info.origin
        self.map_w=info.width; self.map_h=info.height; self.map_res=info.resolution
        self.map_ox=origin.position.x; self.map_oy=origin.position.y
        self.map_oyaw=yaw_from_q(origin.orientation.x,origin.orientation.y,origin.orientation.z,origin.orientation.w)
        self.map_data=bytes([(v+256)%256 for v in msg.data]); self.map_ver+=1
    def _update_tf(self):
        for child in ('base_link','base_footprint'):
            try:
                t=self.tf_buf.lookup_transform('map', child, Time())
                tx=t.transform.translation.x; ty=t.transform.translation.y
                q=t.transform.rotation; yaw=yaw_from_q(q.x,q.y,q.z,q.w)
                self.mx,self.my,self.myaw=tx,ty,yaw; self.tf_ok=True
                if self._last is None or (self.mx-self._last[0])**2+(self.my-self._last[1])**2>=self._min_spacing**2:
                    self.path.append((self.mx,self.my)); self._last=(self.mx,self.my)
                    if len(self.path)>self._max_pts: self.path.pop(0)
                return
            except TransformException:
                continue
        self.tf_ok=False

    # snapshots for GUI
    def get_map_snap(self):
        return dict(ver=self.map_ver,w=self.map_w,h=self.map_h,res=self.map_res,
                    ox=self.map_ox,oy=self.map_oy,oyaw=self.map_oyaw,data=self.map_data)
    def get_state_map(self):
        return (self.mx,self.my,self.myaw, self.v_lin,self.v_ang, list(self.path), self.tf_ok)

# ------------------------------- Canvas (MAP) ---------------------------------
class Canvas(QWidget):
    def __init__(self,node:TB3Node):
        super().__init__()
        self.n=node
        self.scale=80.0; self.offset_x=0.0; self.offset_y=0.0
        self.follow=True; self.show_grid=True; self.show_map=True
        # sprite cache
        self.sprite_loaded=False; self.sprite=QPixmap()
        if os.path.exists(ROBOT_IMAGE_PATH):
            sp=QPixmap(ROBOT_IMAGE_PATH)
            if not sp.isNull(): self.sprite=sp; self.sprite_loaded=True
        self._sprite_key=None; self._sprite_cache=None
        # map cache
        self._map_ver_seen=-1; self._map_img=None; self._map_rgba=None

    # transforms
    def world_to_view(self,x,y): return x*self.scale+self.offset_x, -y*self.scale+self.offset_y
    def view_to_world(self,vx,vy): return (vx-self.offset_x)/self.scale, -(vy-self.offset_y)/self.scale
    # pan/zoom
    def wheelEvent(self,e):
        f=1.15 if e.angleDelta().y()>0 else 1/1.15
        mx,my=e.x(),e.y(); wx,wy=self.view_to_world(mx,my)
        self.scale=max(10.0,min(600.0,self.scale*f))
        self.offset_x=mx - wx*self.scale; self.offset_y=my + wy*self.scale; self.update()
    def mousePressEvent(self,e):
        if e.button()==Qt.LeftButton: self._drag=True; self._last=e.pos(); self.follow=False
    def mouseMoveEvent(self,e):
        if getattr(self,'_drag',False):
            d=e.pos()-self._last; self.offset_x+=d.x(); self.offset_y+=d.y(); self._last=e.pos(); self.update()
    def mouseReleaseEvent(self,e): self._drag=False
    def center_on(self,x,y):
        w=self.width() or 800; h=self.height() or 600
        cx=x*self.scale; cy=-y*self.scale; self.offset_x=w/2-cx; self.offset_y=h/2-cy; self.update()

    def save_view_png(self,parent=None):
        dlg=QFileDialog(parent or self,"Save View"); dlg.setAcceptMode(QFileDialog.AcceptSave)
        dlg.setNameFilters(["PNG Files (*.png)","JPEG Files (*.jpg *.jpeg)"]); dlg.setDefaultSuffix("png")
        dlg.setOption(QFileDialog.DontUseNativeDialog, True)
        if dlg.exec_()==QFileDialog.Accepted:
            path=dlg.selectedFiles()[0]; pix=self.grab()
            if not pix.save(path): QMessageBox.critical(self,"Save View","Failed to save image.")

    # map builder
    def _ensure_map_image(self,s):
        if s['ver']==0 or not s['data']: return None
        if s['ver']==self._map_ver_seen and self._map_img is not None: return self._map_img
        w,h=s['w'], s['h']; data=s['data']
        rgba=bytearray(w*h*4)
        for i in range(w*h):
            v=data[i]
            if v==255: rgba[4*i:4*i+4]=bytes((255,255,255,0))
            else:
                g=255-int(255*(v/100.0))
                rgba[4*i:4*i+4]=bytes((g,g,g,220))
        self._map_rgba=bytes(rgba); self._map_img=QImage(self._map_rgba, w,h, QImage.Format_RGBA8888)
        self._map_ver_seen=s['ver']; return self._map_img

    def paintEvent(self,_):
        p=QPainter(self); p.setRenderHint(QPainter.Antialiasing,True)
        p.fillRect(self.rect(), QColor('white'))

        mx,my,myaw, v_lin,v_ang, path, tf_ok = self.n.get_state_map()
        if self.follow and tf_ok: self.center_on(mx,my)

        # map
        if self.show_map:
            s=self.n.get_map_snap(); img=self._ensure_map_image(s)
            if img is not None:
                vx0,vy0=self.world_to_view(s['ox'], s['oy'])
                p.save(); p.translate(vx0,vy0); p.rotate(-math.degrees(s['oyaw']))
                sc=self.scale * s['res']; p.scale(sc, -sc); p.setOpacity(1.0)
                p.drawImage(0,0,img)   # top-left anchor
                p.restore()

        # grid
        if self.show_grid:
            w=self.width() or 800; h=self.height() or 600
            x_min,y_max=self.view_to_world(0,0); x_max,y_min=self.view_to_world(w,h)
            spacing=1.0
            while spacing*self.scale<30: spacing*=2.0
            p.setPen(QPen(QColor(230,230,230)))
            xx=math.floor(x_min/spacing)*spacing
            while xx<=x_max:
                x1,y1=self.world_to_view(xx,y_min); x2,y2=self.world_to_view(xx,y_max)
                p.drawLine(QPointF(x1,y1),QPointF(x2,y2)); xx+=spacing
            yy=math.floor(y_min/spacing)*spacing
            while yy<=y_max:
                x1,y1=self.world_to_view(x_min,yy); x2,y2=self.world_to_view(x_max,yy)
                p.drawLine(QPointF(x1,y1),QPointF(x2,y2)); yy+=spacing

        # path
        if len(path) >= 2:
            p.setPen(QPen(QColor(0,160,0), 2))
            last = None
            for px, py in path:
                vx, vy = self.world_to_view(px, py)
                pt = QPointF(vx, vy)
                if last is not None:
                    p.drawLine(last, pt)
                last = pt  # <-- update every loop


        # robot
        if tf_ok:
            cx,cy=self.world_to_view(mx,my)
            if os.path.exists(ROBOT_IMAGE_PATH):
                if getattr(self,'sprite_loaded',False)==False:
                    sp=QPixmap(ROBOT_IMAGE_PATH); self.sprite=sp; self.sprite_loaded=not sp.isNull()
                if self.sprite_loaded:
                    px_w=max(8,int(self.n.W*self.scale)); px_h=max(8,int(self.n.L*self.scale))
                    yaw_q=int(round((-math.degrees(myaw))/2.0)); key=(px_w,px_h,yaw_q)
                    if getattr(self,'_sprite_key',None)!=key:
                        base=self.sprite.scaled(px_w,px_h,Qt.KeepAspectRatio,Qt.SmoothTransformation)
                        rot=base.transformed(QTransform().rotate(yaw_q*2), Qt.SmoothTransformation)
                        self._sprite_cache=rot; self._sprite_key=key
                    if self._sprite_cache:
                        hw=self._sprite_cache.width()/2; hh=self._sprite_cache.height()/2
                        p.drawPixmap(int(cx-hw), int(cy-hh), self._sprite_cache)
            else:
                # rectangle fallback
                L=self.n.L; W=self.n.W; pts=[(-W/2,-L/2),(W/2,-L/2),(W/2,L/2),(-W/2,L/2)]
                c,s=math.cos(myaw),math.sin(myaw); poly=[]
                for dx,dy in pts:
                    gx=mx+dx*c-dy*s; gy=my+dx*s+dy*c; vx,vy=self.world_to_view(gx,gy); poly.append(QPointF(vx,vy))
                p.setPen(QPen(Qt.black,1)); p.setBrush(QColor(135,206,235)); p.drawPolygon(*poly)
            # heading
            fx=mx+(self.n.L/2)*math.cos(myaw); fy=my+(self.n.L/2)*math.sin(myaw)
            cfx,cfy=self.world_to_view(fx,fy); p.setPen(QPen(QColor(255,165,0),2)); p.drawLine(QPointF(cx,cy), QPointF(cfx,cfy))

        # axes
        p.setPen(QPen(QColor(220,51,51),2)); x1,y1=self.world_to_view(-1e6,0); x2,y2=self.world_to_view(1e6,0); p.drawLine(QPointF(x1,y1),QPointF(x2,y2))
        p.setPen(QPen(QColor(51,102,220),2)); x1,y1=self.world_to_view(0,-1e6); x2,y2=self.world_to_view(0,1e6); p.drawLine(QPointF(x1,y1),QPointF(x2,y2))

# ----------------------------- Plot widget ------------------------------------
class SpeedPlot(QWidget):
    def __init__(self, node:TB3Node):
        super().__init__()
        self.n=node; self.ts=deque(); self.vs=deque(); self.window_s=60.0; self.paused=False
        L=QVBoxLayout(self)
        if HAVE_PG:
            pg.setConfigOptions(antialias=True)
            self.plot=pg.PlotWidget(background='w'); self.plot.showGrid(x=True,y=True,alpha=0.3)
            self.plot.setLabel('bottom','time',units='s'); self.plot.setLabel('left','speed',units='m/s')
            self.curve=self.plot.plot([],[],pen=pg.mkPen(width=2)); L.addWidget(self.plot)
        else:
            L.addWidget(QLabel("Install pyqtgraph: sudo apt install python3-pyqtgraph")); self.plot=None
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
    def save_png(self,parent=None):
        if not HAVE_PG: return
        dlg=QFileDialog(parent or self,"Save Plot"); dlg.setAcceptMode(QFileDialog.AcceptSave)
        dlg.setNameFilters(["PNG Files (*.png)"]); dlg.setDefaultSuffix("png")
        dlg.setOption(QFileDialog.DontUseNativeDialog, True)
        if dlg.exec_()==QFileDialog.Accepted:
            path=dlg.selectedFiles()[0]; pix=self.plot.grab()
            if not pix.save(path): QMessageBox.critical(self,"Save Plot","Failed to save image.")

# ------------------------------- Main Window ----------------------------------
def spin_ros(n:TB3Node): rclpy.spin(n)

class Main(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("07 — SLAM GUI (final)")
        self.resize(1250, 800)

        rclpy.init(); self.node=TB3Node()
        self.ros_thread=threading.Thread(target=spin_ros, args=(self.node,), daemon=False); self.ros_thread.start()

        splitter=QSplitter(Qt.Horizontal,self); self.setCentralWidget(splitter)

        # Left column
        left=QWidget(); L=QVBoxLayout(left)
        speeds=QGroupBox("Speeds"); sL=QVBoxLayout(speeds)
        self.max_lin=0.22; self.max_ang=1.80
        row1=QHBoxLayout(); self.lbl_lin=QLabel(f"{self.max_lin:.2f} m/s")
        s1=QSlider(Qt.Horizontal); s1.setRange(0,100); s1.setValue(int(self.max_lin*100))
        s1.valueChanged.connect(lambda v:(setattr(self,'max_lin',v/100.0), self.lbl_lin.setText(f"{self.max_lin:.2f} m/s")))
        row1.addWidget(QLabel("Max Linear (m/s)")); row1.addWidget(self.lbl_lin,1,Qt.AlignRight); sL.addLayout(row1); sL.addWidget(s1)
        row2=QHBoxLayout(); self.lbl_ang=QLabel(f"{self.max_ang:.2f} rad/s")
        s2=QSlider(Qt.Horizontal); s2.setRange(0,300); s2.setValue(int(self.max_ang/3.0*300))
        s2.valueChanged.connect(lambda v:(setattr(self,'max_ang',(v/300.0)*3.0), self.lbl_ang.setText(f"{self.max_ang:.2f} rad/s")))
        row2.addWidget(QLabel("Max Angular (rad/s)")); row2.addWidget(self.lbl_ang,1,Qt.AlignRight); sL.addLayout(row2); sL.addWidget(s2)
        L.addWidget(speeds)

        tele=QGroupBox("Tele-op"); grid=QGridLayout(tele)
        mk=lambda v,w:(lambda:(setattr(self.node,'des_v',v), setattr(self.node,'des_w',w)))
        bF=QPushButton("Forward"); bB=QPushButton("Back"); bL=QPushButton("Left"); bR=QPushButton("Right"); bS=QPushButton("Stop")
        bF.clicked.connect(mk(self.max_lin,0)); bB.clicked.connect(mk(-self.max_lin,0)); bL.clicked.connect(mk(0,self.max_ang)); bR.clicked.connect(mk(0,-self.max_ang)); bS.clicked.connect(mk(0,0))
        grid.addWidget(bF,0,1); grid.addWidget(bL,1,0); grid.addWidget(bS,1,1); grid.addWidget(bR,1,2); grid.addWidget(bB,2,1)
        L.addWidget(tele)

        hud=QGroupBox("Robot State (MAP TF)"); hL=QVBoxLayout(hud)
        self.lbl_pose=QLabel("Pose(map): waiting TF…")
        self.lbl_vel =QLabel("Vel: lin=0.00 m/s  ang=0.00 rad/s")
        hL.addWidget(self.lbl_pose); hL.addWidget(self.lbl_vel); L.addWidget(hud)

        opts=QGroupBox("View Options"); oL=QVBoxLayout(opts)
        self.chk_follow=QCheckBox("Follow Robot"); self.chk_follow.setChecked(True)
        self.chk_grid  =QCheckBox("Show Grid");    self.chk_grid.setChecked(True)
        self.chk_map   =QCheckBox("Show Map");     self.chk_map.setChecked(True)
        oL.addWidget(self.chk_follow); oL.addWidget(self.chk_grid); oL.addWidget(self.chk_map)
        L.addWidget(opts)

        plot_box=QGroupBox("Speed (|linear x|, m/s)"); pL=QVBoxLayout(plot_box)
        self.plot=SpeedPlot(self.node); pL.addWidget(self.plot)
        row=QHBoxLayout(); b_pause=QPushButton("Pause Plot"); b_save=QPushButton("Save Plot")
        b_pause.clicked.connect(lambda:(self.plot.toggle(), b_pause.setText("Resume Plot" if self.plot.paused else "Pause Plot")))
        b_save.clicked.connect(lambda: QTimer.singleShot(0, lambda: self.plot.save_png(self)))
        row.addWidget(b_pause); row.addWidget(b_save); pL.addLayout(row)
        L.addWidget(plot_box,1)
        splitter.addWidget(left)

        # Right: canvas + toolbar
        right=QWidget(); R=QVBoxLayout(right)
        tb=QHBoxLayout()
        btn_center=QPushButton("Center on Robot"); btn_in=QPushButton("Zoom In"); btn_out=QPushButton("Zoom Out"); btn_view=QPushButton("Save View")
        btn_center.clicked.connect(lambda: self.canvas.center_on(self.node.mx, self.node.my))
        btn_in.clicked.connect(lambda: self._zoom(1.2)); btn_out.clicked.connect(lambda: self._zoom(1/1.2))
        btn_view.clicked.connect(lambda: QTimer.singleShot(0, lambda: self.canvas.save_view_png(self)))
        tb.addWidget(btn_center); tb.addWidget(btn_in); tb.addWidget(btn_out); tb.addStretch(1); tb.addWidget(btn_view)
        R.addLayout(tb)

        self.canvas=Canvas(self.node); R.addWidget(self.canvas,1); splitter.addWidget(right); splitter.setStretchFactor(1,1)

        # timers
        self.t=QTimer(self); self.t.timeout.connect(self._tick); self.t.start(100)
        QApplication.instance().aboutToQuit.connect(self._clean_shutdown)
        self.setFocusPolicy(Qt.StrongFocus)

    def keyPressEvent(self,e):
        k=e.key()
        if   k in (Qt.Key_W,Qt.Key_Up):    self.node.des_v=self.max_lin;  self.node.des_w=0.0
        elif k in (Qt.Key_S,Qt.Key_Down):  self.node.des_v=-self.max_lin; self.node.des_w=0.0
        elif k in (Qt.Key_A,Qt.Key_Left):  self.node.des_v=0.0;  self.node.des_w=self.max_ang
        elif k in (Qt.Key_D,Qt.Key_Right): self.node.des_v=0.0;  self.node.des_w=-self.max_ang
        elif k==Qt.Key_Space:              self.node.des_v=0.0;  self.node.des_w=0.0
        else: super().keyPressEvent(e)

    def _zoom(self,f):
        w=self.canvas.width() or 800; h=self.canvas.height() or 600
        mx,my=w/2,h/2; wx,wy=self.canvas.view_to_world(mx,my)
        self.canvas.scale=max(10.0,min(600.0,self.canvas.scale*f))
        self.canvas.offset_x=mx - wx*self.canvas.scale; self.canvas.offset_y=my + wy*self.canvas.scale
        self.canvas.update()

    def _tick(self):
        self.canvas.follow   = self.chk_follow.isChecked()
        self.canvas.show_grid= self.chk_grid.isChecked()
        self.canvas.show_map = self.chk_map.isChecked()
        self.canvas.update()
        mx,my,myaw, v_lin,v_ang, _, tf_ok = self.node.get_state_map()
        self.lbl_pose.setText(f"Pose(map): x={mx:.2f}  y={my:.2f}  yaw={math.degrees(myaw):.1f}°  [{'TF OK' if tf_ok else 'TF ?'}]")
        self.lbl_vel.setText(f"Vel: lin={v_lin:.2f} m/s  ang={v_ang:.2f} rad/s")
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
    app=QApplication([])
    win=Main()
    win.show()
    app.exec_()
