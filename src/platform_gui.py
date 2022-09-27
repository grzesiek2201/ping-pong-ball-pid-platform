from PyQt5.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QVBoxLayout,
    QWidget,
    QPushButton,
    QSlider,
    QLineEdit,
    QLabel
)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5 import QtCore
import sys

from pid_platform import Platform

import time

from collections import deque

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import pyqtgraph as pg


class MainWindow(QWidget):
    
    def __init__(self):
        super(MainWindow, self).__init__()
        
        self.init_gui()

    def init_gui(self):
        
        self.setGeometry(200, 200, 450, 620)
        self.setWindowTitle('Servo control')
        self.HBL = QVBoxLayout()
        
        # graph widget
        self.graph_widget = pg.PlotWidget()                
        
        # platform thread
        self.platform_worker = ThreadPlatform(self.graph_widget)
        self.platform_worker.image_update.connect(self.image_update_slot)
        self.platform_worker.finished.connect(self.platform_worker.stop)
        self.platform_worker.start()
        
        # big layouts
        self.pid_layout = QHBoxLayout()
        self.buttons_layout = QHBoxLayout()
        self.image_layout = QHBoxLayout()
        
        self.HBL.addLayout(self.pid_layout)
        self.HBL.addLayout(self.buttons_layout)
        self.HBL.addLayout(self.image_layout)
        
        # pid tweaking layouts for each servo
        self.pid_label_layout = QVBoxLayout()
        self.pid_layout_1 = QVBoxLayout()
        self.pid_layout_2 = QVBoxLayout()
        self.pid_layout_3 = QVBoxLayout()
        
        self.pid_layout.addLayout(self.pid_label_layout)
        self.pid_layout.addLayout(self.pid_layout_1)
        self.pid_layout.addLayout(self.pid_layout_2)
        self.pid_layout.addLayout(self.pid_layout_3)
        
        # pid layouts for each parameter in each servo
        self.layout_p_1 = QHBoxLayout()
        self.layout_i_1 = QHBoxLayout()
        self.layout_d_1 = QHBoxLayout()
        self.layout_p_2 = QHBoxLayout()
        self.layout_i_2 = QHBoxLayout()
        self.layout_d_2 = QHBoxLayout()
        self.layout_p_3 = QHBoxLayout()
        self.layout_i_3 = QHBoxLayout()
        self.layout_d_3 = QHBoxLayout()
        
        self.pid_layout_1.addLayout(self.layout_p_1)
        self.pid_layout_1.addLayout(self.layout_i_1)
        self.pid_layout_1.addLayout(self.layout_d_1)
        self.pid_layout_2.addLayout(self.layout_p_2)
        self.pid_layout_2.addLayout(self.layout_i_2)
        self.pid_layout_2.addLayout(self.layout_d_2)
        self.pid_layout_3.addLayout(self.layout_p_3)
        self.pid_layout_3.addLayout(self.layout_i_3)
        self.pid_layout_3.addLayout(self.layout_d_3)
        
        # pid entires
        self.pid_entry_1_p = QLineEdit(self)
        self.pid_entry_1_i = QLineEdit(self)
        self.pid_entry_1_d = QLineEdit(self)
        self.pid_entry_2_p = QLineEdit(self)
        self.pid_entry_2_i = QLineEdit(self)
        self.pid_entry_2_d = QLineEdit(self)
        self.pid_entry_3_p = QLineEdit(self)
        self.pid_entry_3_i = QLineEdit(self)
        self.pid_entry_3_d = QLineEdit(self)
        
        self.pid_entry_1_p.setMaximumWidth(60)
        self.pid_entry_1_i.setMaximumWidth(60)
        self.pid_entry_1_d.setMaximumWidth(60)
        self.pid_entry_2_p.setMaximumWidth(60)
        self.pid_entry_2_i.setMaximumWidth(60)
        self.pid_entry_2_d.setMaximumWidth(60)
        self.pid_entry_3_p.setMaximumWidth(60)
        self.pid_entry_3_i.setMaximumWidth(60)
        self.pid_entry_3_d.setMaximumWidth(60)
                        
        # pid buttons
        self.pid_minus_button_1_p = QPushButton("-")
        self.pid_minus_button_1_i = QPushButton("-")
        self.pid_minus_button_1_d = QPushButton("-")
        self.pid_minus_button_2_p = QPushButton("-")
        self.pid_minus_button_2_i = QPushButton("-")
        self.pid_minus_button_2_d = QPushButton("-")
        self.pid_minus_button_3_p = QPushButton("-")
        self.pid_minus_button_3_i = QPushButton("-")
        self.pid_minus_button_3_d = QPushButton("-")
        
        self.pid_plus_button_1_p = QPushButton("+")
        self.pid_plus_button_1_i = QPushButton("+")
        self.pid_plus_button_1_d = QPushButton("+")
        self.pid_plus_button_2_p = QPushButton("+")
        self.pid_plus_button_2_i = QPushButton("+")
        self.pid_plus_button_2_d = QPushButton("+")
        self.pid_plus_button_3_p = QPushButton("+")
        self.pid_plus_button_3_i = QPushButton("+")
        self.pid_plus_button_3_d = QPushButton("+")
        
        self.pid_minus_button_1_p.setMaximumWidth(35)
        self.pid_minus_button_1_i.setMaximumWidth(35)
        self.pid_minus_button_1_d.setMaximumWidth(35)
        self.pid_minus_button_2_p.setMaximumWidth(35)
        self.pid_minus_button_2_i.setMaximumWidth(35)
        self.pid_minus_button_2_d.setMaximumWidth(35)
        self.pid_minus_button_3_p.setMaximumWidth(35)
        self.pid_minus_button_3_i.setMaximumWidth(35)
        self.pid_minus_button_3_d.setMaximumWidth(35)
        
        self.pid_plus_button_1_p.setMaximumWidth(35)
        self.pid_plus_button_1_i.setMaximumWidth(35)
        self.pid_plus_button_1_d.setMaximumWidth(35)
        self.pid_plus_button_2_p.setMaximumWidth(35)
        self.pid_plus_button_2_i.setMaximumWidth(35)
        self.pid_plus_button_2_d.setMaximumWidth(35)
        self.pid_plus_button_3_p.setMaximumWidth(35)
        self.pid_plus_button_3_i.setMaximumWidth(35)
        self.pid_plus_button_3_d.setMaximumWidth(35)
        
        # organizing pid layouts
        p_label = QLabel("P")
        i_label = QLabel("I")
        d_label = QLabel("D")
        p_label.setMaximumWidth(10)
        i_label.setMaximumWidth(10)
        d_label.setMaximumWidth(10)
        self.pid_label_layout.addWidget(p_label)
        self.pid_label_layout.addWidget(i_label)
        self.pid_label_layout.addWidget(d_label)
        
        self.layout_p_1.addWidget(self.pid_minus_button_1_p)
        self.layout_p_1.addWidget(self.pid_entry_1_p)
        self.layout_p_1.addWidget(self.pid_plus_button_1_p)
        
        self.layout_i_1.addWidget(self.pid_minus_button_1_i)
        self.layout_i_1.addWidget(self.pid_entry_1_i)
        self.layout_i_1.addWidget(self.pid_plus_button_1_i)
        
        self.layout_d_1.addWidget(self.pid_minus_button_1_d)
        self.layout_d_1.addWidget(self.pid_entry_1_d)
        self.layout_d_1.addWidget(self.pid_plus_button_1_d)

        self.layout_p_2.addWidget(self.pid_minus_button_2_p)
        self.layout_p_2.addWidget(self.pid_entry_2_p)
        self.layout_p_2.addWidget(self.pid_plus_button_2_p)
        
        self.layout_i_2.addWidget(self.pid_minus_button_2_i)
        self.layout_i_2.addWidget(self.pid_entry_2_i)
        self.layout_i_2.addWidget(self.pid_plus_button_2_i)
        
        self.layout_d_2.addWidget(self.pid_minus_button_2_d)
        self.layout_d_2.addWidget(self.pid_entry_2_d)
        self.layout_d_2.addWidget(self.pid_plus_button_2_d)
        
        self.layout_p_3.addWidget(self.pid_minus_button_3_p)
        self.layout_p_3.addWidget(self.pid_entry_3_p)
        self.layout_p_3.addWidget(self.pid_plus_button_3_p)
        
        self.layout_i_3.addWidget(self.pid_minus_button_3_i)
        self.layout_i_3.addWidget(self.pid_entry_3_i)
        self.layout_i_3.addWidget(self.pid_plus_button_3_i)
        
        self.layout_d_3.addWidget(self.pid_minus_button_3_d)
        self.layout_d_3.addWidget(self.pid_entry_3_d)
        self.layout_d_3.addWidget(self.pid_plus_button_3_d)
        
        # buttons
        self.reset_button = QPushButton("Reset")
        self.reset_button.clicked.connect(self.reset_platform)
        
        self.stop_feed_button = QPushButton("Stop feed")
        self.stop_feed_button.clicked.connect(self.stop_feed)
        
        self.pid_update_button = QPushButton("Update PIDs")
        self.pid_update_button.clicked.connect(self.pid_update)
        
        self.start_feed_button = QPushButton("Start feed")
        self.start_feed_button.clicked.connect(self.start_feed)
        
        self.start_platform_button = QPushButton("Start platform")
        self.start_platform_button.clicked.connect(self.start_platform)
        
        self.stop_platform_button = QPushButton("Stop platform")
        self.stop_platform_button.clicked.connect(self.stop_platform)
        
        self.save_points_button = QPushButton("Save point data")
        self.save_points_button.clicked.connect(self.platform_worker.save_points)
        
        self.buttons_layout.addWidget(self.reset_button)
        self.buttons_layout.addWidget(self.start_feed_button)
        self.buttons_layout.addWidget(self.stop_feed_button)
        self.buttons_layout.addWidget(self.pid_update_button)
        self.buttons_layout.addWidget(self.start_platform_button)
        self.buttons_layout.addWidget(self.stop_platform_button)
        self.buttons_layout.addWidget(self.save_points_button)
        
        # add plot to layout
        #self.image_layout.addWidget(self.graph_widget)
        
        # video feed label
        self.feed_label = QLabel()
        self.image_layout.addWidget(self.feed_label)
        
        # main layout
        self.setLayout(self.HBL)
        
    def image_update_slot(self, image):
        self.feed_label.setPixmap(QPixmap.fromImage(image))
        
    def start_feed(self):
        self.platform_worker.start_feed()
        
    def stop_feed(self):
        self.platform_worker.stop_feed()
        
    def start_platform(self):
        self.update_pid_entries()
        self.platform_worker.start_platform()
        
    def stop_platform(self):
        self.platform_worker.stop_platform()
        
    def reset_platform(self):
        self.platform_worker.reset_platform()

    def pid_update(self):
        p_1 = float(self.pid_entry_1_p.text())
        i_1 = float(self.pid_entry_1_i.text())
        d_1 = float(self.pid_entry_1_d.text())
        p_2 = float(self.pid_entry_2_p.text())
        i_2 = float(self.pid_entry_2_i.text())
        d_2 = float(self.pid_entry_2_d.text())
        p_3 = float(self.pid_entry_3_p.text())
        i_3 = float(self.pid_entry_3_i.text())
        d_3 = float(self.pid_entry_3_d.text())
        self.platform_worker.set_pids((p_1, i_1, d_1), (p_2, i_2, d_2), (p_3, i_3, d_3))

    def update_pid_entries(self):
        try:
            pid_values = self.platform_worker.get_pid_values()
        except AttributeError:
            return
        self.pid_entry_1_p.setText(str(pid_values[0][0]))
        self.pid_entry_1_i.setText(str(pid_values[0][1]))
        self.pid_entry_1_d.setText(str(pid_values[0][2]))
        self.pid_entry_2_p.setText(str(pid_values[1][0]))
        self.pid_entry_2_i.setText(str(pid_values[1][1]))
        self.pid_entry_2_d.setText(str(pid_values[1][2]))
        self.pid_entry_3_p.setText(str(pid_values[2][0]))
        self.pid_entry_3_i.setText(str(pid_values[2][1]))
        self.pid_entry_3_d.setText(str(pid_values[2][2]))


class ThreadPlatform(QThread):
    
    image_update = pyqtSignal(QImage)
    
    def __init__(self, graph_widget):
        super(QThread, self).__init__()
        self.graph_widget = graph_widget
        
        #self.timer = QtCore.QTimer()
        #self.timer.setInterval(400)
        #self.timer.timeout.connect(self.update_plot)
        #self.timer.start()
    
    def run(self):
        self.feed_active = False
        self.platform_active = False
        self.thread_active = True
        self.x_points = deque()
        self.y_points = deque()
        
        self.platform = Platform(show=False)
        
        while self.thread_active:
            image, cx, cy = self.platform.one_tick(self.platform_active)
            
            self.x_points.append(cx)
            self.y_points.append(cy)
            
            height, width, channel = image.shape
            bytes_per_line = 3 * width
            image_qt_format = QImage(image.data.tobytes(), width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            
            if self.feed_active:
                self.image_update.emit(image_qt_format)
            
            time.sleep(0.05)
    
    def stop(self):
        self.thread_active = False
        self.quit()
        
    def save_points(self):
        print("save_points")
        with open("xy_points.csv", 'w') as file:
            for point in zip(self.x_points, self.y_points):
                file.write(f"{point[0]};{point[1]}\n")
        
    def update_plot(self):
        #self.graph_widget.plot(self.x_points)
        #self.graph_widget.plot(self.y_points)
        pass
        
    def start_feed(self):
        self.feed_active = True
                
    def stop_feed(self):
        self.feed_active = False
        
    def start_platform(self):
        self.platform_active = True
        
    def stop_platform(self):
        self.platform_active = False
        
    def reset_platform(self):
        self.platform.reset_pids()
        
    def set_pids(self, p1, p2, p3):
        self.platform.set_pids(p1, p2, p3)    
    
    def get_pid_values(self):
        return self.platform.get_pids()
    
if __name__ == '__main__':
    app = QApplication(sys.argv)
    with open('style.qss', 'r') as f:
        style = f.read()
    app.setStyleSheet(style)
    root = MainWindow()
    root.show()
    sys.exit(app.exec())
