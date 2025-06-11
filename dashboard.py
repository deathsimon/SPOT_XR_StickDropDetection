import cv2
import configparser
import time
import sys
# import threading
import numpy as np
from skimage.metrics import structural_similarity as ssim
from PySide6.QtGui import QIcon, QPixmap, QImage, QGuiApplication
from PySide6.QtCore import QThread, Signal, Slot, Qt, QEvent
from PySide6.QtQuickControls2 import QQuickStyle
from PySide6.QtWidgets import QLabel, QWidget, QApplication, QGridLayout, QMainWindow, QSizePolicy, QPushButton, QVBoxLayout

from stick_detection import StickDetector

class GuiContainer(QMainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # setup        
        self.setWindowTitle("Stick Detector")
        self.setGeometry(100, 100, 640, 480)
        
        # create the detection thread
        self.runner_thread = StickDetector(self, config_file="config")
        self.runner_thread.updateFrame.connect(self.setImage)
        self.runner_thread.start()
        self.runner_thread.setPriority(QThread.Priority.HighestPriority) # or TimeCritical

        # create a label to house the imagestream
        self.label = QLabel()
        self.label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.label.setMouseTracking(True)
        
        # install mouse & keyboard handling
        self.installEventFilter(self.runner_thread)
        self.setFocus()
        
        # buttons
        self.btn_reset        = QPushButton()
        self.btn_open_full    = QPushButton()
        self.btn_open_partial = QPushButton()
        self.btn_close        = QPushButton()
        self.btn_sit          = QPushButton()
        self.btn_stand        = QPushButton()
        
        # layout_right_bar = QVBoxLayout()
        # layout_right_bar.addWidget(self.btn_reset)
        # layout_right_bar.addWidget(self.btn_open_full)
        # layout_right_bar.addWidget(self.btn_open_partial)
        # layout_right_bar.addWidget(self.btn_close)
        # layout_right_bar.addWidget(self.btn_sit)
        # layout_right_bar.addWidget(self.btn_stand)
        # widget = QWidget()
        # widget.setLayout(layout_right_bar)
        # self.addDockWidget(widget)
        
        # create the layouts
        layout = QGridLayout()
        layout.addWidget(self.label)
        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(self.label)
    
    @Slot(QImage)
    def setImage(self, image):
        #update image
        image = image.scaled(self.label.width(), self.label.height(), Qt.KeepAspectRatio)
        self.label.setPixmap(QPixmap.fromImage(image)) 
    
    def __del__(self):
        self.runner_thread.running = False
        self.label.destroy()

if __name__ == "__main__":
    # initialize Qt-app
    app = QApplication(sys.argv)
    QQuickStyle.setStyle("Material")
    
    gui = GuiContainer()
    gui.show()
    
    sys.exit(app.exec())