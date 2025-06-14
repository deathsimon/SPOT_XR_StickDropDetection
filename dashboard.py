import sys
# import threading
from PySide6.QtGui import QPixmap, QImage
from PySide6.QtCore import QThread, Slot, Qt
from PySide6.QtQuickControls2 import QQuickStyle
from PySide6.QtWidgets import QWidget, QApplication, QMainWindow, QStyle, QPushButton
from PySide6.QtUiTools import QUiLoader

from stick_detection import StickDetector

class GuiContainer(QMainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # setup        
        self.setWindowTitle("Stick Detector")
        self.setGeometry(100, 100, 640, 480)
        
        # load from file
        loader = QUiLoader()
        self.window : QWidget = loader.load("dashboard.ui", None)
        
        # create the detection thread
        self.runner_thread = StickDetector(self, config_file="config")
        self.runner_thread.updateFrame.connect(self.setImage)
        self.runner_thread.start()
        self.runner_thread.setPriority(QThread.Priority.HighestPriority) # or TimeCritical
        
        # install mouse & keyboard handling
        self.installEventFilter(self.runner_thread)
        self.setFocus()
        
        # create the layouts
        self.setCentralWidget(self.window)
        
        # handle all the buttons
        self.window.detection_btn.clicked.connect(lambda: self.runner_thread.keyboard_callback('d'))
        self.window.reset_btn.clicked.connect(lambda: self.runner_thread.keyboard_callback('r'))
        self.window.reset_btn.setIcon(self.style().standardIcon(QStyle.SP_BrowserReload))
        self.window.open_full_btn.clicked.connect(lambda: self.runner_thread.keyboard_callback('o'))
        self.window.sit_btn.clicked.connect(lambda: self.runner_thread.keyboard_callback('s'))
        self.window.close_btn.clicked.connect(lambda: self.runner_thread.keyboard_callback('c'))
        self.window.drop_stick_button.clicked.connect(lambda: self.runner_thread.keyboard_callback('1'))
        
        self.window.save_aoi_btn.clicked.connect(lambda: self.runner_thread._save_aoi())
        self.window.reload_cfg_btn.clicked.connect(lambda: self.runner_thread._reload_settings(self.runner_thread.reloadable_config))
    
    def _dynamic_btn_sizer(self, btn : QPushButton):
        font = btn.font()
        height = btn.height()
        font.setPixelSize(int(height * 0.5))
        btn.setFont(font)
        
        icon_size = btn.iconSize()
        icon_size.scale(10000, int(height * 0.6), Qt.AspectRatioMode.KeepAspectRatio)
        btn.setIconSize(icon_size)

    
    def resizeEvent(self, event):
        super().resizeEvent(event)
        for btn in dir(self.window):
            if isinstance(getattr(self.window, btn), QPushButton):
                self._dynamic_btn_sizer(getattr(self.window, btn))
    
    @Slot(QImage)
    def setImage(self, image : QImage):
        #update image
        self.frame_width, self.frame_height = image.width(), image.height()
        image = image.scaled(self.window.video_stream.width(), self.window.video_stream.height(), Qt.KeepAspectRatio)
        self.scaled_width, self.scaled_height = image.width(), image.height()
        self.window.video_stream.setPixmap(QPixmap.fromImage(image)) 

if __name__ == "__main__":
    # initialize Qt-app
    app = QApplication(sys.argv)
    QQuickStyle.setStyle("Material")
    
    gui = GuiContainer()
    gui.show()
    
    sys.exit(app.exec())