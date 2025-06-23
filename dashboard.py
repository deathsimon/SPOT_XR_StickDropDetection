import sys

# import threading
from PySide6.QtGui import QPixmap, QImage, QPainter, QFont, QBrush, QPen
from PySide6.QtCore import QThread, Slot, Qt, QRect, QMargins
from PySide6.QtQuickControls2 import QQuickStyle
from PySide6.QtWidgets import QWidget, QApplication, QMainWindow, QStyle, QPushButton
from PySide6.QtUiTools import QUiLoader
import PySide6
from PySide6.QtWebEngineWidgets import QWebEngineView


from stick_detection import StickDetector


class GuiContainer(QMainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # setup
        self.setWindowTitle("Stick Detector")
        self.setGeometry(100, 100, 640, 480)

        # load from file
        loader = QUiLoader()
        self.window: QMainWindow = loader.load("dashboard_v2.ui", None)

        # create the detection thread
        self.runner_thread = StickDetector(self, config_file="config")
        self.runner_thread.updateFrame.connect(self.setImage)
        self.runner_thread.start()
        self.runner_thread.setPriority(
            QThread.Priority.HighestPriority
        )  # or TimeCritical

        # install mouse & keyboard handling
        self.installEventFilter(self.runner_thread)
        self.setFocus()

        # create the layouts
        self.setCentralWidget(self.window)
        self.window.ministryLogo = QWebEngineView()
        self.window.ministryLogo.load(
            "https://www.bmbf.de/SiteGlobals/Frontend/Images/images/logo-en.svg?__blob=normal&v=4"
        )
        self.window.ministryLogo.page().settings().setAttribute(
            PySide6.QtWebEngineCore.QWebEngineSettings.WebAttribute.ShowScrollBars,
            False,
        )
        # handle all the buttons predictiveSched_btn
        self.window.predictiveSched_btn.clicked.connect(
            lambda: self.runner_thread.keyboard_callback(",")
        )
        self.window.reactiveSched_btn.clicked.connect(
            lambda: self.runner_thread.keyboard_callback(".")
        )
        # self.window.detectionStart_btn.clicked.connect(lambda: self.runner_thread.keyboard_callback('d'))
        # self.window.detectionstop_btn.clicked.connect(lambda: self.runner_thread.keyboard_callback('d'))
        # self.window.spotReady_btn.clicked.connect(lambda: self.runner_thread.keyboard_callback('r'))
        # self.window.reset_btn.setIcon(self.style().standardIcon(QStyle.SP_BrowserReload))
        # self.window.gripperOpen_btn.clicked.connect(lambda: self.runner_thread.keyboard_callback('o'))
        # self.window.gripperClose_btn.clicked.connect(lambda: self.runner_thread.keyboard_callback('c'))
        # self.window.sitStand_btn.clicked.connect(lambda: self.runner_thread.keyboard_callback('s'))
        # self.window.dropStick_btn.clicked.connect(lambda: self.runner_thread.keyboard_callback('1'))
        # self.window.raiseStick_btn.clicked.connect(lambda: self.runner_thread.keyboard_callback('2'))

        # self.window.save_aoi_btn.clicked.connect(lambda: self.runner_thread._save_aoi())
        # self.window.reload_cfg_btn.clicked.connect(lambda: self.runner_thread._reload_settings(self.runner_thread.reloadable_config))

    def _dynamic_btn_sizer(self, btn: QPushButton):
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

    def draw_status_tag(self, painter, x, y, w, h, text, color):
        # set fontsize
        painter.setFont(QFont("Arial", 20))

        # draw borders around text and background
        painter.setPen(QPen(color, 3))
        painter.pen().setJoinStyle(Qt.PenJoinStyle.MiterJoin)
        rect = painter.fontMetrics().boundingRect(text)
        rect.moveTo(x, y - painter.fontMetrics().height() * 1.2)
        rect += QMargins(0, 5, 10, 0)
        painter.drawRect(rect)
        painter.fillRect(rect, QBrush(color, Qt.BrushStyle.SolidPattern))

        # draw border around frame
        painter.drawRect(x, y, w, h)

        # draw the actual text
        painter.setPen(QPen(Qt.GlobalColor.white, 1))
        painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, text)

    @Slot(QImage)
    def setImage(self, image: QImage):
        # update image
        self.frame_width, self.frame_height = image.width(), image.height()
        image = image.scaled(
            self.window.video_stream.width(),
            self.window.video_stream.height(),
            Qt.KeepAspectRatio,
        )
        self.scaled_width, self.scaled_height = image.width(), image.height()
        sx, sy = (
            self.scaled_width / self.frame_width,
            self.scaled_height / self.frame_height,
        )

        if self.runner_thread.qt_drawing == True:
            with QPainter(image) as painter:

                # draw fps
                painter.setRenderHint(QPainter.RenderHint.TextAntialiasing, True)
                painter.setPen(Qt.GlobalColor.white)
                painter.setFont(QFont("Sans Serif", 16))
                painter.drawText(
                    QRect(20, 20, 100, 100),
                    Qt.AlignmentFlag.AlignLeft,
                    f"{self.runner_thread.fps:03.1f}",
                )

                # draw aoi
                if (
                    self.runner_thread.selecting_aoi
                    and self.runner_thread.start_point
                    and self.runner_thread.end_point
                ):
                    x1, y1 = self.runner_thread.start_point
                    x2, y2 = self.runner_thread.end_point
                    self.draw_status_tag(
                        painter,
                        x1 * sx,
                        y1 * sy,
                        (x2 - x1) * sx,
                        (y2 - y1) * sy,
                        "Selecting",
                        Qt.GlobalColor.blue,
                    )
                elif self.runner_thread.aoi:
                    x, y, w, h = self.runner_thread.aoi
                    if self.runner_thread.detecting:
                        aoi_color = Qt.GlobalColor.green  # Green for detecting
                        status_tag = "Detecting"
                    else:
                        aoi_color = Qt.GlobalColor.red  # Red for not detecting
                        status_tag = "Detected"
                    self.draw_status_tag(
                        painter, x * sx, y * sy, w * sx, h * sy, status_tag, aoi_color
                    )

        self.window.video_stream.setPixmap(QPixmap.fromImage(image))


if __name__ == "__main__":
    # initialize Qt-app
    app = QApplication(sys.argv)
    QQuickStyle.setStyle("Material")

    gui = GuiContainer()
    gui.show()

    sys.exit(app.exec())
