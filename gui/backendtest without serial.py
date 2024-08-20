from CameraSystenGUI import MainWindow as CameraSystemMainWindow
from frontendtest import Ui_MainWindow as Ui_MainWindow_main
from MainStereoVisionWindow import MyWidget
import sys
import cv2
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget, QPushButton, QDialog
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCore import QTimer
from frontendManual import Ui_ManualControl
from autonomousfrontend import Ui_MainWindow as Ui_MainWindow_auto

class WebCam(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Web Camera')
        layout = QVBoxLayout()
        self.videoLabel = QLabel("Press 'show' to show the live webcam!")
        self.videoLabel.setFixedSize(640, 480)  # Set size to match the webcam feed
        layout.addWidget(self.videoLabel)
        # Add "Show" button
        self.showButton = QPushButton("Show")
        self.showButton.clicked.connect(self.startCamera)
        layout.addWidget(self.showButton)
        # Add "Screenshot" button
        self.screenshotButton = QPushButton("Screenshot")
        self.screenshotButton.clicked.connect(self.takeScreenshot)
        layout.addWidget(self.screenshotButton)

        self.setLayout(layout)

        # Initialize video capture and timer
        self.cap = cv2.VideoCapture(0)
        self.timer = QTimer()
        self.timer.timeout.connect(self.updateframe)

    def startCamera(self):
        if not self.timer.isActive():
            self.timer.start(30)

    def updateframe(self):
        rval, frame = self.cap.read()
        if rval:
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            qimg = QImage(rgb_image.data, w, h, ch * w, QImage.Format.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)
            self.videoLabel.setPixmap(pixmap)

    def takeScreenshot(self):
        if self.cap.isOpened():
            rval, frame = self.cap.read()
            if rval:
                cv2.imwrite('screenshot.png', frame)

    def closeEvent(self, event):
        self.timer.stop()
        self.cap.release()
        event.accept()


class CarControl(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Car Control')
        layout = QVBoxLayout()

        # Setting up label
        self.label = QLabel("Car Control")
        layout.addWidget(self.label)

        # Creating and setting up buttons
        self.autonomousButton = QPushButton("Autonomous")
        layout.addWidget(self.autonomousButton)
        self.manualButton = QPushButton("Manual")
        layout.addWidget(self.manualButton)
        self.manualButton.clicked.connect(self.open_manual_control)
        self.autonomousButton.clicked.connect(self.open_autonomous_control)
        self.setLayout(layout)

        # self.connectSerial()
    def open_autonomous_control(self):
        self.autoWindow = QMainWindow()
        self.ui_auto = Ui_MainWindow_auto()
        self.ui_auto.setupUi(self.autoWindow)
        self.autoWindow.show()
    def open_manual_control(self):
         #setting up manual window
         self.manual_window = QMainWindow()
         self.ui_manual = Ui_ManualControl()
         self.ui_manual.setupUi(self.manual_window)
         #showing manual window
         self.manual_window.show()



class MainWindow(QMainWindow):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.ui = Ui_MainWindow_main()
        self.ui.setupUi(self)
        self.ui.carcontrolbutton.clicked.connect(self.show_new_window)
        self.ui.liveCamButton.clicked.connect(self.webCamDialogue)
        self.ui.videoStitchButton.clicked.connect(self.video_stitching)
        self.ui.stereoVisionButton.clicked.connect(self.stereo_vision)
    def show_new_window(self):
        self.w = CarControl()
        self.w.show()
    def video_stitching(self):
        self.w = CameraSystemMainWindow()
        self.w.show()
    def stereo_vision(self):
        self.w = MyWidget()
        self.w.show()
    def webCamDialogue(self):
        self.w=WebCam()
        self.w.show()
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())