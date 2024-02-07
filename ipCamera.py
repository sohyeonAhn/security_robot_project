# ip 카메라 관련 클래스
# ipCamera.py
import cv2
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, Qt
from PyQt5.QtGui import QImage, QPixmap

class Camera(QThread):
    update_image = pyqtSignal(QImage)

    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent
        self.camera = cv2.VideoCapture('rtsp://admin:admin0102@192.168.10.2:554/stream1')

    def run(self):
        while True:
            ret, frame = self.camera.read()
            if ret:
                rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                q_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                self.update_image.emit(q_image)

    def stop(self):
        self.camera.release()
