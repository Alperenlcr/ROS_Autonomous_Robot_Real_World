#!/usr/bin/python3.8
import cv2
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout
from qt_utils import Ui_MainWindow
from sensor_msgs.msg import Image
import sys
from PyQt5.QtGui import QColor, QImage, QPixmap
from PyQt5.QtCore import QTimer
import rospy
from cv_bridge import CvBridge, CvBridgeError


class App(QtWidgets.QMainWindow):
    def __init__(self):
        super(App, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setCentralWidget(self.ui.centralwidget)

        # time
        self.date_time_edit = QtWidgets.QDateTimeEdit()
        self.date_time_edit.setDisplayFormat("dd/MM/yyyy hh:mm:ss")

        # info combined
        self.viewfinder = QLabel(self)
        self.viewfinder.setAlignment(QtCore.Qt.AlignCenter)  # Görüntüyü merkeze hizala
        self.ui.verticalLayout.addWidget(self.viewfinder)
        self.ui.centralwidget.setLayout(self.ui.verticalLayout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.dateTime)
        self.timer.timeout.connect(self.listener)
        self.timer.start(30)  # 30 ms'de bir güncelleme yap


    def listener(self):
        frame = rospy.wait_for_message('info_combined', Image)
        rgb_image = bridge.imgmsg_to_cv2(frame, desired_encoding="passthrough")
        cv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)

        # Calculate the new dimensions for resizing
        height, width = cv_image.shape[:2]
        new_width = int(width * 0.75)
        new_height = int(height * 0.75)

        # Resize the image to the new dimensions
        resized_image = cv2.resize(cv_image, (new_width, new_height))


        h, w, ch = resized_image.shape
        bytes_per_line = ch * w
        image = QImage(resized_image.data, w, h, bytes_per_line, QImage.Format_RGB888)

        pixmap = QPixmap.fromImage(image)
        self.viewfinder.setPixmap(pixmap)


    def dateTime(self):
        self.date_time_edit.setDateTime(QtCore.QDateTime.currentDateTime())
        self.ui.dateTimeEdit.setDateTime(self.date_time_edit.dateTime())
        

if __name__ == '__main__':
    rospy.init_node('interface_QT', anonymous=True)
    bridge = CvBridge()
    app=QtWidgets.QApplication(sys.argv)
    win=App()
    win.setWindowTitle("ROBOT INTERFACE")
    background_color = QColor(192, 192, 192)  
    win.setStyleSheet(f"background-color: {background_color.name()};")
    win.show()
    sys.exit(app.exec_())