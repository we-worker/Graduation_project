import os
import sys
import logging
from multiprocessing import freeze_support


from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog, QGraphicsItem, QGraphicsPixmapItem,QGraphicsScene, QGraphicsView
from PyQt5.QtCore import QTimer, Qt, QCoreApplication, QRect, QRectF, QSize
from PyQt5 import uic, QtWebEngineWidgets
from PyQt5.QtGui import QIcon,QFont,QTextCursor,QPainter, QPixmap, QWheelEvent

from qt_material import apply_stylesheet, QtStyleTools, density

from tcp_logic import TcpLogic
from main_ui import Ui_MainWindow  




QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
app = QApplication([])
freeze_support()


class RuntimeStylesheets(QMainWindow, QtStyleTools):
    def __init__(self):
        super().__init__()
        self.main = Ui_MainWindow()
        self.main.setupUi(self)

        self.setWindowTitle("电弧无人车上位机")
        logo = QIcon("qt_material:/logo/logo.svg")
        self.setWindowIcon(logo)

        self.client = TcpLogic()
        self.display_image()  # 显示图片

        app.lastWindowClosed.connect(app.quit)

    
    def write_debug(self, msg):
        # signal_write_msg信号会触发这个函数
        """
        功能函数，向debug区写入数据的方法
        """
        self.main.Debug_Text.insertPlainText(msg)
        # 滚动条移动到结尾
        self.main.Debug_Text.moveCursor(QTextCursor.End)

    def connect_btn_clicked(self):
        # 检查连接是否成功
        if self.client.link:
            self.client.tcp_close()

            self.main.connect_btn.setStyleSheet("background-color: white")
            self.main.connect_btn.setText("连接")
        else:
            ip = self.main.ip_Edit.text()
            port = int(self.main.port_Edit.text())
            self.client.tcp_client_start(ip, port)
            if self.client.link:
                self.main.connect_btn.setStyleSheet("background-color: #00bcd4 ; color: #FFFFFF")
                self.main.connect_btn.setText("断开")
            else:
                self.write_debug("无法连接目标服务器\n")

    def display_image(self):
        # 1. 加载图片到 QPixmap 对象
        pixmap = QPixmap('902.pgm')
        # 2. 创建一个 QGraphicsPixmapItem 对象，使用 QPixmap 对象作为参数
        pixmap_item = QGraphicsPixmapItem(pixmap)
        # 3. 创建一个 QGraphicsScene 对象
        scene = QGraphicsScene()
        # 4. 将 QGraphicsPixmapItem 对象添加到 QGraphicsScene 对象中
        scene.addItem(pixmap_item)
        # 5. 将 QGraphicsScene 对象设置为 graphicsView 的场景
        self.main.graphicsView.setScene(scene)
        
    def set_target_clicked(self):
        self.main.graphicsView._setTargetPoseMode=True
    def start_all_clicked(self):
        pass


           



# Extra stylesheets
extra = {
    # Button colors
    'danger': '#dc3545',
    'warning': '#ffc107',
    'success': '#17a2b8',
    # Font
    # 'font_family': 'monoespace',
    'font_size': '13px',
}

if __name__ == "__main__":
    theme = 'light_cyan_500'
    apply_stylesheet(
        app,
        theme + '.xml',
        invert_secondary=('light' in theme and 'dark' not in theme),
        extra=extra,
    )

    frame = RuntimeStylesheets()
    frame.show()
    app.exec_()
