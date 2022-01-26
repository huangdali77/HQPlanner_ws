#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
import multiprocessing as mp
import pyqtgraph as pg
from PySide2 import QtWidgets, QtCore
import sys
import os

outconn, inconn = mp.Pipe()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.win = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self.win)
        self.p = self.win.addPlot(title="data")
        self.p.showGrid(x=True, y=True)
        self.p.setXRange(0, 100, padding=0)
        self.p.setYRange(-1, 1, padding=0.1)
        pen = pg.mkPen(width=2)
        self.line = self.p.plot(pen=pen)
        self.timer = QtCore.QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def update_plot_data(self):
        global inconn
        data = inconn.recv()
        self.line.setData(data)


def callback(msg):
    global outconn
    outconn.send(msg.data)
    # print('I here U!')


def StartSubscriber():
    rospy.init_node('recver')
    rospy.Subscriber('data1d', Float32MultiArray, callback)
    rospy.spin()


job = mp.Process(target=StartSubscriber, args=())
job.start()
app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
w.show()
sys.exit(app.exec_())