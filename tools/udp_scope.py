import sys
import socket
import struct
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# ========================== UDP 配置 ==========================
UDP_IP = "0.0.0.0"
UDP_PORT = 12345
BUFFER_SIZE = 10000

CURVE_NUM = 4

# ========================== PyQtGraph 窗口配置 ==========================
class RealTimePlotWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        
        # 初始化窗口
        self.setWindowTitle("Real-Time UDP Waveform")
        self.resize(800, 400)
        
        # 创建绘图部件
        self.plot_widget = pg.PlotWidget()
        self.setCentralWidget(self.plot_widget)

        self.legend = pg.LegendItem((80,60), offset=(70,20))
        self.legend.setParentItem(self.plot_widget.getPlotItem()) 

        self.curves = []
        self.curves_data = []

        pen_colors = [
            (255,0,0),
            (0,255,0),
            (0,0,255),
            (100,100,100)
        ]
        # 初始化曲线
        for i in range(CURVE_NUM):
            plot_name = f'wave{i}'
            plot = self.plot_widget.plot(pen=pen_colors[i],name=plot_name)
            plot_data = np.zeros(BUFFER_SIZE)  # 初始数据缓冲区
            self.legend.addItem(plot, plot_name)
            self.curves.append(plot)
            self.curves_data.append(plot_data)
        
    def update_plot(self,values):
        for i in range(CURVE_NUM):
            plot_data = self.curves_data[i]

            plot_data[:-1] = plot_data[1:]
            plot_data[-1] = values[i] 
            self.curves[i].setData(plot_data)

# ========================== UDP 接收线程 ==========================
class UdpReceiverThread(QtCore.QThread):
    data_received = QtCore.pyqtSignal(tuple)  # 定义信号

    def __init__(self):
        super().__init__()
        self.running = True

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
        print(f"Listening on UDP {UDP_IP}:{UDP_PORT}...")

        while self.running:
            try:
                data, _ = sock.recvfrom(BUFFER_SIZE)
                value = struct.unpack('<4f', data)  # 解析为float
                self.data_received.emit(value)  # 发射信号
            except Exception as e:
                print(f"Error: {e}")
                break

    def stop(self):
        self.running = False

# ========================== 主程序 ==========================
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = RealTimePlotWindow()
    window.show()

    # 创建并启动UDP线程
    udp_thread = UdpReceiverThread()
    udp_thread.data_received.connect(window.update_plot)  # 连接信号到数据更新
    
    udp_thread.start()

    # 运行应用
    sys.exit(app.exec_())