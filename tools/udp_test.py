import socket
import time
import math
import struct

UDP_IP = "127.0.0.1"  # 接收端IP
UDP_PORT = 12345

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

frequency = 0.5  # 正弦波频率
amplitude = 100   # 幅值
phase = 0

while True:
    value = amplitude * math.sin(2 * math.pi * frequency * phase)
    value1 = amplitude * math.sin(2 * math.pi * frequency * phase) * 2
    value2 = amplitude * math.sin(2 * math.pi * frequency * phase) * 3
    value3 = amplitude * math.sin(2 * math.pi * frequency * phase) * 4
    data = struct.pack('<4f', *[value,value1,value2,value3])  # 打包为 float32
    sock.sendto(data, (UDP_IP, UDP_PORT))
    phase += 0.01
    time.sleep(0.01)  # 发送间隔