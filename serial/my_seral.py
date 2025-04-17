import serial

ser = serial.Serial('/dev/ttyS0', 115200)  # 使用你系统中的串口设备路径
ser.timeout = 1  # 设置超时时间

# 读取数据并打印
while True:
    if ser.in_waiting > 0:
        data = ser.readline()
        print(data.decode('utf-8').strip())