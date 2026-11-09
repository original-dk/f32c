"""列出电脑当前可见的串口。"""

from serial.tools import list_ports

ports = list(list_ports.comports())
if not ports:
    print("未发现串口。请检查 USB 转 TTL 模块及其驱动。")
else:
    for port in ports:
        print(f"{port.device}\t{port.description}")
