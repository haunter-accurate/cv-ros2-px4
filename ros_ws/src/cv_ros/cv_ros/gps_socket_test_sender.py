#!/usr/bin/env python3
"""
GPS 坐标发送器（测试用）

此程序用于测试 GPS 坐标接收器，模拟无人机发送 GPS 数据。
不需要 ROS2 依赖，可以在任何安装了 Python 的电脑上运行。

使用方法:
python gps_socket_test_sender.py [--host HOST] [--port PORT]

默认参数:
- host: localhost
- port: 5000
"""

import socket
import json
import argparse
import time


def main():
    """主函数"""
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='GPS 坐标发送器（测试用）')
    parser.add_argument('--host', type=str, default='localhost', help='服务器地址')
    parser.add_argument('--port', type=int, default=5000, help='服务器端口')
    args = parser.parse_args()

    print("===========================================")
    print("GPS 坐标发送器（测试用）")
    print("===========================================")
    print(f"发送到: {args.host}:{args.port}")
    print("准备发送测试 GPS 数据...")
    print("===========================================")

    # 测试 GPS 数据
    test_gps_data = {
        'latitude': 39.9042,
        'longitude': 116.4074,
        'altitude': 10.5,
        'timestamp': int(time.time() * 1000000)  # 微秒时间戳
    }

    try:
        # 创建 socket
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # 连接到服务器
        client_socket.connect((args.host, args.port))
        print("✅ 成功连接到服务器")
        
        # 发送数据
        data = json.dumps(test_gps_data).encode('utf-8')
        client_socket.sendall(data)
        print("✅ GPS 数据发送成功")
        
        # 显示发送的数据
        print("\n===========================================")
        print("发送的 GPS 数据:")
        print("===========================================")
        print(f"纬度: {test_gps_data['latitude']}")
        print(f"经度: {test_gps_data['longitude']}")
        print(f"高度: {test_gps_data['altitude']} 米")
        print(f"时间戳: {test_gps_data['timestamp']}")
        print("===========================================")

    except ConnectionRefusedError:
        print("❌ 连接失败: 服务器未启动或地址错误")
        print("请先启动 GPS 坐标接收器")
    except Exception as e:
        print(f"❌ 发送失败: {e}")
    finally:
        # 关闭连接
        try:
            client_socket.close()
        except:
            pass
        print("\n✅ 测试完成")


if __name__ == '__main__':
    main()
