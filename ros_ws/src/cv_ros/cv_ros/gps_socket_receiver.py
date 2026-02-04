#!/usr/bin/env python3
"""
GPS 坐标接收器

此程序用于在电脑上接收通过 socket 发送的 GPS 坐标。
不需要 ROS2 依赖，可以在任何安装了 Python 的电脑上运行。

使用方法:
python gps_socket_receiver.py [--host HOST] [--port PORT]

默认参数:
- host: 0.0.0.0 (监听所有网络接口)
- port: 5000
"""

import socket
import json
import argparse
import sys
import threading


def receive_connections(s, args):
    """接收连接的线程函数"""
    try:
        while True:
            # 等待连接
            conn, addr = s.accept()
            print(f"\n接收到来自 {addr} 的连接")
            
            try:
                # 接收数据
                data = conn.recv(1024).decode()
                if not data:
                    continue
                
                # 解析 JSON 数据
                try:
                    gps_data = json.loads(data)
                    
                    # 提取 GPS 坐标
                    latitude = gps_data.get('latitude', 'N/A')
                    longitude = gps_data.get('longitude', 'N/A')
                    altitude = gps_data.get('altitude', 'N/A')
                    timestamp = gps_data.get('timestamp', 'N/A')
                    
                    # 显示 GPS 坐标
                    print("===========================================")
                    print("接收到 GPS 坐标:")
                    print("===========================================")
                    print(f"纬度: {latitude}")
                    print(f"经度: {longitude}")
                    print(f"高度: {altitude} 米")
                    print(f"时间戳: {timestamp}")
                    print("===========================================")
                    
                    # 可以在这里添加处理逻辑，例如:
                    # - 存储到文件
                    # - 显示在地图上
                    # - 发送到其他系统
                    
                except json.JSONDecodeError as e:
                    print(f"解析 JSON 数据失败: {e}")
                    print(f"原始数据: {data}")
                
            except Exception as e:
                print(f"处理连接时出错: {e}")
            finally:
                # 关闭连接
                conn.close()
                print(f"已关闭与 {addr} 的连接")
                print("继续等待下一个连接...")
                
    except Exception as e:
        # 当 socket 被关闭时，accept() 会抛出异常
        if not isinstance(e, OSError) or "[WinError 10038]" not in str(e):
            print(f"接收连接时出错: {e}")


def main():
    """主函数"""
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='GPS 坐标接收器')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='监听地址')
    parser.add_argument('--port', type=int, default=5000, help='监听端口')
    args = parser.parse_args()

    # 创建 socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    try:
        # 绑定地址和端口
        s.bind((args.host, args.port))
        print(f"GPS 坐标接收器已启动，监听 {args.host}:{args.port}")
        print("等待 GPS 坐标数据...")
        print("按 'q' 键退出")
        
        # 开始监听
        s.listen(1)
        
        # 创建并启动接收连接的线程
        receive_thread = threading.Thread(target=receive_connections, args=(s, args))
        receive_thread.daemon = True
        receive_thread.start()
        
        # 主线程等待用户输入
        while True:
            user_input = input()
            if user_input.lower() == 'q':
                print("\n收到退出命令，正在退出...")
                break
                
    except Exception as e:
        print(f"错误: {e}")
    finally:
        # 关闭 socket
        s.close()
        print("GPS 坐标接收器已关闭")


if __name__ == '__main__':
    main()
