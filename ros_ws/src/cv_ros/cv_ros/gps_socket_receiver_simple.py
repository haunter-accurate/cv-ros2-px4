#!/usr/bin/env python3
"""
GPS 坐标接收器（简化版）

此程序用于在电脑上接收通过 socket 发送的 GPS 坐标。
不需要 ROS2 依赖，可以在任何安装了 Python 的电脑上运行。

使用方法:
python gps_socket_receiver_simple.py

默认监听端口: 5000

退出方法:
- 按 Ctrl+C 退出
"""

import socket
import json
import sys
import signal

# 全局变量，用于控制程序运行
running = True


def signal_handler(sig, frame):
    """信号处理函数，用于捕获Ctrl+C"""
    global running
    running = False
    print("\n接收到终止信号，正在退出...")


def main():
    """主函数"""
    # 注册信号处理函数
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print("===========================================")
    print("GPS 坐标接收器")
    print("===========================================")
    print("监听端口: 5000")
    print("等待 GPS 坐标数据...")
    print("按 Ctrl+C 退出")
    print("===========================================")

    try:
        # 创建 socket 服务器
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind(('0.0.0.0', 5000))
            server_socket.listen(5)
            
            # 设置 socket 超时，以便及时响应终止信号
            server_socket.settimeout(1.0)
            
            print("\n服务器已启动，正在等待连接...")

            # 持续接收连接
            while running:
                try:
                    # 等待连接
                    client_socket, client_address = server_socket.accept()
                    with client_socket:
                        print(f"\n✅ 接收到来自 {client_address} 的连接")

                        # 接收数据
                        data = client_socket.recv(1024)
                        if data:
                            try:
                                # 解析 JSON 数据
                                gps_data = json.loads(data.decode('utf-8'))
                                
                                # 提取 GPS 坐标
                                latitude = gps_data.get('latitude', 'N/A')
                                longitude = gps_data.get('longitude', 'N/A')
                                altitude = gps_data.get('altitude', 'N/A')
                                timestamp = gps_data.get('timestamp', 'N/A')
                                
                                # 显示 GPS 坐标
                                print("\n===========================================")
                                print("接收到 GPS 坐标:")
                                print("===========================================")
                                print(f"纬度: {latitude}")
                                print(f"经度: {longitude}")
                                print(f"高度: {altitude} 米")
                                print(f"时间戳: {timestamp}")
                                print("===========================================")
                                
                            except json.JSONDecodeError as e:
                                print(f"❌ 解析 JSON 数据失败: {e}")
                                print(f"原始数据: {data}")
                        else:
                            print("❌ 未接收到数据")

                except socket.timeout:
                    # 超时是正常的，只是为了检查是否需要退出
                    continue
                except ConnectionResetError:
                    print("❌ 连接被客户端重置")
                except Exception as e:
                    if not running and "[WinError 10038]" in str(e):
                        # 忽略 socket 关闭错误
                        break
                    print(f"❌ 处理连接时出错: {e}")

    except Exception as e:
        print(f"❌ 服务器启动失败: {e}")
        print("可能的原因:")
        print("1. 端口 5000 已被占用")
        print("2. 没有足够的权限绑定端口")
        print("3. 防火墙阻止了连接")
    finally:
        print("\n✅ GPS 坐标接收器已关闭")


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
        sys.exit(1)
