#!/usr/bin/env python3

import socket
import json
import argparse
import signal
import sys

# 全局变量，用于控制程序运行
running = True


def signal_handler(sig, frame):
    """信号处理函数，用于捕获Ctrl+C"""
    global running
    running = False
    print("\n接收到终止信号，正在退出...")


def main():
    """Socket测试接收端主函数。"""
    # 注册信号处理函数
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Socket Test Receiver')
    parser.add_argument('--listen-ip', type=str, default='0.0.0.0', help='监听IP地址（0.0.0.0表示监听所有接口）')
    parser.add_argument('--listen-port', type=int, default=5000, help='监听端口')
    parser.add_argument('--buffer-size', type=int, default=1024, help='缓冲区大小')
    parser.add_argument('--headless', action='store_true', help='启用无头模式')
    args = parser.parse_args()

    # 初始化变量
    received_count = 0
    connection_flag = 0  # 连接标志位，0表示未连接，1表示已连接
    
    # 输出初始化信息
    if not args.headless:
        print("Socket Test Receiver 启动")
        print(f"参数配置: headless={args.headless}")
        print(f"监听配置: {args.listen_ip}:{args.listen_port}")
        print(f"连接标志位初始值: {connection_flag}")
        print("正在等待测试消息...")
        print("按Ctrl+C或任何键退出...")

    try:
        # 创建socket服务器
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((args.listen_ip, args.listen_port))
            s.listen()
            if not args.headless:
                print(f"Socket服务器已启动，监听 {args.listen_ip}:{args.listen_port}")
            
            # 设置socket超时，以便及时响应终止信号
            s.settimeout(1.0)
            
            # 持续接收连接
            while running:
                try:
                    # 等待连接
                    conn, addr = s.accept()
                    with conn:
                        if not args.headless:
                            print(f"接收到来自 {addr} 的连接")
                        
                        # 设置连接标志位为1
                        connection_flag = 1
                        
                        # 接收数据
                        data = conn.recv(args.buffer_size)
                        if data:
                            try:
                                # 解析测试数据
                                test_data = json.loads(data.decode('utf-8'))
                                received_count += 1
                                
                                if not args.headless:
                                    print("✅ 成功接收测试消息")
                                    print(f"   连接标志位: {connection_flag}")
                                    print(f"   消息计数: #{test_data['count']}")
                                    print(f"   消息内容: {test_data['message']}")
                                    print(f"   时间戳: {test_data['timestamp']}")
                                    print(f"   总接收数: {received_count}")
                            except json.JSONDecodeError as e:
                                if not args.headless:
                                    print(f"解析测试数据失败: {str(e)}")
                        else:
                            if not args.headless:
                                print("接收到空数据")
                except socket.timeout:
                    # 超时是正常的，只是为了检查是否需要退出
                    continue
                except Exception as e:
                    if not args.headless:
                        print(f"接收连接时发生错误: {e}")
                    # 继续运行
                    continue
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        if not args.headless:
            print(f"\nSocket Test Receiver 关闭，共接收了 {received_count} 条消息")
        sys.exit(0)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
        sys.exit(1)