#!/usr/bin/env python3

import socket
import time
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
    """Socket测试发送端主函数。"""
    # 注册信号处理函数
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Socket Test Sender')
    parser.add_argument('--target-ip', type=str, default='localhost', help='目标IP地址')
    parser.add_argument('--target-port', type=int, default=5000, help='目标端口')
    parser.add_argument('--test-message', type=str, default='Hello from PX4 Drone!', help='测试消息')
    parser.add_argument('--send-interval', type=float, default=2.0, help='发送间隔（秒）')
    parser.add_argument('--headless', action='store_true', help='启用无头模式')
    args = parser.parse_args()

    # 初始化变量
    message_count = 0
    
    # 输出初始化信息
    if not args.headless:
        print("Socket Test Sender 启动")
        print(f"参数配置: headless={args.headless}, send_interval={args.send_interval}秒")
        print(f"目标配置: {args.target_ip}:{args.target_port}")
        print(f"测试消息: {args.test_message}")
        print("按Ctrl+C或任何键退出...")

    try:
        while running:
            # 准备测试数据
            test_data = {
                'message': args.test_message,
                'count': message_count,
                'timestamp': time.time()
            }
            
            try:
                # 创建socket连接
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((args.target_ip, args.target_port))
                    # 发送测试数据
                    data = json.dumps(test_data).encode('utf-8')
                    s.sendall(data)
                    message_count += 1
                    if not args.headless:
                        print(f"✅ 成功发送测试消息 #{message_count}")
                        print(f"   目标: {args.target_ip}:{args.target_port}")
                        print(f"   消息: {test_data['message']}")
            except Exception as e:
                if not args.headless:
                    print(f"发送测试消息失败: {str(e)}")
                    print("请确保接收端已启动并在同一网络中")
            
            # 等待指定的发送间隔，同时检查是否需要退出
            wait_time = args.send_interval
            start_time = time.time()
            while time.time() - start_time < wait_time and running:
                time.sleep(0.1)  # 短时间睡眠，以便及时响应终止信号
                
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        if not args.headless:
            print(f"\nSocket Test Sender 关闭，共发送了 {message_count} 条消息")
        sys.exit(0)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
        sys.exit(1)