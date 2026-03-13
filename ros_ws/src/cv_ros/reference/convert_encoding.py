#!/usr/bin/env python3
"""
文件编码转换脚本
将GBK编码的C文件转换为UTF-8编码，修复中文注释乱码问题
"""

import os
import sys
import chardet

def detect_file_encoding(file_path):
    """检测文件编码"""
    with open(file_path, 'rb') as f:
        raw_data = f.read()
        result = chardet.detect(raw_data)
        return result['encoding']

def convert_file_encoding(file_path, source_encoding=None, target_encoding='utf-8'):
    """转换文件编码"""
    try:
        if source_encoding is None:
            source_encoding = detect_file_encoding(file_path)
            print(f"检测到文件编码: {source_encoding}")
        
        # 读取原始文件
        with open(file_path, 'r', encoding=source_encoding, errors='ignore') as f:
            content = f.read()
        
        # 备份原始文件
        backup_path = file_path + '.backup'
        with open(backup_path, 'r', encoding=source_encoding, errors='ignore') as f:
            backup_content = f.read()
        
        with open(backup_path, 'w', encoding=source_encoding) as f:
            f.write(backup_content)
        print(f"已创建备份文件: {backup_path}")
        
        # 写入转换后的内容
        with open(file_path, 'w', encoding=target_encoding) as f:
            f.write(content)
        
        print(f"文件编码转换完成: {source_encoding} -> {target_encoding}")
        return True
        
    except UnicodeDecodeError:
        print(f"错误: 无法使用 {source_encoding} 编码读取文件")
        return False
    except Exception as e:
        print(f"转换过程中发生错误: {e}")
        return False

def main():
    if len(sys.argv) < 2:
        print("使用方法: python convert_encoding.py <文件路径> [源编码]")
        print("示例: python convert_encoding.py Ano_OPMV_CBTracking_Ctrl.c")
        print("示例: python convert_encoding.py Ano_OPMV_CBTracking_Ctrl.c gbk")
        sys.exit(1)
    
    file_path = sys.argv[1]
    source_encoding = sys.argv[2] if len(sys.argv) > 2 else None
    
    if not os.path.exists(file_path):
        print(f"错误: 文件不存在 - {file_path}")
        sys.exit(1)
    
    print(f"开始转换文件: {file_path}")
    
    if convert_file_encoding(file_path, source_encoding):
        print("转换成功！")
    else:
        print("转换失败！")
        sys.exit(1)

if __name__ == '__main__':
    main()
