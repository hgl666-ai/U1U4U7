#!/usr/bin/env python3
"""
固件合并工具
功能：将APP1.bin、control.bin、LVGL_ui.bin加密后合并为一个固件文件
如果某个固件文件不存在，则不包含该固件
"""
#python encrypt_firmware.py  使用这个命令合并原始固件
#python encrypt_firmware.py --stm32-version 1.0.1 --app1-version 1.0.3 --control-version 1.0.33 --lvgl-version 1.0.2
#python encrypt_firmware.py --stm32-version 1.0.1 --lvgl-version 1.0.1 
import os       # 导入os模块，用于文件操作
import struct   # 导入struct模块，用于处理二进制数据
import hashlib # 导入hashlib模块，用于计算校验和
import argparse # 导入argparse模块，用于处理命令行参数
from Crypto.Cipher import AES # 导入AES模块，用于加密数据

# 加密参数（与encrypt_firmware.py保持一致）
AES_KEY = bytes([
    0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
    0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C,
    0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
    0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
])

AES_IV = bytes([
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
])

# 固件类型定义
FIRMWARE_TYPES = [
    ("STM32G031G8Ux", "STM32G031G8Ux.bin"),
    ("APP1", "APP1.bin"),
    ("control", "control.bin"),
    ("LVGL_ui", "LVGL_ui.bin")
]

# 魔术字（用于识别合并固件格式）
FIRMWARE_MAGIC = 0x4649524D  # "FIRM"

def encrypt_firmware_data(plaintext):
    """加密固件数据（与encrypt_firmware.py使用相同的算法）"""
    # 添加零填充
    if len(plaintext) % 16 != 0:
        padding_length = 16 - (len(plaintext) % 16)
        plaintext = plaintext + bytes([0] * padding_length)
    
    # 加密
    cipher = AES.new(AES_KEY, AES.MODE_CBC, AES_IV)
    ciphertext = cipher.encrypt(plaintext)
    
    return ciphertext

def calculate_checksum(data):
    """计算数据的校验和（使用SHA256）"""
    return hashlib.sha256(data).digest()

def create_combined_firmware(output_path, component_versions):
    """创建合并固件文件"""
    # 收集存在的固件文件
    firmware_data = []
    total_data_size = 0
    
    print("检测固件文件...")
    for firmware_type, firmware_file in FIRMWARE_TYPES:
        if os.path.exists(firmware_file):
            file_size = os.path.getsize(firmware_file)
            # 获取组件版本号，如果没有指定则使用文件名
            version = component_versions.get(firmware_type, os.path.basename(firmware_file))
            firmware_data.append({
                "type": firmware_type,
                "file": firmware_file,
                "size": file_size,
                "version": version,
                "data": None  # 稍后读取并加密
            })
            total_data_size += file_size
            print(f"  找到 {firmware_file} ({file_size} bytes)，版本: {version}")
        else:
            print(f"  未找到 {firmware_file} (将跳过)")
    
    if not firmware_data:
        print("错误：没有找到任何固件文件！")
        return False
    
    # 读取并加密固件数据
    print("\n读取并加密固件数据...")
    for firmware in firmware_data:
        with open(firmware["file"], "rb") as f:
            raw_data = f.read()
            # 加密固件数据
            firmware["data"] = encrypt_firmware_data(raw_data)
        print(f"  加密 {firmware['file']} (原始: {firmware['size']} bytes, 加密后: {len(firmware['data'])} bytes)")
    
    # 构建主头部（移除主版本号字段）
    main_header = struct.pack(
        "<I I 64s",  # 小端序：魔术字(4) + 组件数(4) + 预留(64)
        FIRMWARE_MAGIC,
        len(firmware_data),
        b'\x00' * 64  # 预留
    )
    
    # 构建固件头部（暂时使用0作为偏移）
    component_headers = b''
    for firmware in firmware_data:
        component_header = struct.pack(
            "<16s 32s I I 12s",  # 小端序：类型(16) + 版本(32) + 大小(4) + 偏移(4) + 预留(12)
            firmware["type"].encode('utf-8').ljust(16, b'\x00'),
            firmware["version"].encode('utf-8').ljust(32, b'\x00'),
            len(firmware["data"]),  # 使用加密后的大小
            0,  # 暂时设置为0，后面会更新
            b'\x00' * 12  # 预留
        )
        component_headers += component_header
    
    # 计算实际的总头部大小
    total_header_size = len(main_header) + len(component_headers)
    print(f"\n头部大小信息：")
    print(f"  主头部大小: {len(main_header)} bytes")
    print(f"  组件头部大小: {len(component_headers)} bytes")
    print(f"  总头部大小: {total_header_size} bytes")
    
    # 计算各固件数据的偏移地址
    current_offset = total_header_size
    for firmware in firmware_data:
        firmware["offset"] = current_offset
        current_offset += len(firmware["data"])
    
    # 更新组件头部中的偏移地址
    component_headers = b''
    for firmware in firmware_data:
        component_header = struct.pack(
            "<16s 32s I I 12s",  # 小端序：类型(16) + 版本(32) + 大小(4) + 偏移(4) + 预留(12)
            firmware["type"].encode('utf-8').ljust(16, b'\x00'),
            firmware["version"].encode('utf-8').ljust(32, b'\x00'),
            len(firmware["data"]),  # 使用加密后的大小
            firmware["offset"],
            b'\x00' * 12  # 预留
        )
        component_headers += component_header
    
    print(f"\n固件偏移信息：")
    for firmware in firmware_data:
        print(f"  {firmware['type']}: 偏移={firmware['offset']}, 大小={len(firmware['data'])} bytes")
    
    # 构建固件数据
    firmware_data_bytes = b''
    for firmware in firmware_data:
        firmware_data_bytes += firmware["data"]
    
    # 计算校验和
    all_data = main_header + component_headers + firmware_data_bytes
    checksum = calculate_checksum(all_data)
    
    # 写入合并固件文件
    with open(output_path, "wb") as f:
        f.write(main_header)
        f.write(component_headers)
        f.write(firmware_data_bytes)
        f.write(checksum)  # 将校验和写入文件末尾
    
    print(f"\n合并加密固件创建成功！")
    print(f"输出文件：{os.path.abspath(output_path)}")
    print(f"总大小：{os.path.getsize(output_path)} bytes")
    print(f"包含固件：{[fw['type'] for fw in firmware_data]}")
    
    return True

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="固件合并工具")
    parser.add_argument("-o", "--output", default="combined_firmware.bin",
                      help="输出合并固件文件名")
    parser.add_argument("--stm32-version", default=None,
                      help="STM32G031G8Ux.bin固件版本号")
    parser.add_argument("--app1-version", default=None,
                      help="APP1.bin固件版本号")
    parser.add_argument("--control-version", default=None,
                      help="control.bin固件版本号")
    parser.add_argument("--lvgl-version", default=None,
                      help="LVGL_ui.bin固件版本号")
    
    args = parser.parse_args()
    
    # 构建组件版本号字典
    component_versions = {}
    if args.stm32_version:
        component_versions["STM32G031G8Ux"] = args.stm32_version
    if args.app1_version:
        component_versions["APP1"] = args.app1_version
    if args.control_version:
        component_versions["control"] = args.control_version
    if args.lvgl_version:
        component_versions["LVGL_ui"] = args.lvgl_version
    
    print("=== 固件合并工具 ===")
    print(f"输出文件: {args.output}")
    if component_versions:
        print("组件版本号:")
        for comp_type, version in component_versions.items():
            print(f"  {comp_type}: {version}")
    print()
    
    create_combined_firmware(args.output, component_versions)

if __name__ == "__main__":
    main()
