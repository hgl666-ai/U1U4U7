def modbus_crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def frame(cmd, seq, data_hex):
    data = bytes.fromhex(data_hex) if data_hex else b''
    length = len(data)
    hdr = bytes([0xA5, 0x5B,
                 (cmd >> 8) & 0xFF, cmd & 0xFF,
                 (seq >> 8) & 0xFF, seq & 0xFF,
                 (length >> 8) & 0xFF, length & 0xFF])
    payload = hdr + data
    crc = modbus_crc16(payload)
    full = payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    return full.hex(' ').upper()

n = 0
def seq():
    global n; n += 1; return n

print("=" * 70)
print("PC->U1 固件下载边缘测试 - 完整 HEX 帧")
print("波特率 460800 8N1, 十六进制发送")
print("=" * 70)

# ==== FW_START (0x0009) ====
print("\n=== 一、FW_START (0x0009) ===")

print("\n[1] 正常: 100帧, 20000字节, v1.0.0")
print("    " + frame(0x0009, seq(), '0064 00004E20 010000'))

print("\n[2] BUSY: 连发两次 (第2次应返回 0x01)")
s = seq()
print("    第1次: " + frame(0x0009, s, '0064 00004E20 010000'))
print("    第2次: " + frame(0x0009, seq(), '0064 00004E20 010000'))

print("\n[3] ERR_SIZE: LEN=6 (旧格式缺版本号)")
print("    " + frame(0x0009, seq(), '0064 00004E20'))

print("\n[4] ERR_SIZE: 文件大小=0")
print("    " + frame(0x0009, seq(), '0064 00000000 010000'))

print("\n[5] ERR_SIZE: 大小=65536 (>65528)")
print("    " + frame(0x0009, seq(), '0064 00010000 010000'))

print("\n[6] ERR_SIZE: 总帧数=0")
print("    " + frame(0x0009, seq(), '0000 00004E20 010000'))

# ==== FW_DATA (0x000A) ====
print("\n=== 二、FW_DATA (0x000A) ===")

print("\n[7] 正常: 256字节数据 (先发[1]进入UPLOADING)")
print("    " + frame(0x000A, seq(), '00' * 256))

print("\n[8] BUSY: 复位后直接发DATA (不走START)")
print("    " + frame(0x000A, seq(), 'AA BB CC'))

print("\n[9] ERR_OVERFLOW: START声明100字节, 实际发105")
s9 = seq()
print("    STEP1-START: " + frame(0x0009, s9, '000A 00000064 010000'))
print("    STEP2-DATA1: " + frame(0x000A, seq(), 'A5' * 100))
print("    STEP3-DATA2: " + frame(0x000A, seq(), 'B6' * 5))
print("    -> STEP3 ACK 应为 0x0D")

print("\n[10] ERR_TIMEOUT: 发START后等>10秒")
print("    " + frame(0x0009, seq(), '0064 00004E20 010000'))
print("    -> 等10s, U1 自发 ACK 0x0C")

print("\n[11] 错误锁存: 接[9]STEP3后再发DATA, ACK应维持0x0D")
print("    " + frame(0x000A, seq(), 'CC' * 10))

print("\n[12] 残页写入: 100字节(非256整数)后END, ACK=0x00")
s12 = seq()
print("    STEP1-START: " + frame(0x0009, s12, '000A 00000064 010000'))
print("    STEP2-DATA : " + frame(0x000A, seq(), 'DE' * 100))
print("    STEP3-END  : " + frame(0x000B, seq(), ''))

# ==== FW_END (0x000B) ====
print("\n=== 三、FW_END (0x000B) ===")

print("\n[13] 正常结束: 接[7]后发END")
print("    " + frame(0x000B, seq(), ''))

print("\n[14] BUSY: 复位后直接发END")
print("    " + frame(0x000B, seq(), ''))

print("\n[15] ERR_COUNT: 声明20000字节, 发250字节后END")
s15 = seq()
print("    STEP1-START: " + frame(0x0009, s15, '0050 00004E20 010000'))
print("    STEP2-DATA : " + frame(0x000A, seq(), 'AF' * 250))
print("    STEP3-END  : " + frame(0x000B, seq(), ''))

# ==== QUERY_FW_VER (0x000C) ====
print("\n=== 四、QUERY_FW_VER (0x000C) ===")

print("\n[16] 正常查询: 完整上传后 (先做[12]全流程)")
print("    " + frame(0x000C, seq(), ''))

print("\n[17] ERR_NO_FW: 复位后直接查询")
print("    " + frame(0x000C, seq(), ''))

print("\n[18] ERR_INCOMPLETE: START+DATA后直接查(不发END)")
s18 = seq()
print("    STEP1-START: " + frame(0x0009, s18, '0064 00004E20 010000'))
print("    STEP2-DATA : " + frame(0x000A, seq(), 'AD' * 256))
print("    STEP3-QUERY: " + frame(0x000C, seq(), ''))

print("\n" + "=" * 70)
print("操作:")
print("  多步用例按 STEP1->STEP2->STEP3 顺序发")
print("  每次测试前建议复位U1恢复IDLE")
print("  测试[9][11] 连续执行: 先发完[9]三帧, 再发[11]")
print("=" * 70)
