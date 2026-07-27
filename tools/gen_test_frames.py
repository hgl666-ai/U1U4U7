def crc16(d):
    c=0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            if c & 1: c = (c >> 1) ^ 0xA001
            else: c >>= 1
    return c

def frame(cmd, seq, data_hex):
    data = bytes.fromhex(data_hex) if data_hex else b''
    length = len(data)
    hdr = bytes([0xA5, 0x5B,
                 (cmd >> 8) & 0xFF, cmd & 0xFF,
                 (seq >> 8) & 0xFF, seq & 0xFF,
                 (length >> 8) & 0xFF, length & 0xFF])
    payload = hdr + data
    crc = crc16(payload)
    full = payload + bytes([(crc >> 8) & 0xFF, crc & 0xFF])
    return full.hex(' ').upper()

n = 0
def seq():
    global n; n += 1; return n

print("=== [7] 正常DATA: 先发START进入UPLOADING, 再发256字节数据 ===")
print("STEP1: " + frame(0x0009, seq(), '0064 00004E20 010000'))
print("STEP2: " + frame(0x000A, seq(), '00' * 256))

print("\n=== [8] BUSY: 复位后直接发DATA ===")
print(frame(0x000A, 1, 'AA BB CC'))

print("\n=== [9] OVERFLOW ===")
s = seq()
print("STEP1-START: " + frame(0x0009, s, '000A 00000064 010000'))
print("STEP2-100B : " + frame(0x000A, seq(), 'A5' * 100))
print("STEP3-5B   : " + frame(0x000A, seq(), 'B6' * 5))

print("\n=== [11] 错误锁存 ===")
print(frame(0x000A, seq(), 'CC' * 10))

print("\n=== [12] 残页 ===")
s = seq()
print("STEP1-START: " + frame(0x0009, s, '000A 00000064 010000'))
print("STEP2-DATA : " + frame(0x000A, seq(), 'DE' * 100))
print("STEP3-END  : " + frame(0x000B, seq(), ''))

print("\n=== [13] 正常END ===")
print(frame(0x000B, seq(), ''))

print("\n=== [14] BUSY END ===")
print(frame(0x000B, 1, ''))

print("\n=== [15] ERR_COUNT ===")
s = seq()
print("STEP1-START: " + frame(0x0009, s, '0050 00004E20 010000'))
print("STEP2-DATA : " + frame(0x000A, seq(), 'AF' * 250))
print("STEP3-END  : " + frame(0x000B, seq(), ''))

print("\n=== [16] 正常QUERY ===")
print(frame(0x000C, seq(), ''))

print("\n=== [17] ERR_NO_FW ===")
print(frame(0x000C, 1, ''))

print("\n=== [18] ERR_INCOMPLETE ===")
s = seq()
print("STEP1-START: " + frame(0x0009, s, '0064 00004E20 010000'))
print("STEP2-DATA : " + frame(0x000A, seq(), 'AD' * 256))
print("STEP3-QUERY: " + frame(0x000C, seq(), ''))
