from smbus2 import SMBus, i2c_msg
from nmea_decoder import NMEADecoder  # nmea_decoder.py에 NMEADecoder가 있다고 가정
import time

I2C_BUS = 1
CAND_ADDR = 0x42
MAX_CHUNK = 32

def read_available(bus, addr) -> int:
    # 0xFD부터 2바이트 읽기 (0xFD=LSB, 0xFE=MSB인 경우가 보통)
    w = i2c_msg.write(addr, [0xFD])
    r = i2c_msg.read(addr, 2)
    bus.i2c_rdwr(w, r)

    # ⚠️ 여기 바이트 순서는 환경에 따라 뒤집혀 보일 수 있습니다.
    msb, lsb = list(r)
    return (msb << 8) | lsb

def is_gga_sentence(line: str) -> bool:
    # $GNGGA, $GPGGA, $GLGGA 등 talker는 달라도 msg_type이 GGA면 true
    line = line.strip()
    if not line.startswith("$"):
        return False
    body = line[1:].split("*", 1)[0]
    head = body.split(",", 1)[0]
    return len(head) >= 5 and head[2:5] == "GGA"

def on_epoch_end(frame):
    """
    epoch 종료 시점에 하고 싶은 처리를 여기에 넣으세요.
    예: frame.GGA[0].lat_deg 출력, 로그 저장 등
    """
    pass

def main():
    decoder = NMEADecoder(verify_checksum=False)  # checksum 켜고 싶으면 True
    decoder.begin_epoch(epoch_id=1)

    line_buf = bytearray()

    with SMBus(I2C_BUS) as bus:
        while True:
            try:
                avail = read_available(bus, CAND_ADDR)
            except OSError as e:
                print(CAND_ADDR, f": failed {e}")
                time.sleep(0.5)
                continue

            if avail <= 0:
                time.sleep(0.01)
                continue

            n = min(avail, MAX_CHUNK)
            data = bytes(bus.read_i2c_block_data(CAND_ADDR, 0xFF, n))

            # DDC에서 데이터 없을 때 0xFF 패딩이 섞일 수 있어 제거(필요 시)
            data = bytes(b for b in data if b != 0xFF)
            if not data:
                time.sleep(0.01)
                continue

            # 스트림을 라인(\n) 단위로 조립
            for b in data:
                line_buf.append(b)
                if b == 0x0A:  # '\n'
                    line = line_buf.decode("ascii", errors="replace")
                    line_buf.clear()

                    # (원하면) 원문 출력
                    print(line, end="", flush=True)

                    # --- epoch 규칙: GGA가 나오면 새 epoch 시작 ---
                    if is_gga_sentence(line):
                        prev = decoder.end_epoch()
                        if prev is not None:
                            on_epoch_end(prev)
                            print("GGA:", prev.GGA)
                        decoder.begin_epoch()  # epoch_id 자동 증가(구현에 따라)
                    # -------------------------------------------
                    decoder.feed_line(line)

            time.sleep(0.01)

if __name__ == "__main__":
    main()
