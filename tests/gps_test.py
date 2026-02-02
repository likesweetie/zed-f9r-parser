import spidev   
import time



BUS = 0
DEV = 0

READ_LEN = 256
POLL_HZ = 20
MAX_SPEED_HZ = 1_000_000

spi1 = spidev.SpiDev()
spi1.open(BUS, DEV)

spi1.max_speed_hz = MAX_SPEED_HZ
spi1.mode = 0b00
spi1.bits_per_word = 8

line = bytearray()
in_sentence = False

dummy = [0x00] * READ_LEN

period = 1.0 / POLL_HZ

while True:
    rx = spi1.xfer2(dummy)
    # print(rx)
    for b in rx:
        if b == 0xFF:
            continue
        # print(b)
        if not in_sentence:
            if b == 0x24: # this is '$'
                in_sentence = True
                line.clear()
                line.append(b)
            continue

        if len(line) < 255 :
            line. append(b)

        if b == 0x0A: #this is '\n'
            try:
                print(line.decode("ascii", errors="replace"), end="")
            finally:
                in_sentence = False
                line.clear()
    # print("poll")
    time.sleep(period)