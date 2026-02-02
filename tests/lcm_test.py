import spidev   
import time

import lcm
from gps_lcm_type import gps_t



lcm_node = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")




msg = gps_t()

msg.timestamp = 0
msg.alt = 0
msg.lat = 0


while True:
        
    time.sleep(0.5)
    lcm_node.publish("GPS_DATA", msg.encode())
    print("published")