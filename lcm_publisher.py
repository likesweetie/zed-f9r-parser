 
import time

import lcm
from gps_lcm_type import gps_t


class LCMPublisher:
    def __init__(self, multicast_addr, channel) -> None:

        self.lcm_node = lcm.LCM(multicast_addr)
        self.time = time.time()
        self.channel = channel

    def publish_gps(self, msg):
        self.lcm_node.publish(self.channel, msg.encode())
