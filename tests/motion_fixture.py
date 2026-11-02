"""只替换调度时钟，真实 Ruckig 和执行循环仍参与测试。"""
import time
from unittest.mock import patch


class VirtualClock:
    def __init__(self):
        self.now = time.monotonic()
        self.time = time.time

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(0, seconds)


def virtual_time(test):
    clock = VirtualClock()
    patcher = patch('armhub.adapters.panthera_ht.time', clock)
    patcher.start()
    test.addCleanup(patcher.stop)
    return clock
