#!/usr/bin/env python3
"""只停止本工程环境启动的 armhub_sim launch，不按 gzserver 名称批量结束进程。"""
import os
from pathlib import Path
import signal
import time

ROOT = Path(__file__).resolve().parents[1]
EXPECTED_PREFIX = str(ROOT / 'ros_ws/install/armhub_sim')


def main():
    matches = []
    for directory in Path('/proc').iterdir():
        if not directory.name.isdigit():
            continue
        try:
            args = directory.joinpath('cmdline').read_bytes().decode().split('\0')
            wanted = ['launch', 'armhub_sim', 'sim.launch.py']
            if not any(args[index:index+3] == wanted for index in range(len(args)-2)):
                continue
            environment = directory.joinpath('environ').read_bytes().decode().split('\0')
            prefix = next((item.partition('=')[2] for item in environment if item.startswith('AMENT_PREFIX_PATH=')), '')
            if EXPECTED_PREFIX not in prefix.split(':'):
                continue
            stat = directory.joinpath('stat').read_text().rsplit(')', 1)[1].split()
            matches.append((int(directory.name), stat[19]))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    if not matches:
        print('本工程没有正在运行的仿真。')
        return
    if len(matches) != 1:
        raise RuntimeError(f'发现多个本工程实例 {matches}，请在各自启动终端退出')
    pid, start_ticks = matches[0]
    print(f'向 ArmHub launch PID {pid} 发送 SIGINT，等待整套仿真退出……', flush=True)
    os.kill(pid, signal.SIGINT)
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline:
        try:
            stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
            if stat[0] == 'Z' or stat[19] != start_ticks:
                break
        except FileNotFoundError:
            break
        time.sleep(0.1)
    else:
        raise RuntimeError('launch 尚未退出，请查看启动日志；未强杀其他进程')
    print('ArmHub 仿真 launch 已退出。')


if __name__ == '__main__':
    main()
