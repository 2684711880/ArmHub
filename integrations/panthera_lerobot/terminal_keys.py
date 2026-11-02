"""One terminal reader thread; phase-scoped queues never send robot commands."""
import atexit
import os
import queue
import re
import select
import sys
import threading


class TerminalInput(threading.Thread):
    def __init__(self, stream=None, output=None):
        super().__init__(name='lerobot-terminal-keys',daemon=True)
        self.stream=sys.stdin if stream is None else stream
        self.output=sys.stdout if output is None else output
        self.fd=self.stream.fileno()
        self.requests=queue.Queue(maxsize=1)
        self.responses=queue.Queue(maxsize=8)
        self.stop_event=threading.Event();self.eof=threading.Event()
        self.prompt_ready=threading.Event()
        self.lock=threading.Lock();self.epoch=0;self.waiting=False

    def _flush_pending(self):
        if os.isatty(self.fd):
            import termios
            termios.tcflush(self.fd,termios.TCIFLUSH)
        else:
            while select.select([self.fd],[],[],0)[0]:
                if not os.read(self.fd,4096):
                    self.eof.set();break

    def run(self):
        active=None;buffer=b''
        try:
            while not self.stop_event.is_set() and not self.eof.is_set():
                try:
                    token,prompt=self.requests.get_nowait()
                except queue.Empty:
                    pass
                else:
                    self._flush_pending();buffer=b''
                    active=token
                    print(prompt,end='',file=self.output,flush=True)
                    self.prompt_ready.set()
                if not select.select([self.fd],[],[],.05)[0]:
                    continue
                chunk=os.read(self.fd,4096)
                if not chunk:
                    self.eof.set();break
                if active is None:
                    continue  # Discard keys typed during recording or preparation.
                buffer+=chunk
                if b'\n' not in buffer:
                    continue
                line=buffer.split(b'\n',1)[0].decode(errors='replace')
                line=re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]','',line)
                with self.lock:
                    valid=self.waiting and self.epoch==active
                token=active
                active=None;buffer=b'';self.prompt_ready.clear()
                if valid:
                    self.responses.put_nowait((token,line))
        except (OSError,ValueError):
            self.eof.set()
        finally:
            self.prompt_ready.clear()

    def ask(self, prompt):
        with self.lock:
            if self.waiting:
                raise RuntimeError('终端已有一个输入阶段，不能并发提问')
            if self.stop_event.is_set() or self.eof.is_set():
                raise EOFError('终端输入已关闭')
            self.waiting=True;self.epoch+=1;token=self.epoch
        try:
            self.requests.put_nowait((token,prompt))
            while not self.stop_event.is_set():
                try:
                    received,line=self.responses.get(timeout=.05)
                    if received==token:return line
                except queue.Empty:
                    if self.eof.is_set():raise EOFError('终端输入已结束')
            raise EOFError('终端输入已关闭')
        finally:
            with self.lock:
                self.waiting=False

    def close(self):
        self.stop_event.set()
        if self.is_alive() and threading.current_thread() is not self:
            self.join(timeout=.3)


_console=None


def console_input(prompt):
    global _console
    if _console is None:
        _console=TerminalInput();_console.start()
    return _console.ask(prompt)


def close_console():
    global _console
    if _console is not None:
        _console.close();_console=None


atexit.register(close_console)
