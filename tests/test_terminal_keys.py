"""PTY tests: all terminal reads occur on one worker, with fresh phase input."""
from contextlib import contextmanager
import io
import os
import pty
import threading
import unittest
from unittest.mock import patch
from terminal_keys import TerminalInput


class TerminalKeyTests(unittest.TestCase):
    @contextmanager
    def console(self):
        master,slave=pty.openpty()
        stream=os.fdopen(slave,'rb',buffering=0)
        reader=TerminalInput(stream,io.StringIO());reader.start()
        try:yield reader,master
        finally:
            reader.close();stream.close();os.close(master)
            self.assertFalse(reader.is_alive())

    def answer(self,reader,master,value):
        def send():
            if reader.prompt_ready.wait(2):os.write(master,value)
        sender=threading.Thread(target=send);sender.start()
        return sender

    def test_terminal_read_occurs_only_on_dedicated_thread(self):
        ids=[];real_read=os.read
        with self.console() as (reader,master):
            def observed_read(fd,size):
                if fd==reader.fd:ids.append(threading.get_ident())
                return real_read(fd,size)
            with patch('terminal_keys.os.read',side_effect=observed_read):
                sender=self.answer(reader,master,b'n\n')
                self.assertEqual(reader.ask('start? '),'n');sender.join(timeout=2)
            self.assertTrue(ids)
            self.assertEqual(set(ids),{reader.ident})
            self.assertNotEqual(reader.ident,threading.get_ident())

    def test_partial_stale_key_is_flushed_before_next_prompt(self):
        with self.console() as (reader,master):
            sender=self.answer(reader,master,b'n\n')
            self.assertEqual(reader.ask('start? '),'n');sender.join(timeout=2)
            os.write(master,b's')  # No Enter: still pending in terminal canonical buffer.
            sender=self.answer(reader,master,b'r\n')
            self.assertEqual(reader.ask('review? '),'r');sender.join(timeout=2)

    def test_extra_lines_cannot_accept_the_next_phase(self):
        with self.console() as (reader,master):
            sender=self.answer(reader,master,b'n\ns\n')
            self.assertEqual(reader.ask('start? '),'n');sender.join(timeout=2)
            sender=self.answer(reader,master,b'r\n')
            self.assertEqual(reader.ask('review? '),'r');sender.join(timeout=2)

    def test_arrow_escape_is_not_mixed_into_review_key(self):
        with self.console() as (reader,master):
            sender=self.answer(reader,master,b'\x1b[Cs\n')
            self.assertEqual(reader.ask('review? '),'s');sender.join(timeout=2)

    def test_shutdown_unblocks_pending_question(self):
        with self.console() as (reader,master):
            errors=[]
            def wait_for_input():
                try:reader.ask('start? ')
                except EOFError:errors.append('closed')
            waiter=threading.Thread(target=wait_for_input);waiter.start()
            self.assertTrue(reader.prompt_ready.wait(2))
            reader.close();waiter.join(timeout=2)
            self.assertEqual(errors,['closed'])
            self.assertFalse(waiter.is_alive())


if __name__=='__main__':unittest.main()
