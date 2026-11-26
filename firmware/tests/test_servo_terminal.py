import importlib.util
import io
import json
import os
from pathlib import Path
import pty
import select
import tempfile
import unittest
from unittest.mock import patch

MODULE = Path(__file__).resolve().parents[1] / 'tools' / 'servo_terminal.py'
spec = importlib.util.spec_from_file_location('servo_terminal', MODULE)
terminal = importlib.util.module_from_spec(spec)
spec.loader.exec_module(terminal)


class FakePort:
    def __init__(self, data=b'', open_error=None):
        self.data = data
        self.open_error = open_error
        self.io_error = None
        self.closed = False
        self.writes = []
        self.partial = False

    def open(self):
        if self.open_error:
            raise self.open_error
        assert self.dtr is False and self.rts is False

    def close(self):
        self.closed = True

    @property
    def in_waiting(self):
        if self.io_error:
            raise self.io_error
        return len(self.data)

    def read(self, count):
        result, self.data = self.data[:count], self.data[count:]
        return result

    def write(self, data):
        self.writes.append(data)
        if self.io_error:
            raise self.io_error
        return 1 if self.partial else len(data)


class ReconnectTests(unittest.TestCase):
    def test_missing_at_start_retries_without_writing(self):
        missing = FakePort(open_error=terminal.serial.SerialException('missing'))
        present = FakePort(b'{"type":"telemetry"}\n')
        with patch.object(terminal.serial, 'Serial', side_effect=[missing, present]) as factory, \
                patch.object(terminal.time, 'monotonic', return_value=0) as clock:
            link = terminal.SerialLink('/dev/test')
            self.assertEqual(link.poll(), [])
            self.assertFalse(link.connected)
            self.assertFalse(link.send('varredura'))
            self.assertEqual(link.poll(), [])
            self.assertEqual(factory.call_count, 1)
            clock.return_value = 1
            self.assertEqual(link.poll(), [b'{"type":"telemetry"}'])
            self.assertTrue(link.connected)
            self.assertEqual(present.writes, [])
            link.close()

    def test_read_disconnect_clears_partial_frame_and_reconnects(self):
        first, second = FakePort(b'{"old":'), FakePort(b'{"new":1}\n')
        with patch.object(terminal.serial, 'Serial', side_effect=[first, second]), \
                patch.object(terminal.time, 'monotonic', return_value=0) as clock:
            link = terminal.SerialLink('/dev/test')
            self.assertEqual(link.poll(), [])
            first.io_error = OSError('USB reset')
            old_generation = link.generation
            self.assertEqual(link.poll(), [])
            self.assertFalse(link.connected)
            self.assertTrue(first.closed)
            self.assertNotEqual(old_generation, link.generation)
            self.assertEqual(link.pending, b'')
            clock.return_value = 1
            self.assertEqual(link.poll(), [b'{"new":1}'])
            self.assertEqual(second.writes, [])
            link.close()

    def test_failed_or_partial_writes_are_never_replayed(self):
        for error in (OSError('unplug'), terminal.serial.SerialTimeoutException('timeout'), None):
            with self.subTest(error=error):
                first, second = FakePort(), FakePort()
                with patch.object(terminal.serial, 'Serial', side_effect=[first, second]), \
                        patch.object(terminal.time, 'monotonic', return_value=0) as clock:
                    link = terminal.SerialLink('/dev/test')
                    link.poll()
                    first.io_error = error
                    first.partial = error is None
                    self.assertFalse(link.send('varredura'))
                    self.assertFalse(link.connected)
                    clock.return_value = 1
                    link.poll()
                    self.assertEqual(first.writes, [b'varredura\n'])
                    self.assertEqual(second.writes, [])
                    link.close()

    def test_fragmented_lines_and_bounded_memory(self):
        port = FakePort(b'{"type":')
        with patch.object(terminal.serial, 'Serial', return_value=port):
            link = terminal.SerialLink('/dev/test')
            self.assertEqual(link.poll(), [])
            port.data = b'"telemetry"}\nignored\n'
            self.assertEqual(link.poll(), [b'{"type":"telemetry"}', b'ignored'])
            port.data = b'x' * 25000
            for _ in range(10):
                link.poll()
                self.assertLessEqual(len(link.pending), 16384)
            link.close()

    def test_curses_survives_disconnect_and_clears_old_measurement(self):
        port = FakePort(json.dumps({'type':'telemetry','distance_active':True,'distance_mm':123}).encode()+b'\n')
        frames = []

        class Screen:
            def timeout(self, _): pass
            def erase(self): self.lines = []
            def getmaxyx(self): return (30, 160)
            def addnstr(self, row, col, line, width): self.lines.append(line)
            def refresh(self): frames.append(self.lines)
            def getch(self):
                if len(frames) == 1:
                    port.io_error = OSError('USB reset')
                    return -1
                return 4

        with patch.object(terminal.serial, 'Serial', return_value=port), \
                patch.object(terminal.curses, 'wrapper', side_effect=lambda run: run(Screen())):
            link = terminal.SerialLink('/dev/test')
            terminal.distance_console(link, io.StringIO())
            self.assertIn('Distância: 123 mm', frames[0])
            self.assertNotIn('Distância: 123 mm', frames[1])
            self.assertTrue(any('USB desconectado' in line for line in frames[1]))
            self.assertEqual(port.writes, [])
            link.close()

    def test_os_pty_disconnect_and_reconnect(self):
        """Exercise the real pyserial/POSIX driver, without connecting to the ESP."""
        master, slave = pty.openpty()
        new_master = new_slave = None
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'monitor'
            path.symlink_to(os.ttyname(slave))
            link = terminal.SerialLink(str(path))
            try:
                with patch.object(terminal.time, 'monotonic', return_value=0) as clock:
                    link.poll()
                    self.assertTrue(link.connected)
                    os.write(master, b'{"boot":1}\n')
                    self.assertTrue(select.select([link.port.fileno()], [], [], 1)[0])
                    self.assertEqual(link.poll(), [b'{"boot":1}'])
                    os.close(master)
                    master = None
                    link.poll()
                    self.assertFalse(link.connected)
                    new_master, new_slave = pty.openpty()
                    path.unlink()
                    path.symlink_to(os.ttyname(new_slave))
                    clock.return_value = 1
                    link.poll()
                    self.assertTrue(link.connected)
                    os.write(new_master, b'{"boot":2}\n')
                    self.assertTrue(select.select([link.port.fileno()], [], [], 1)[0])
                    self.assertEqual(link.poll(), [b'{"boot":2}'])
            finally:
                link.close()
                for descriptor in (master, slave, new_master, new_slave):
                    if descriptor is not None:
                        os.close(descriptor)


if __name__ == '__main__':
    unittest.main()
