#!/usr/bin/env python3
"""Run bench protocol assertions against a connected ESP32-S3."""
import argparse
import hashlib
import json
import subprocess
import time
from pathlib import Path

import serial


def provenance(firmware, build_dir):
    """Identify exactly what is under test; an unresolvable field is recorded, never guessed."""
    def git(*cmd):
        try:
            return subprocess.run(('git', '-C', str(firmware)) + cmd, capture_output=True,
                                  text=True, check=True).stdout.strip()
        except (OSError, subprocess.CalledProcessError):
            return None

    def sha256(path):
        try:
            return hashlib.sha256(path.read_bytes()).hexdigest()
        except OSError:
            return None

    try:
        env = json.loads((build_dir / 'config.env').read_text())
    except (OSError, ValueError):
        env = {}
    status = git('status', '--porcelain')
    return {'commit': git('rev-parse', 'HEAD'),
            'tree_clean': None if status is None else status == '',
            'idf_version': env.get('IDF_VERSION'),
            'app_sha256': sha256(build_dir / 'magnibot_firmware.bin'),
            'elf_sha256': sha256(build_dir / 'magnibot_firmware.elf')}


def main():
    args = argparse.ArgumentParser()
    args.add_argument('--port', required=True)
    args.add_argument('--output-dir', required=True)
    args.add_argument('--milestone', choices=('m2', 'm3', 'm4a'), default='m2')
    args.add_argument('--build-dir', default='build/bench',
                      help='build of the flashed image, relative to the firmware checkout')
    options = args.parse_args()
    output = Path(options.output_dir).resolve()
    firmware = Path(__file__).resolve().parents[1]
    if output == firmware or firmware in output.parents:
        raise SystemExit('Store test evidence outside the firmware repository.')
    output.mkdir(parents=True, exist_ok=True)
    build = provenance(firmware, (firmware / options.build_dir).resolve())
    port = serial.Serial(port=None, baudrate=115200, timeout=0.1)
    port.dtr = False
    port.rts = False
    port.port = options.port
    port.open()
    frames, raw, checks = [], [], []

    def collect(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            line = port.readline().decode('utf-8', errors='replace')
            if not line:
                continue
            raw.append(line)
            try:
                frame = json.loads(line)
            except ValueError:
                continue
            frames.append(frame)

    def response(request, expected, ident, state=None):
        start = len(frames)
        port.write(request)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            collect(0.15)
            candidates = [f for f in frames[start:] if f.get('type') == 'response'
                          and f.get('id') == ident and f.get('code') == expected]
            if candidates:
                frame = candidates[-1]
                assert frame['version'] == 1 and frame['mode'] == 'bench'
                assert frame['motion_enabled'] is False
                assert frame['ok'] == (expected == 'ok')
                if state:
                    assert frame['state'] == state, frame
                checks.append({'id': ident, 'expected': expected, 'response': frame})
                return frame
        raise AssertionError(f'Missing response id={ident} code={expected}')

    try:
        collect(2)
        response(b'MB1 1 status\n', 'ok', 1, 'IDLE')
        initial_diagnostic = response(b'MB1 2 diagnostic\r\n', 'ok', 2, 'IDLE')
        if options.milestone in ('m3', 'm4a'):
            d = initial_diagnostic['diagnostic']
            assert d['backend'] == 'disabled' and d['axes_configured'] == 0
            assert d['sequence_phase'] == 'idle' and d['sequence_fault'] == 'none'
            assert d['distance_status'] == d['current_status'] == 'unavailable'
            if options.milestone == 'm4a':
                assert d['distance_io'] == d['current_io'] == 'disabled'
                assert d['actuator_io'] == 'blocked' and d['actuator_shutdown_confirmed'] is False
        for ident, command in enumerate(('start', 'confirm', 'rescan'), 3):
            response(f'MB1 {ident} {command}\n'.encode(), 'motion_blocked', ident, 'IDLE')
        port.write(b'MB1 6 sta')
        time.sleep(0.1)
        response(b'tus\n', 'ok', 6, 'IDLE')
        response(b'MB2 7 status\n', 'bad_version', None)
        response(b'MB1 4294967296 status\n', 'bad_id', None)
        response(b'MB1 8 invalid\n', 'unknown_command', 8)
        response(b'MB1 9 status extra\n', 'bad_frame', 9)
        response(b'\x00\n', 'bad_frame', None)
        response(b'x' * 120 + b'\n', 'too_long', None)
        response(b'MB1 10 status', 'timeout', None)
        response(b'\nMB1 11 status\n', 'ok', 11, 'IDLE')
        response(b'MB1 12 estop\n', 'ok', 12, 'ERROR')
        response(b'MB1 13 estop\n', 'ok', 13, 'ERROR')
        for ident, command in enumerate(('start', 'confirm', 'rescan'), 14):
            response(f'MB1 {ident} {command}\n'.encode(), 'emergency_latched', ident, 'ERROR')
        # Burst overload must not reset ERROR or compromise subsequent recovery.
        port.write(b'\n' * 80 + b'MB1 17 estop\n')
        collect(2)
        response(b'MB1 18 status\n', 'ok', 18, 'ERROR')
        diagnostic = response(b'MB1 19 diagnostic\n', 'ok', 19, 'ERROR')
        assert diagnostic['diagnostic']['protocol_errors'] > 0
        assert diagnostic['diagnostic']['dropped_responses'] > 0
        if options.milestone in ('m3', 'm4a'):
            d = diagnostic['diagnostic']
            assert d['backend'] == 'disabled' and d['axes_configured'] == 0
            assert d['sequence_phase'] == 'error' and d['sequence_fault'] == 'emergency'
            assert d['distance_status'] == d['current_status'] == 'unavailable'
            if options.milestone == 'm4a':
                assert d['distance_io'] == d['current_io'] == d['actuator_io'] == 'latched'
                assert d['actuator_shutdown_confirmed'] is True
        collect(2)
        telemetry = [f for f in frames if f.get('type') == 'telemetry']
        assert len(telemetry) >= 10
        assert all(f['motion_enabled'] is False for f in telemetry)
        assert all(f[k] is None for f in telemetry for k in
                   ('angles_deg', 'distance_mm', 'current_ma', 'gripper_percent'))
        assert telemetry[-1]['state'] == 'ERROR'
        assert all(t['stack_min_free_bytes'] > 0 for t in telemetry)
        if options.milestone in ('m3', 'm4a'):
            assert all(t['execution']['backend'] == 'disabled' and
                       t['execution']['axes_configured'] == 0 for t in telemetry)
            assert telemetry[-1]['execution']['phase'] == 'error'
            assert telemetry[-1]['execution']['fault'] == 'emergency'
        periods = [b['uptime_ms'] - a['uptime_ms']
                   for a, b in zip(telemetry, telemetry[1:])]
        assert periods and all(450 <= ms <= 750 for ms in periods), periods
        assert not any(t in ''.join(raw) for t in ('Guru Meditation', 'Brownout', 'abort()', 'Falha de inicializacao'))
        result = {'milestone': options.milestone, 'status': 'PASS', 'build': build, 'checks': checks, 'telemetry_frames': len(telemetry),
                  'final_state': telemetry[-1]['state'], 'diagnostic': diagnostic['diagnostic'],
                  'telemetry_period_ms_min': min(periods),
                  'telemetry_period_ms_max': max(periods)}
        print(json.dumps(result, ensure_ascii=False))
        (output / f'{options.milestone}-result.json').write_text(json.dumps(result, indent=2) + '\n')
    finally:
        port.close()
        (output / f'{options.milestone}-serial.txt').write_text('\n'.join(line.rstrip() for line in ''.join(raw).splitlines()) + '\n')


if __name__ == '__main__':
    main()
