#!/usr/bin/env python3
import argparse
import curses
import json
import readline
import time

import serial


def distance_console(port, log=None):
    def run(screen):
        screen.timeout(50)
        command = ""
        response = ""
        telemetry = {}
        while True:
            while port.in_waiting:
                try:
                    raw = port.readline()
                    if log:
                        log.write(raw.decode(errors="replace"))
                        log.flush()
                    message = json.loads(raw)
                except (ValueError, UnicodeError):
                    continue
                if not isinstance(message, dict):
                    continue
                if message.get("type") == "telemetry":
                    telemetry = message
                elif message.get("type") == "response":
                    response = message.get("code", "")
            active = telemetry.get("distance_active", False)
            mm = telemetry.get("distance_mm")
            if active and mm is not None:
                distance = f"{mm:.0f} mm"
            elif active:
                distance = "Sem leitura válida"
            else:
                distance = "Aguardando conclusão do preparo"
            lines = [
                "MagniBot — distância",
                "",
                f"Distância: {distance}",
                f"Sensor: {telemetry.get('distance_status', 'aguardando telemetria')}",
                f"Estado: {telemetry.get('state', 'aguardando')}",
                "",
                f"magnibot> {command}",
                f"Resposta: {response}",
                "",
                "Digite varredura para posicionar e varrer. IDLE retorna à pose inicial.",
                "Ctrl+D sai. Emergência física no GPIO 6.",
            ]
            screen.erase()
            height, width = screen.getmaxyx()
            for row, line in enumerate(lines[:max(0, height - 1)]):
                screen.addnstr(row, 0, line, max(0, width - 1))
            screen.refresh()
            key = screen.getch()
            if key == 4:
                return
            if key in (10, 13, curses.KEY_ENTER) and command.strip():
                line = command.strip()
                port.write((line + "\n").encode())
                if line in ("varredura", "preparar", "IDLE"):
                    telemetry["distance_active"] = False
                    telemetry["distance_mm"] = None
                command = ""
                response = "aguardando"
            elif key in (8, 127, curses.KEY_BACKSPACE):
                command = command[:-1]
            elif 32 <= key <= 126 and len(command) < 96:
                command += chr(key)
    curses.wrapper(run)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("--distance", action="store_true")
    parser.add_argument("--log", type=argparse.FileType("a"))
    args = parser.parse_args()

    with serial.Serial(port=None, baudrate=115200, timeout=0.1, write_timeout=1, exclusive=True) as port:
        port.dtr = port.rts = False
        port.port = args.port
        port.open()
        time.sleep(1)
        if args.distance:
            distance_console(port, args.log)
            return
        print("MagniBot. Digite nome ângulo ou um comando MB1. Ctrl+D sai.")
        print("varredura: punho_flex 140 -> cotovelo 50 -> ombro 100; reajuste final do punho 140 e garra 180.")
        print("IDLE: ombro 127, cotovelo 40, punho_rot 63, punho_flex 81, garra 45.")
        while True:
            try:
                line = input("magnibot> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return
            if not line:
                continue
            port.reset_input_buffer()
            port.write((line + "\n").encode())
            deadline = time.monotonic() + 0.8
            while time.monotonic() < deadline:
                try:
                    message = json.loads(port.readline())
                except (ValueError, UnicodeError):
                    continue
                if isinstance(message, dict) and message.get("type") == "response":
                    print(json.dumps(message, ensure_ascii=False))
                    if message.get("code") == "pose_not_ready":
                        print("Aguarde o movimento atual terminar e envie preparar novamente.")
                    break


if __name__ == "__main__":
    main()
