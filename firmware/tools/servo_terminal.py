#!/usr/bin/env python3
import argparse
import curses
import json
import readline
import time

import serial


class SerialLink:
    """Reconnect the monitor without resetting the board or replaying writes."""

    def __init__(self, path):
        self.path = path
        self.port = None
        self.retry_at = 0
        self.generation = 0
        self.pending = b""
        self.status = "Aguardando USB"

    @property
    def connected(self):
        return self.port is not None

    def close(self):
        if self.port is not None:
            try:
                self.port.close()
            except (serial.SerialException, OSError):
                pass
        self.port = None
        self.pending = b""

    def disconnect(self, error):
        self.close()
        self.generation += 1
        self.retry_at = time.monotonic() + 1
        self.status = f"USB desconectado; reconectando em {self.path}: {error}"

    def poll(self):
        if not self.connected:
            if time.monotonic() < self.retry_at:
                return []
            port = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=1, exclusive=True)
            port.dtr = port.rts = False
            port.port = self.path
            try:
                port.open()
            except (serial.SerialException, OSError) as error:
                port.close()
                self.retry_at = time.monotonic() + 1
                self.status = f"Aguardando USB em {self.path}: {error}"
                return []
            self.port = port
            self.generation += 1
            self.status = f"Conectado em {self.path}"
        try:
            data = self.port.read(min(self.port.in_waiting, 4096))
        except (serial.SerialException, OSError) as error:
            self.disconnect(error)
            return []
        lines = (self.pending + data).split(b"\n")
        self.pending = lines.pop()
        if len(self.pending) > 16384:
            self.pending = b""
        return lines

    def send(self, command):
        if not self.connected:
            return False
        data = (command + "\n").encode()
        try:
            if self.port.write(data) != len(data):
                raise serial.SerialException("envio incompleto")
        except (serial.SerialException, OSError) as error:
            self.disconnect(error)
            return False
        return True


def distance_console(link, log=None):
    def run(screen):
        screen.timeout(50)
        command = ""
        response = ""
        telemetry = {}
        search_labels = {
            "idle": "aguardando", "positioning": "posicionando braço", "searching": "procurando",
            "found": "objeto encontrado", "returning_home": "alvo salvo, retornando à origem",
            "awaiting_confirm": "aguardando dois cliques / MB1 1 confirm",
            "returning_origin": "retornando à origem", "returning_object": "retornando ao alvo", "at_object": "no alvo, pega pendente", "no_object": "sem alvo; na origem, aguarda nova varredura",
            "sensor_fault": "falha do sensor", "fault": "falha", "stopped": "parada",
        }
        pickup_labels = {
            "disabled": "desabilitada", "ready": "aguardando alvo", "wait_sensor": "aguardando leitura nova",
            "approach": "aproximando até 60 mm", "verify": "confirmando distância",
            "close": "fechando garra em 0°", "lift": "elevando ombro em 20°",
            "home": "retornando base à origem", "open": "abrindo garra em 70°",
            "done": "concluída", "failed": "interrompida; exige reset"
        }
        generation = link.generation
        last_telemetry = 0
        while True:
            messages = link.poll()
            if generation != link.generation:
                generation = link.generation
                telemetry = {}
                response = "Conexão alterada; comandos anteriores não serão repetidos"
            if time.monotonic() - last_telemetry > 3:
                telemetry = {}
            for raw in messages:
                if log:
                    log.write(raw.decode(errors="replace") + "\n")
                    log.flush()
                try:
                    message = json.loads(raw)
                except (ValueError, UnicodeError):
                    continue
                if not isinstance(message, dict):
                    continue
                if message.get("type") == "telemetry":
                    telemetry = message
                    last_telemetry = time.monotonic()
                elif message.get("type") == "response":
                    response = message.get("code", "")
                    if response == "sensor_unavailable":
                        response = "sensor indisponível; varredura bloqueada antes do movimento"
            active = telemetry.get("distance_active", False)
            mm = telemetry.get("distance_mm")
            if active and mm is not None:
                distance = f"{mm:.0f} mm"
            elif active:
                distance = "Sem leitura válida"
            else:
                distance = "Medição suspensa ou aguardando telemetria"
            lines = [
                "MagniBot — distância",
                f"USB: {link.status}",
                f"Distância: {distance}",
                f"Sensor: {telemetry.get('distance_status', 'aguardando telemetria')}",
                f"Estado: {telemetry.get('state', 'aguardando')}",
                f"Busca (até 155 mm): {search_labels.get(telemetry.get('search_status'), 'aguardando')}",
                f"Pega: {pickup_labels.get(telemetry.get('pickup_status'), 'aguardando')} / {telemetry.get('pickup_fault', 'none')}",
                (f"Alvo salvo: {telemetry.get('target_distance_mm')} mm; referência temporal "
                 f"{telemetry.get('target_offset_ms')} ms" if telemetry.get("target_valid") else "Sem alvo salvo"),
                "",
                f"magnibot> {command}",
                f"Resposta: {response}",
                "",
                "varredura inicia. MB1 1 confirm confirma. MB1 2 rescan repete.",
                "Ajustes manuais e IDLE disponíveis apenas em IDLE.",
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
                sent = link.send(line)
                command = ""
                response = "aguardando" if sent else "Envio não confirmado; comando descartado, sem repetição"
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

    link = SerialLink(args.port)
    try:
        if args.distance:
            distance_console(link, args.log)
        else:
            command_console(link)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()
        if args.log:
            args.log.close()


def command_console(link):
    print("MagniBot. Digite nome ângulo ou um comando MB1. Ctrl+D sai.")
    print("varredura: punho_flex 140 -> cotovelo 65 -> ombro 118; reajuste final do punho 140 e garra 180.")
    print("IDLE: ombro 145, cotovelo 55, punho_rot 63, punho_flex 81, garra 45.")
    while True:
        link.poll()
        if not link.connected:
            print(link.status)
            time.sleep(1)
            continue
        try:
            line = input("magnibot> ").strip()
        except EOFError:
            return
        if not line:
            continue
        link.poll()
        if not link.send(line):
            print("Envio não confirmado; comando descartado, sem repetição.")
            continue
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            for raw in link.poll():
                try:
                    message = json.loads(raw)
                except (ValueError, UnicodeError):
                    continue
                if isinstance(message, dict) and message.get("type") == "response":
                    print(json.dumps(message, ensure_ascii=False))
                    if message.get("code") == "pose_not_ready":
                        print("Aguarde o movimento atual terminar e envie preparar novamente.")
                    deadline = 0
                    break
            time.sleep(0.02)


if __name__ == "__main__":
    main()
