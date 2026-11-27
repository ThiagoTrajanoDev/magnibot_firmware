"""Fontes de dados do gêmeo digital: a porta serial (ESP32 de verdade) ou um simulador."""
import json
import math
import time

from braco import NOMES, POSE_NEUTRA

# Nomes dos estados, na ordem de state_code usada pelo firmware (docs/serial-protocol.md).
ESTADOS = {
    0: "IDLE",
    1: "VARREDURA",
    2: "VARREDURA CONCLUÍDA",
    3: "DESLOCAMENTO",
    4: "ERRO",
}


def interpretar_linha(linha):
    """Lê uma linha de texto da serial. Devolve a telemetria (dicionário) ou None.

    O ESP32 mistura mensagens de boot e logs com os JSONs, então ignoramos
    tudo que não for um objeto JSON com type=telemetry e version=1.
    """
    linha = linha.strip()
    if not linha.startswith("{"):          # log comum do ESP-IDF, não é JSON
        return None
    try:
        dados = json.loads(linha)
    except ValueError:                     # JSON quebrado (linha cortada no meio)
        return None
    if not isinstance(dados, dict):
        return None
    if dados.get("type") != "telemetry" or dados.get("version") != 1:
        return None
    return dados


def extrair_angulos(telemetria):
    """Devolve a lista com os 6 ângulos, ou None se o firmware ainda não os envia.

    Hoje o firmware manda "angles_deg": null. Quando mandar valores, aceitamos
    uma lista na ordem [base, ombro, cotovelo, punho_rot, punho_flex, garra]
    ou um dicionário com esses nomes. (FORMATO A CONFIRMAR com quem faz o firmware.)
    """
    bruto = telemetria.get("angles_deg")
    if isinstance(bruto, (list, tuple)) and len(bruto) >= 6:
        valores = list(bruto[:6])
    elif isinstance(bruto, dict) and all(nome in bruto for nome in NOMES):
        valores = [bruto[nome] for nome in NOMES]
    else:
        return None
    if not all(isinstance(v, (int, float)) for v in valores):
        return None
    return [float(v) for v in valores]


class LeitorSerial:
    """Lê a telemetria da porta serial sem travar o desenho."""

    def __init__(self, porta_serial):
        self.porta = porta_serial   # objeto da biblioteca pyserial (ou um falso, nos testes)
        self.sobra = ""             # pedaço de linha que ainda não terminou

    def ler(self):
        """Devolve a lista de telemetrias que chegaram desde a última chamada."""
        chegou = self.porta.read(self.porta.in_waiting or 0)   # só o que já está disponível
        self.sobra += chegou.decode("utf-8", errors="ignore")
        *linhas, self.sobra = self.sobra.split("\n")           # a última parte pode estar incompleta
        resultado = []
        for linha in linhas:
            telemetria = interpretar_linha(linha)
            if telemetria is not None:
                resultado.append(telemetria)
        return resultado

    def fechar(self):
        self.porta.close()


def abrir_serial(nome_da_porta):
    """Abre a porta de verdade. A biblioteca só é carregada aqui, para o modo --simular não exigir pyserial."""
    import serial
    # timeout=0: ler() nunca espera; 115200 é a velocidade combinada no protocolo.
    return LeitorSerial(serial.Serial(nome_da_porta, 115200, timeout=0))


# ---------------------------------------------------------------------------
# Simulador: finge ser o ESP32 para a equipe desenvolver sem o braço.
# Cada etapa: (duração em s, state_code, ângulos iniciais, ângulos finais, distância inicial, final)
# ---------------------------------------------------------------------------
_N = POSE_NEUTRA
_ETAPAS = [
    (3, 0, _N,                         _N,                         None, None),
    (3, 1, _N,                         [-45, 90, 90, 90, 90, 90],  600, 600),
    (6, 1, [-45, 90, 90, 90, 90, 90],  [60, 90, 90, 90, 90, 90],   600, 250),
    (2, 2, [60, 90, 90, 90, 90, 90],   [60, 90, 90, 90, 90, 90],   250, 250),
    (3, 3, [60, 90, 90, 90, 90, 90],   [60, 55, 125, 90, 105, 150], 250, 120),
    (2, 3, [60, 55, 125, 90, 105, 150], [60, 55, 125, 90, 105, 60], 120, 120),
    (4, 3, [60, 55, 125, 90, 105, 60], [0, 90, 90, 90, 105, 60],   120, 300),
    (2, 3, [0, 90, 90, 90, 105, 60],   [0, 90, 90, 90, 105, 150],  300, 300),
    (3, 0, [0, 90, 90, 90, 105, 150],  _N,                         None, None),
]
DURACAO_CICLO = sum(etapa[0] for etapa in _ETAPAS)


def _misturar(a, b, fracao):
    """Valor no meio do caminho entre a e b (fracao 0 = a, 1 = b)."""
    return a + (b - a) * fracao


def telemetria_em(t):
    """Telemetria que o ESP32 mandaria t segundos após o começo (o ciclo se repete)."""
    t = t % DURACAO_CICLO
    for duracao, codigo, ang_ini, ang_fim, dist_ini, dist_fim in _ETAPAS:
        if t <= duracao:
            fracao = t / duracao
            angulos = [round(_misturar(a, b, fracao), 1) for a, b in zip(ang_ini, ang_fim)]
            distancia = None if dist_ini is None else round(_misturar(dist_ini, dist_fim, fracao))
            return {
                "type": "telemetry", "version": 1, "uptime_ms": int(t * 1000),
                "state": ESTADOS[codigo], "state_code": codigo, "mode": "simulador",
                "motion_enabled": codigo != 0,
                "angles_deg": angulos, "distance_mm": distancia,
                "current_ma": None, "gripper_percent": None,
            }
        t -= duracao
    return telemetria_em(0)


class Simulador:
    """Mesma interface do LeitorSerial, mas inventa os dados (20 mensagens por segundo)."""

    def __init__(self):
        self.inicio = time.time()
        self.ultimo = 0.0

    def ler(self):
        agora = time.time() - self.inicio
        if agora - self.ultimo < 0.05:      # ainda não passou 1/20 de segundo
            return []
        self.ultimo = agora
        return [telemetria_em(agora)]

    def fechar(self):
        pass
