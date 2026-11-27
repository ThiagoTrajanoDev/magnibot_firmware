"""Calibração: converte os graus do servo (a unidade do firmware) nos ângulos do modelo 3D.

"Graus do servo" = (pulso - 500) * 180 / 2000. É a unidade de "ombro 127" no terminal do
firmware e de "angles_deg" na telemetria (bench_servo_named_angle, em firmware/main/bench_servo.c).
Os comandos digitados no gêmeo usam essa mesma unidade.

O modelo 3D usa outra convenção (braco.py): 90 = a peça na posição em que o STL foi montado.
A conversão de cada junta é:   modelo = 90 + sinal * (graus_do_servo - neutro) + offset
"""
from braco import NOMES, POSE_NEUTRA


def _graus(pulso_us):
    """Pulso do servo (us) -> graus do servo, como o firmware calcula."""
    return (pulso_us - 500) * 180 / 2000


# neutro: referência em graus do servo que coincide com a pose do STL quando offset é zero.
#   Valores iniciais = pulso neutro calibrado no firmware (joint_pwm, em bench_servo.c), supondo que
#   o neutro do firmware é a mesma pose do STL. CONFERIR NO BRAÇO e ajustar aqui se não for.
# sinal: +1 se aumentar o grau do servo mexe a peça no mesmo sentido do modelo; -1 se é ao contrário.
#   CONFERIR NO BRAÇO com "ombro 140", "cotovelo 60" etc. e comparar com o gêmeo.
# offset: correção em graus do modelo, aplicada depois do sinal. Ex.: +3.0 soma 3° ao desenho.
CALIBRACAO = {
    "ombro":      {"neutro": _graus(1916), "sinal": +1, "offset": -20.0},
    "cotovelo":   {"neutro": _graus(939),  "sinal": -1, "offset": 0.0},
    "punho_rot":  {"neutro": _graus(1205), "sinal": +1, "offset": 0.0},
    "punho_flex": {"neutro": _graus(1400), "sinal": +1, "offset": 0.0},
    "garra":      {"neutro": 90.0,        "sinal": +1, "offset": 0.0},
}

# Base: servo de giro contínuo, sem encoder. O firmware só sabe por quanto tempo girou
# ("base_drive_ms", positivo = pulso 1573). Para virar graus é preciso medir a velocidade:
# gire a base por um tempo conhecido e meça o ângulo. None = sem medida (a base fica em 0).
BASE_GRAUS_POR_S = None
BASE_SINAL = +1        # -1 se o pulso 1573 gira a base no sentido contrário ao do modelo

# Pose IDLE do firmware (idle_pose, em bench_servo.c), em graus do servo. A base fica em 0.
POSE_IDLE_SERVO = [0.0, 145.0, 55.0, 63.0, 81.0, 45.0]


def servo_para_modelo(angulos):
    """[base, ombro, ...] em graus do servo -> ângulos do modelo 3D. A base já vem em graus."""
    modelo = [float(angulos[0])]
    for i, nome in enumerate(NOMES[1:], start=1):
        c = CALIBRACAO[nome]
        modelo.append(POSE_NEUTRA[i] + c["sinal"] * (angulos[i] - c["neutro"]) + c["offset"])
    return modelo


def modelo_para_servo(angulos):
    """Inverso de servo_para_modelo."""
    servo = [float(angulos[0])]
    for i, nome in enumerate(NOMES[1:], start=1):
        c = CALIBRACAO[nome]
        servo.append(c["neutro"] + (angulos[i] - POSE_NEUTRA[i] - c["offset"]) / c["sinal"])
    return servo


# Pose do STL, em graus do servo (o comando "neutro" do gêmeo).
POSE_NEUTRA_SERVO = modelo_para_servo(POSE_NEUTRA)


def base_em_graus(drive_ms):
    """Tempo de giro da base (ms, com sinal) -> graus, ou None se a base não foi calibrada."""
    if BASE_GRAUS_POR_S is None or not isinstance(drive_ms, (int, float)):
        return None
    return BASE_SINAL * BASE_GRAUS_POR_S * drive_ms / 1000
