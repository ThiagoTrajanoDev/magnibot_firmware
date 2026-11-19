"""Geometria do braço: transforma os ângulos dos servos em pontos 3D para desenhar."""
import math

import numpy as np

# ---------------------------------------------------------------------------
# MEDIDAS DO BRAÇO, em milímetros.
# ATENÇÃO: são valores PROVISÓRIOS. Troque pelas medidas do modelo 3D (GrabCAD).
# ---------------------------------------------------------------------------
ALTURA_BASE = 60       # do chão até a articulação do ombro
COMPR_BRACO = 120      # ombro -> cotovelo
COMPR_ANTEBRACO = 120  # cotovelo -> punho
COMPR_PUNHO = 50       # punho -> ponta onde ficam os dedos da garra
COMPR_DEDO = 40        # comprimento de cada dedo da garra
ABERTURA_MAX = 40      # meia abertura da garra quando o servo está em 180 graus

# Ordem dos 6 ângulos (a mesma ordem da tabela de pinagem do projeto).
NOMES = ["base", "ombro", "cotovelo", "punho_rot", "punho_flex", "garra"]

# Posição "neutra": base no zero, os servos de 180 graus em 90 (o neutro da tabela).
POSE_NEUTRA = [0.0, 90.0, 90.0, 90.0, 90.0, 90.0]


def _direcao(phi_graus):
    """Direção de um pedaço do braço no plano vertical: (alcance, altura).

    phi = 0 aponta para cima; phi positivo inclina o pedaço para a frente.
    """
    phi = math.radians(phi_graus)
    return math.sin(phi), math.cos(phi)


def calcular_pose(angulos):
    """Recebe os 6 ângulos e devolve os pontos que o desenho precisa.

    Retorna um dicionário:
      "corpo": 5 pontos 3D (chão, ombro, cotovelo, punho, ponta do punho)
      "dedos": 2 segmentos 3D (um por dedo da garra)
      "barra": segmento que liga a base dos dois dedos
    """
    base, ombro, cotovelo, punho_rot, punho_flex, garra = angulos

    # --- Passo 1: o braço visto "de lado", num plano vertical (alcance, altura) ---
    # Cada articulação soma sua inclinação à do pedaço anterior,
    # como no seu próprio braço: mexer o ombro leva o antebraço junto.
    phi1 = ombro - 90                # inclinação do braço (90 = em pé)
    phi2 = phi1 + (cotovelo - 90)    # inclinação do antebraço
    phi3 = phi2 + (punho_flex - 90)  # inclinação do punho

    alcance, altura = 0.0, 0.0
    pontos_rz = [(alcance, altura)]              # chão
    altura = ALTURA_BASE
    pontos_rz.append((alcance, altura))          # ombro
    for phi, comprimento in ((phi1, COMPR_BRACO), (phi2, COMPR_ANTEBRACO), (phi3, COMPR_PUNHO)):
        dr, dz = _direcao(phi)
        alcance += comprimento * dr
        altura += comprimento * dz
        pontos_rz.append((alcance, altura))      # cotovelo, punho, ponta

    # --- Passo 2: a base gira o plano inteiro em volta do eixo vertical ---
    psi = math.radians(base)
    corpo = np.array([[r * math.cos(psi), r * math.sin(psi), z] for r, z in pontos_rz])

    # --- Passo 3: os dedos da garra ---
    dr, dz = _direcao(phi3)
    eixo = np.array([dr * math.cos(psi), dr * math.sin(psi), dz])       # direção do punho
    lado = np.array([-math.sin(psi), math.cos(psi), 0.0])               # "para o lado" do plano
    cima = np.array([math.cos(math.radians(phi3)) * math.cos(psi),      # perpendicular ao punho,
                     math.cos(math.radians(phi3)) * math.sin(psi),      # dentro do plano
                     -math.sin(math.radians(phi3))])
    giro = math.radians(punho_rot - 90)                                  # rotação do punho
    lado = math.cos(giro) * lado + math.sin(giro) * cima

    abertura = max(0.0, min(garra, 180.0)) / 180.0 * ABERTURA_MAX
    ponta = corpo[-1]
    dedo_a = (ponta + lado * abertura, ponta + lado * abertura + eixo * COMPR_DEDO)
    dedo_b = (ponta - lado * abertura, ponta - lado * abertura + eixo * COMPR_DEDO)
    barra = (dedo_a[0], dedo_b[0])

    return {"corpo": corpo, "dedos": [dedo_a, dedo_b], "barra": barra}


# ---------------------------------------------------------------------------
# Quadros de referência: onde fica cada peça e para que lado ela aponta.
# O modelo 3D (modelo.py) usa isto para posicionar cada peça do braço.
# ---------------------------------------------------------------------------
def _gira_z(graus):
    """Matriz que gira em volta do eixo vertical (z)."""
    a = math.radians(graus)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


def _gira_y(graus):
    """Matriz que inclina em volta do eixo y (para frente/trás)."""
    a = math.radians(graus)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def calcular_quadros(angulos):
    """Devolve, para cada peça, a rotação (3x3) e a posição (3) da sua articulação.

    Em cada peça, o eixo z local aponta ao longo da peça (do ombro para o cotovelo, etc.).
    """
    base, ombro, cotovelo, punho_rot, punho_flex, _garra = angulos
    phi1 = ombro - 90
    phi2 = phi1 + (cotovelo - 90)
    phi3 = phi2 + (punho_flex - 90)

    r_cintura = _gira_z(base)
    r_braco = r_cintura @ _gira_y(phi1)
    r_antebraco = r_cintura @ _gira_y(phi2)
    r_punho = r_cintura @ _gira_y(phi3) @ _gira_z(-(punho_rot - 90))

    ombro_pos = np.array([0.0, 0.0, ALTURA_BASE])
    cotovelo_pos = ombro_pos + r_braco @ np.array([0.0, 0.0, COMPR_BRACO])
    punho_pos = cotovelo_pos + r_antebraco @ np.array([0.0, 0.0, COMPR_ANTEBRACO])

    return {
        "base": (np.eye(3), np.zeros(3)),
        "cintura": (r_cintura, np.zeros(3)),
        "braco": (r_braco, ombro_pos),
        "antebraco": (r_antebraco, cotovelo_pos),
        "punho": (r_punho, punho_pos),
    }
