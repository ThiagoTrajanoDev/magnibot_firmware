"""Modelo 3D simplificado do MagniBot, montado com caixas e cilindros.

As peças imitam o desenho do GrabCAD (base cinza com o logo, braços vermelhos,
garra cinza com servo azul), mas com medidas aproximadas tiradas das fotos.
Cada peça é guardada no "quadro" da sua articulação e só é girada/movida a cada
atualização (ver calcular_quadros em braco.py).
"""
import math

import numpy as np

from braco import ABERTURA_MAX, COMPR_ANTEBRACO, COMPR_BRACO, COMPR_DEDO, COMPR_PUNHO, calcular_quadros

VERMELHO = "#d42424"
CINZA = "#b8c2cc"
CINZA_BASE = "#9eacba"
AZUL = "#2f5bd8"

LUZ = np.array([0.4, -0.5, 0.8])
LUZ = LUZ / np.linalg.norm(LUZ)


# ----------------------------------------------------------------- formas básicas
def caixa(x0, x1, y0, y1, z0, z1):
    """Caixa entre os limites dados. Devolve (vértices, faces)."""
    v = np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)], dtype=float)
    # índice do vértice = 4*ix + 2*iy + iz
    faces = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1], [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]]
    return v, faces


def cilindro(raio, altura, eixo="z", centro=(0.0, 0.0, 0.0), lados=18):
    """Cilindro centrado em 'centro', com o comprimento ao longo do eixo pedido (z ou y)."""
    ang = np.linspace(0, 2 * math.pi, lados, endpoint=False)
    baixo = [[raio * math.cos(a), raio * math.sin(a), -altura / 2] for a in ang]
    cima = [[raio * math.cos(a), raio * math.sin(a), altura / 2] for a in ang]
    v = np.array(baixo + cima, dtype=float)
    if eixo == "y":                                   # deita o cilindro: z vira y
        v = v @ np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=float).T
    v = v + np.array(centro, dtype=float)
    faces = [[i, (i + 1) % lados, lados + (i + 1) % lados, lados + i] for i in range(lados)]
    faces.append(list(range(lados)))                  # tampa de baixo
    faces.append(list(range(lados, 2 * lados)))       # tampa de cima
    return v, faces


class ModeloBraco:
    def __init__(self):
        # Cada peça: (nome do quadro, vértices, faces, cor)
        self.pecas = []

        def add(quadro, forma, cor):
            self.pecas.append((quadro, forma[0], forma[1], cor))

        # --- base fixa (cinza), com a aba de fixação no chão
        add("base", cilindro(62, 4, centro=(0, 0, 2), lados=28), CINZA)
        add("base", cilindro(50, 42, centro=(0, 0, 25), lados=28), CINZA_BASE)

        # --- cintura (gira com a base): prato vermelho e o suporte do ombro
        add("cintura", cilindro(50, 7, centro=(0, 0, 49.5), lados=28), VERMELHO)
        add("cintura", caixa(-22, 22, -26, 26, 53, 74), VERMELHO)

        # --- braço (ombro -> cotovelo): barra + disco em cada ponta
        add("braco", caixa(-11, 11, -13, 13, 0, COMPR_BRACO), VERMELHO)
        add("braco", cilindro(19, 34, eixo="y", centro=(0, 0, 0)), VERMELHO)
        add("braco", cilindro(17, 34, eixo="y", centro=(0, 0, COMPR_BRACO)), VERMELHO)

        # --- antebraço (cotovelo -> punho)
        add("antebraco", caixa(-10, 10, -12, 12, 0, COMPR_ANTEBRACO), VERMELHO)
        add("antebraco", cilindro(16, 30, eixo="y", centro=(0, 0, COMPR_ANTEBRACO)), VERMELHO)

        # --- punho: bloco vermelho, placa cinza da garra e servo azul
        add("punho", caixa(-13, 13, -13, 13, -8, COMPR_PUNHO - 14), VERMELHO)
        add("punho", caixa(-6, 6, -25, 25, COMPR_PUNHO - 14, COMPR_PUNHO), CINZA)
        add("punho", caixa(8, 22, -10, 10, 10, COMPR_PUNHO - 6), AZUL)

    def _dedos(self, garra):
        """Os dois dedos da garra: se afastam quando o servo da garra aumenta."""
        abertura = max(0.0, min(garra, 180.0)) / 180.0 * ABERTURA_MAX
        dedos = []
        for lado in (1, -1):
            y_a, y_b = sorted((lado * abertura, lado * (abertura + 8)))
            forma = caixa(-5, 5, y_a, y_b, COMPR_PUNHO, COMPR_PUNHO + COMPR_DEDO)
            dedos.append(("punho", forma[0], forma[1], CINZA))
        return dedos

    def poligonos(self, angulos):
        """Devolve (lista de polígonos 3D, lista de cores) já posicionados e com sombreamento."""
        quadros = calcular_quadros(angulos)
        polis, cores = [], []
        for quadro, verts, faces, cor in self.pecas + self._dedos(angulos[5]):
            rot, pos = quadros[quadro]
            v = verts @ rot.T + pos                      # leva a peça para o lugar certo
            centro = v.mean(axis=0)
            rgb = np.array([int(cor[i:i + 2], 16) for i in (1, 3, 5)]) / 255.0
            for f in faces:
                p = v[f]
                normal = np.cross(p[1] - p[0], p[2] - p[0])
                tamanho = np.linalg.norm(normal)
                if tamanho == 0:
                    continue
                normal = normal / tamanho
                if np.dot(normal, p.mean(axis=0) - centro) < 0:   # garante que aponta para fora
                    normal = -normal
                brilho = 0.55 + 0.45 * max(0.0, float(np.dot(normal, LUZ)))
                polis.append(p)
                cores.append((*np.clip(rgb * brilho, 0, 1), 1.0))
        return polis, cores
