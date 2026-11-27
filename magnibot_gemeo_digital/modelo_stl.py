"""Modelo 3D do MagniBot usando os STLs reais.

Mesma interface do modelo procedural anterior:
    ModeloBraco().poligonos(angulos) -> (lista de triângulos 3D, lista de cores RGBA)

Cada peça é guardada no "quadro" da sua articulação e só é girada/movida a cada
atualização (ver calcular_quadros em braco.py).

O desenho é montado UMA vez no __init__ (pose neutra = POSE_NEUTRA do braco.py).
A cada quadro, as juntas ativas rodam os vértices da peça em torno dos pivôs
correspondentes.
"""

from pathlib import Path

import numpy as np
import trimesh

from braco import POSE_NEUTRA

PASTA_STL = Path(__file__).parent / "magnibot_model_stl"
ESCALA_STL = 1000          # STLs em metros -> milímetros
MAX_FACES = 600            # reduz o número de triângulos por peça (performance no matplotlib)


# ---------------------------------------------------------------------------
# Peças na pose neutra.
# O último campo é a lista de juntas que afetam a peça (da raiz para a folha).
# ---------------------------------------------------------------------------
PECAS = [
    # nome                       arquivo                    posicao          rotacao             cor                     juntas
    ("Base",                    "Base_r1",                  (0, 0, 0),       (0, 0, 0),          [110, 110, 110, 255],   []),
    ("Waist",                   "Waist_r1",                 (0, 57, 0),      (0, 0, 0),          [180, 180, 180, 255],   ["base"]),
    ("Arm 01",                  "Arm 01_r1",                (9, 97, -24),    (0, 90, 0),         [200, 60, 60, 255],     ["base", "ombro"]),
    ("Arm 02",                  "Arm 02 v3_r1",             (-11, 252, -19), (0, -90, 0),        [60, 100, 200, 255],    ["base", "ombro", "cotovelo"]),
    ("Arm 03",                  "Arm 03_r1",                (-11, 308, -13), (0, -90, 0),        [60, 100, 200, 255],    ["base", "ombro", "cotovelo", "punho_rot"]),
    ("Gripper Base",            "Gripper base_r1",          (-25, 370, 25),  (-90, 180, 0),      [60, 255, 200, 255],    ["base", "ombro", "cotovelo", "punho_rot", "punho_flex"]),
    ("Gear 1",                  "gear1_r1",                 (19, 382, 5),    (-90, 180, -125),   [200, 100, 200, 255],   ["base", "ombro", "cotovelo", "punho_rot", "punho_flex"]),
    ("Gear 2",                  "gear2_r1",                 (4, 382, 5),     (-90, 180, 119),    [200, 100, 200, 255],   ["base", "ombro", "cotovelo", "punho_rot", "punho_flex"]),
    ("Gripper Direito",         "Gripper 1_r1",             (23, 418, 32),   (90, 90, 22),       [150, 120, 200, 255],   ["base", "ombro", "cotovelo", "punho_rot", "punho_flex"]),
    ("Grip Link Direito",       "grip link 1_r1",           (0, 405, 4),     (90, 0, 55),        [100, 120, 200, 255],   ["base", "ombro", "cotovelo", "punho_rot", "punho_flex"]),
    ("Grip Link Direito Baixo", "grip link 1_r1",           (0, 405, -9),    (90, 0, 55),        [100, 120, 200, 255],   ["base", "ombro", "cotovelo", "punho_rot", "punho_flex"]),
    ("Gripper Esquerdo",        "Gripper 1_r1",             (-28, 418, 32),  (90, -90, 336),     [150, 120, 200, 255],   ["base", "ombro", "cotovelo", "punho_rot", "punho_flex"]),
    ("Grip Link Esquerdo",      "grip link 1_r1",           (-28, 405, 4),   (90, 0, 305),       [100, 120, 200, 255],   ["base", "ombro", "cotovelo", "punho_rot", "punho_flex"]),
    ("Grip Link Esquerdo Baixo","grip link 1_r1",           (-28, 405, -9),  (90, 0, 305),       [100, 120, 200, 255],   ["base", "ombro", "cotovelo", "punho_rot", "punho_flex"]),
    ("Servo MG996R",            "Servo Motor MG996R_r1",    (-9, 5, 0),      (0, 180, 0),        [60, 100, 200, 255],    []),
]


# ---------------------------------------------------------------------------
# Pivôs, eixos e sinais das juntas (espaço STL, Y = cima, mm).
# Se algum movimento ficar torto, ajuste o pivô/eixo/sinal daquela junta e teste com --simular.
# ---------------------------------------------------------------------------
JUNTAS = {
    "base":       {"pivo": (0.0,   0.0, 0.0), "eixo": (0, 1, 0), "sinal": +1},  # yaw vertical
    "ombro":      {"pivo": (0.0,  110.0, 0.0), "eixo": (1, 0, 0), "sinal": -1},  # inclinação para frente
    "cotovelo":   {"pivo": (100.0, 230.0, 0.0), "eixo": (1, 0, 0), "sinal": -1},
    "punho_flex": {"pivo": (0.0, 370.0, 0.0), "eixo": (1, 0, 0), "sinal": -1},
    "punho_rot":  {"pivo": (0.0, 308.0, 0.0), "eixo": (0, 1, 0), "sinal": -1},  # gira em volta do braço
}
ORDEM = ["base", "ombro", "cotovelo", "punho_flex", "punho_rot"]

# Gripper abre/fecha deslizando em X_stl (rotacionado pelas juntas ativas).
GARRA_PECAS = {
    "Gripper Direito":  +1,
    "Gripper Esquerdo": -1,
}
ABERTURA_POR_GRAU = 50.0 / 180.0   # 50 mm de curso total

GIRO_Z = -90 # rotacao do modelo em stl para conformidade com matplotlib

# Ajustar ao simular
GEAR_ROTACAO = {"Gear 1": -1.0, "Gear 2": +1.0}   # graus dos gears por grau de garra

def _matriz_mundo():
    """Y-up (STL) -> Z-up (matplotlib), com giro extra em Z."""
    Rx = trimesh.transformations.rotation_matrix(np.radians(90), [1, 0, 0])
    Rz = trimesh.transformations.rotation_matrix(np.radians(GIRO_Z), [0, 0, 1])
    return Rz @ Rx

def _localizar(nome):
    achados = list(PASTA_STL.rglob(f"{nome}.stl"))
    if not achados:
        raise FileNotFoundError(f"STL não encontrado: {nome}.stl")
    return achados[0]


def _rotacionar(vertices, pivo, eixo, angulo_graus):
    if abs(angulo_graus) < 1e-9:
        return vertices
    R = trimesh.transformations.rotation_matrix(
        np.radians(angulo_graus), np.asarray(eixo, dtype=float), point=pivo
    )
    hom = np.hstack([vertices, np.ones((len(vertices), 1))])
    return (R @ hom.T).T[:, :3]


def _rotacionar_ponto(ponto, pivo, eixo, angulo_graus):
    return _rotacionar(np.asarray(ponto, dtype=float).reshape(1, 3),
                       pivo, eixo, angulo_graus)[0]


def _rotacionar_vetor(vetor, eixo, angulo_graus):
    if abs(angulo_graus) < 1e-9:
        return vetor
    R = trimesh.transformations.rotation_matrix(
        np.radians(angulo_graus), np.asarray(eixo, dtype=float)
    )
    return R[:3, :3] @ vetor


class ModeloBraco:
    def __init__(self):
        self._cache = {}          
        self._pecas = []          # (nome, vertices, faces, cor, juntas)

        for nome, arquivo, pos, rot, cor, juntas in PECAS:
            if arquivo not in self._cache:
                mesh = trimesh.load(_localizar(arquivo), force="mesh", process=True)
                mesh.apply_scale(ESCALA_STL)
                if len(mesh.faces) > MAX_FACES:
                    mesh = mesh.simplify_quadric_decimation(face_count=MAX_FACES)
                self._cache[arquivo] = (mesh.vertices.copy(), mesh.faces.copy())

            v_base, f = self._cache[arquivo]
            mesh = trimesh.Trimesh(vertices=v_base.copy(), faces=f.copy(), process=False)
            self._aplicar_transformacao(mesh, pos, rot)

            self._pecas.append((nome, mesh.vertices, mesh.faces, cor, juntas))

    @staticmethod
    def _aplicar_transformacao(mesh, posicao, rotacao):
        """Igual ao transformar_peca do seu montar(), mas sem print."""
        rx, ry, rz = np.radians(rotacao)
        R = trimesh.transformations.euler_matrix(rx, ry, rz, axes="sxyz")
        centro = mesh.bounding_box.centroid
        T_c = trimesh.transformations.translation_matrix(centro)
        T_o = trimesh.transformations.translation_matrix(-centro)
        T_g = trimesh.transformations.translation_matrix(posicao)
        mesh.apply_transform(T_g @ T_c @ R @ T_o)

    def _pivos_em_stl(self, deltas):
        """Pivôs de cada junta, acumulando as rotações anteriores (espaço STL, Y-up)."""
        pivos = {j: np.array(JUNTAS[j]["pivo"], dtype=float) for j in ORDEM}
        for i, j in enumerate(ORDEM):
            ang = JUNTAS[j]["sinal"] * deltas[j]
            if abs(ang) < 1e-9:
                continue
            eixo = JUNTAS[j]["eixo"]
            for j2 in ORDEM[i + 1:]:
                pivos[j2] = _rotacionar_ponto(pivos[j2], pivos[j], eixo, ang)
        return pivos

    def pontos_juntas(self, angulos):
        """Para debug. Devolve (nomes, pontos 3D) dos pivôs das juntas, no referencial do matplotlib.
        Desenhar um ponto em cada pivô para visualizar a localização exata no modelo.
        """
        deltas = {
            "base":       angulos[0] - POSE_NEUTRA[0],
            "ombro":      angulos[1] - POSE_NEUTRA[1],
            "cotovelo":   angulos[2] - POSE_NEUTRA[2],
            "punho_rot":  angulos[3] - POSE_NEUTRA[3],
            "punho_flex": angulos[4] - POSE_NEUTRA[4],
        }
        pivos = self._pivos_em_stl(deltas)
        M = _matriz_mundo()
        M3, t = M[:3, :3], M[:3, 3]
        nomes = list(ORDEM)
        pontos = np.array([M3 @ pivos[j] + t for j in nomes])
        return nomes, pontos

    # ------------------------------------------------------------------ API
    def poligonos(self, angulos):
        """Devolve (lista de triângulos 3D, lista de cores RGBA)."""
        deltas = {
            "base":       angulos[0] - POSE_NEUTRA[0],
            "ombro":      angulos[1] - POSE_NEUTRA[1],
            "cotovelo":   angulos[2] - POSE_NEUTRA[2],
            "punho_rot":  angulos[3] - POSE_NEUTRA[3],
            "punho_flex": angulos[4] - POSE_NEUTRA[4],
        }
        delta_garra = angulos[5] - POSE_NEUTRA[5]

        pivos = self._pivos_em_stl(deltas) 

        polis, cores = [], []
        for nome, v_ref, faces, cor, juntas_peca in self._pecas:
            vertices = v_ref.copy()

            # Rotação das juntas que afetam esta peça
            for j in ORDEM:
                if j not in juntas_peca:
                    continue
                ang = JUNTAS[j]["sinal"] * deltas[j]
                if abs(ang) < 1e-9:
                    continue
                vertices = _rotacionar(vertices, pivos[j], JUNTAS[j]["eixo"], ang)

            # Direção de abertura (X_stl rotacionado pelas juntas desta peça)
            eixo_x = np.array([1.0, 0.0, 0.0])
            eixo_y = np.array([0.0, 1.0, 0.0])
            for j in ORDEM:
                if j not in juntas_peca:
                    continue
                ang = JUNTAS[j]["sinal"] * deltas[j]
                if abs(ang) < 1e-9:
                    continue
                eixo_x = _rotacionar_vetor(eixo_x, JUNTAS[j]["eixo"], ang)
                eixo_y = _rotacionar_vetor(eixo_y, JUNTAS[j]["eixo"], ang)

            # Gripper abre/fecha
            if nome in GARRA_PECAS and abs(delta_garra) > 1e-6:
                desloc = GARRA_PECAS[nome] * delta_garra * ABERTURA_POR_GRAU
                vertices = vertices + desloc * eixo_x

            # Engrenagem gira com a garra
            if nome in GEAR_ROTACAO and abs(delta_garra) > 1e-6:
                ang_gear = delta_garra * GEAR_ROTACAO[nome]
                centro = vertices.mean(axis=0)
                vertices = _rotacionar(vertices, centro, eixo_y, ang_gear)

            rgba = tuple(c / 255.0 for c in cor)
            for f in faces:
                polis.append(vertices[f])
                cores.append(rgba)

        # Converte do referencial Y-up (STL) para Z-up (matplotlib)
        M = _matriz_mundo()
        M3 = M[:3, :3]
        t = M[:3, 3]
        polis_zup = [(M3 @ p.T).T + t for p in polis]

        return polis_zup, cores