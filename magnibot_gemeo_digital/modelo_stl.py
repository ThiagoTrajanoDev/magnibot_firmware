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
# Pivôs, eixos e sinais das juntas (espaço STL, Y = cima, mm), todos na POSE NEUTRA.
# Os pivôs são os centros dos cubos arredondados nas pontas das peças (medidos nos STLs
# já montados). Os braços ficam centrados em x ~ -11 e z ~ -13, não na origem.
# Para ombro, cotovelo e punho_flex (eixo X) o x do pivô não importa; para punho_rot (eixo Y)
# o pivô tem de estar sobre a linha central do Arm 02/Arm 03.
# Se algum movimento ficar torto, ajuste o pivô/eixo/sinal daquela junta e teste com --simular --pivos.
# ---------------------------------------------------------------------------
JUNTAS = {
    "base":       {"pivo": (0.0,     0.0,   0.0), "eixo": (0, 1, 0), "sinal": +1},  # yaw vertical
    "ombro":      {"pivo": (0.0,    85.0, -11.0), "eixo": (1, 0, 0), "sinal": -1},  # cubo de baixo do Arm 01
    "cotovelo":   {"pivo": (0.0,   222.0, -14.0), "eixo": (1, 0, 0), "sinal": -1},  # cubo de cima do Arm 01
    "punho_rot":  {"pivo": (-11.0, 308.0, -13.5), "eixo": (0, 1, 0), "sinal": -1},  # linha central do Arm 02/03
    "punho_flex": {"pivo": (0.0,   331.0, -13.4), "eixo": (1, 0, 0), "sinal": -1},  # ressalto do Gripper base
}
# Cadeia física, da raiz para a ponta: o Arm 03 é movido pelo punho_rot, e a garra pelo punho_flex.
ORDEM = ["base", "ombro", "cotovelo", "punho_rot", "punho_flex"]

GIRO_Z = -90 # rotacao do modelo em stl para conformidade com matplotlib

# Centros XY dos furos nos STLs montados, em mm. Cada lado tem um
# mecanismo de quatro barras: base, engrenagem, dedo e grip link.
# Os links superior e inferior compartilham os mesmos eixos XY.
LADOS_GARRA = (
    ("Direito", "Gear 1", +1, (14.40, 374.14), (32.04, 399.33), (6.40, 394.50),
     ((32.69, 399.37), (24.45, 419.77)), ((6.61, 394.30), (24.39, 419.70))),
    ("Esquerdo", "Gear 2", -1, (-12.55, 374.30), (-30.83, 399.01), (-3.60, 394.50),
     ((-30.22, 399.58), (-21.27, 419.67)), ((-3.61, 394.30), (-21.39, 419.70))),
)
DISTANCIA_FUROS_DEDO = 22.0
DISTANCIA_FUROS_LINK = 31.0
# Curso visual conservador em torno da montagem de referência (garra a 90°).
CURSO_ENGRENAGEM = 80.0

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


def _alinhar_furos(furos, inicio, fim):
    """Move uma peça rígida no plano XY, alinhando seus dois furos."""
    origem, segundo = np.asarray(furos)
    antes = segundo - origem
    depois = fim - inicio
    angulo = np.arctan2(depois[1], depois[0]) - np.arctan2(antes[1], antes[0])
    matriz = trimesh.transformations.rotation_matrix(angulo, (0, 0, 1))
    matriz[:2, 3] = inicio - matriz[:2, :2] @ origem
    return matriz


def _matrizes_garra(graus):
    matrizes = {}
    delta = (np.clip(graus, 0, 180) - POSE_NEUTRA[5]) * CURSO_ENGRENAGEM / 180
    for lado, gear, sinal, pivo, pino, apoio, furos_dedo, furos_link in LADOS_GARRA:
        giro = trimesh.transformations.rotation_matrix(
            np.radians(-sinal * delta), (0, 0, 1), point=(*pivo, 0)
        )
        pino_movel = giro[:2, :2] @ pino + giro[:2, 3]
        apoio_fixo = np.asarray(apoio)

        # O pino comum do dedo e do link é a interseção dos círculos
        # centrados no pino móvel da engrenagem e no apoio fixo do link.
        entre_centros = apoio_fixo - pino_movel
        distancia = np.linalg.norm(entre_centros)
        direcao = entre_centros / distancia
        perpendicular = np.array([-direcao[1], direcao[0]])
        projecao = (DISTANCIA_FUROS_DEDO**2 - DISTANCIA_FUROS_LINK**2 + distancia**2) / (2 * distancia)
        altura = np.sqrt(DISTANCIA_FUROS_DEDO**2 - projecao**2)
        articulacao = pino_movel + projecao * direcao - sinal * altura * perpendicular

        matrizes[gear] = giro
        matrizes[f"Gripper {lado}"] = _alinhar_furos(furos_dedo, pino_movel, articulacao)
        link = _alinhar_furos(furos_link, apoio_fixo, articulacao)
        matrizes[f"Grip Link {lado}"] = link
        matrizes[f"Grip Link {lado} Baixo"] = link
    return matrizes


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

    @staticmethod
    def _deltas(angulos):
        """Quanto cada junta saiu da pose neutra, em graus do modelo."""
        return {j: angulos[i] - POSE_NEUTRA[i] for i, j in enumerate(
            ["base", "ombro", "cotovelo", "punho_rot", "punho_flex"])}

    @staticmethod
    def _matrizes(deltas):
        """Rotação 4x4 de cada junta, com o pivô e o eixo da POSE NEUTRA."""
        return {
            j: trimesh.transformations.rotation_matrix(
                np.radians(JUNTAS[j]["sinal"] * deltas[j]),
                np.asarray(JUNTAS[j]["eixo"], dtype=float), point=JUNTAS[j]["pivo"])
            for j in ORDEM
        }

    @staticmethod
    def _cadeia(matrizes, juntas):
        """Composição da raiz para a ponta: T = T_base @ T_ombro @ ...

        Aplicada a um ponto da pose neutra, a junta mais da ponta age primeiro e cada junta
        anterior leva junto as seguintes (pivôs e eixos), como num braço de verdade.
        """
        T = np.eye(4)
        for j in ORDEM:
            if j in juntas:
                T = T @ matrizes[j]
        return T

    def pontos_juntas(self, angulos):
        """Para debug. Devolve (nomes, pontos 3D) dos pivôs das juntas, no referencial do matplotlib.
        Desenhar um ponto em cada pivô para visualizar a localização exata no modelo.
        """
        matrizes = self._matrizes(self._deltas(angulos))
        M = _matriz_mundo()
        pontos = []
        for i, j in enumerate(ORDEM):
            T = M @ self._cadeia(matrizes, ORDEM[:i])          # só as juntas antes desta movem o pivô
            pontos.append((T @ np.append(JUNTAS[j]["pivo"], 1.0))[:3])
        return list(ORDEM), np.array(pontos)

    # ------------------------------------------------------------------ API
    def poligonos(self, angulos):
        """Devolve (lista de triângulos 3D, lista de cores RGBA)."""
        matrizes = self._matrizes(self._deltas(angulos))
        garra = _matrizes_garra(angulos[5])

        polis, cores = [], []
        for nome, v_ref, faces, cor, juntas_peca in self._pecas:
            # Juntas que movem esta peça, da ponta para a raiz.
            T = self._cadeia(matrizes, juntas_peca)
            if nome in garra:
                T = T @ garra[nome]
            vertices = v_ref @ T[:3, :3].T + T[:3, 3]

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
