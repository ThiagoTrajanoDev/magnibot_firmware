"""Testes simples. Rode com:  python test_gemeo.py   (se não imprimir erro, está tudo certo)."""
import math

import numpy as np

from braco import calcular_quadros, ALTURA_BASE, COMPR_ANTEBRACO, COMPR_BRACO, COMPR_PUNHO, POSE_NEUTRA, calcular_pose
from leitor import LeitorSerial, extrair_angulos, interpretar_linha, telemetria_em

# Linha real de telemetria do firmware (formato de app_main.c).
LINHA_REAL = (
    '{"type":"telemetry","version":1,"uptime_ms":1500,"state":"IDLE","state_code":0,'
    '"mode":"bench","motion_enabled":false,"estop_configured":true,"angles_deg":null,'
    '"distance_mm":null,"current_ma":null,"gripper_percent":null,'
    '"execution":{"backend":"disabled","axes_configured":0,"phase":"idle","fault":"none"},'
    '"bench_servo":{"enabled":false,"channels":0},"stack_min_free_bytes":2048}'
)


def test_interpretar_linha():
    assert interpretar_linha(LINHA_REAL)["state"] == "IDLE"
    assert interpretar_linha("I (320) cpu_start: Starting scheduler") is None   # log comum
    assert interpretar_linha('{"type":"response","version":1}') is None         # não é telemetria
    assert interpretar_linha('{"type":"telemetry","version":2}') is None        # versão errada
    assert interpretar_linha('{"type":"telemetry","vers') is None               # linha cortada


def test_extrair_angulos():
    assert extrair_angulos(interpretar_linha(LINHA_REAL)) is None               # null no firmware atual
    assert extrair_angulos({"angles_deg": [1, 2, 3, 4, 5, 6]}) == [1, 2, 3, 4, 5, 6]
    nomes = dict(base=1, ombro=2, cotovelo=3, punho_rot=4, punho_flex=5, garra=6)
    assert extrair_angulos({"angles_deg": nomes}) == [1, 2, 3, 4, 5, 6]
    assert extrair_angulos({"angles_deg": [1, None, 3, 4, 5, 6]}) is None


class PortaFalsa:
    """Imita a porta serial entregando os bytes em pedaços, como acontece de verdade."""
    def __init__(self, pedacos):
        self.pedacos = list(pedacos)
        self.in_waiting = 0

    def read(self, _n):
        if not self.pedacos:
            return b""
        dado = self.pedacos.pop(0)
        return dado

    def close(self):
        pass


def test_leitor_serial_com_linha_partida():
    bytes_total = ("boot...\n" + LINHA_REAL + "\n" + LINHA_REAL + "\n").encode()
    meio = len(bytes_total) // 2
    porta = PortaFalsa([bytes_total[:meio], bytes_total[meio:]])
    leitor = LeitorSerial(porta)
    achadas = leitor.ler() + leitor.ler()
    assert len(achadas) == 2     # as duas linhas, mesmo com a primeira cortada no meio


def test_pose_neutra_fica_em_pe():
    corpo = calcular_pose(POSE_NEUTRA)["corpo"]
    assert abs(corpo[-1][0]) < 1e-9 and abs(corpo[-1][1]) < 1e-9               # sem sair do eixo
    assert abs(corpo[-1][2] - (ALTURA_BASE + COMPR_BRACO + COMPR_ANTEBRACO + COMPR_PUNHO)) < 1e-9


def test_base_gira_o_braco():
    pose = [90.0, 180.0, 90.0, 90.0, 90.0, 90.0]   # ombro deitado para a frente (180°), base em 90°
    ponta = calcular_pose(pose)["corpo"][2]         # cotovelo
    assert abs(ponta[0]) < 1e-6 and abs(ponta[1] - COMPR_BRACO) < 1e-6   # foi para o eixo y


def test_garra_abre_e_fecha():
    aberta = calcular_pose([0, 90, 90, 90, 90, 180])["dedos"]
    fechada = calcular_pose([0, 90, 90, 90, 90, 0])["dedos"]
    dist = lambda d: math.dist(d[0][0], d[1][0])
    assert dist(aberta) > dist(fechada) and dist(fechada) < 1e-9


def test_simulador():
    for t in range(0, 40):
        tel = telemetria_em(t)
        assert tel["type"] == "telemetry" and len(tel["angles_deg"]) == 6


def test_quadros_batem_com_os_palitos():
    """O modelo 3D e os palitos usam a mesma geometria: as articulações têm de coincidir."""
    for ang in ([0, 90, 90, 90, 90, 90], [60, 55, 125, 90, 105, 150], [-45, 120, 70, 40, 130, 20]):
        corpo = calcular_pose(ang)["corpo"]
        quadros = calcular_quadros(ang)
        assert np.allclose(quadros["braco"][1], corpo[1])        # ombro
        assert np.allclose(quadros["antebraco"][1], corpo[2])    # cotovelo
        assert np.allclose(quadros["punho"][1], corpo[3])        # punho


def test_modelo_3d():
    from modelo import ModeloBraco
    polis, cores = ModeloBraco().poligonos([60, 55, 125, 90, 105, 150])
    assert len(polis) == len(cores) > 100
    assert all(np.isfinite(p).all() for p in polis)


def test_comandos():
    from comandos import interpretar
    assert interpretar("garra 150") == [{"tipo": "mover", "indice": 5, "nome": "garra", "graus": 150.0}]
    assert interpretar("  GARRA   12,5 ")[0]["graus"] == 12.5                  # maiúsculas, espaços e vírgula
    assert interpretar("punho rot 45")[0]["nome"] == "punho_rot"               # nome com espaço
    assert interpretar("rotação 45")[0]["nome"] == "punho_rot"                 # apelido com acento
    assert interpretar("base -30")[0]["graus"] == -30                          # a base aceita negativo
    assert [a["tipo"] for a in interpretar("neutro; auto; ajuda")] == ["neutro", "auto", "ajuda"]
    assert [a["nome"] for a in interpretar("base 10; ombro 100")] == ["base", "ombro"]
    for ruim in ("garra", "banana 10", "garra abc", "garra 181", "ombro -5", ""):
        assert interpretar(ruim)[0]["tipo"] in ("erro", "vazio"), ruim


def test_executar_no_gemeo():
    import matplotlib
    matplotlib.use("Agg")
    from gemeo import Gemeo
    from leitor import Simulador
    g = Gemeo(Simulador())
    g.executar("garra 150")
    assert g.manual and g.angulos_alvo[5] == 150
    g.receber()                                    # dados do simulador não podem sobrescrever o manual
    assert g.angulos_alvo[5] == 150
    g.executar("auto")
    assert not g.manual


if __name__ == "__main__":
    for nome, funcao in list(globals().items()):
        if nome.startswith("test_"):
            funcao()
            print("ok ", nome)
