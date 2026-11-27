"""Gêmeo digital do MagniBot: desenha em 3D o braço que está sendo comandado pelo ESP32.

Como usar:
    python gemeo.py --simular                               # sem o braço, com dados inventados
    python gemeo.py --porta /dev/ttyACM0                    # com o ESP32 (no Windows, algo como COM3)
    python gemeo.py --simular --imagem figura.png --em 12   # salva uma figura (12 s do ciclo)
Debug:
    python gemeo.py --esqueleto                             # palitos apresentando como o modelo deveria se comportar
    python gemeo.py --pivos                                 # pontos representando os pivos de rotacao de cada parte do modelo 
"""
import argparse
import time
from typing import cast

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, TextBox
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3D, Path3DCollection, Poly3DCollection

from braco import calcular_pose
from calibracao import POSE_IDLE_SERVO, POSE_NEUTRA_SERVO, servo_para_modelo
from comandos import AJUDA, interpretar
from modelo_stl import ModeloBraco
#from modelo import ModeloBraco
from leitor import ESTADOS, LeitorSerial, Simulador, abrir_serial, base_sem_calibracao, extrair_angulos, telemetria_em

# Cor de cada estado (as mesmas cores planejadas para o LED da placa).
CORES = {0: "#2ecc71", 1: "#3b82f6", 2: "#d4a90a", 3: "#c026d3", 4: "#ef4444"}

SUAVIZACAO = 0.3   # a cada quadro, o desenho anda 30% do caminho até o ângulo recebido
CONTROLES = {"start": "Start", "estop": "Emergência"}


class Gemeo:
    def __init__(self, fonte: LeitorSerial | Simulador, esqueleto=False, pivos=False):
        self.esqueleto = esqueleto              # True: desenha também os "palitos" por cima
        self.mostrar_pivos = pivos              # True: desenha pontos nos pivos das pecas
        self.modelo = ModeloBraco()
        self.fonte = fonte                      # LeitorSerial ou Simulador
        # Todos os ângulos guardados aqui estão em graus do servo (a unidade do firmware);
        # a conversão para o modelo 3D (calibracao.py) acontece só na hora de desenhar.
        self.angulos_alvo = list(POSE_NEUTRA_SERVO)     # último ângulo recebido
        self.angulos_desenho = list(POSE_NEUTRA_SERVO)  # ângulo que está desenhado agora (suavizado)
        self.telemetria = None                  # última telemetria recebida
        self.tem_angulos = False
        self.ultima_msg = None                  # horário da última telemetria
        self.manual = False                     # True: o modelo obedece aos comandos digitados
        self.angulos_manual = list(POSE_NEUTRA_SERVO)  # ângulos pedidos pelos comandos
        self.resposta = ""                      # última resposta aos comandos digitados
        self.pendentes = {}
        self.retorno_controle = {}
        self.erro_serial = False

        self._montar_janela()

    # ------------------------------------------------------------------ janela
    def _montar_janela(self):
        self.fig = plt.figure(figsize=(11, 7))
        self.fig.canvas.manager.set_window_title("Gêmeo digital - MagniBot") if self.fig.canvas.manager else None
        self.ax = cast(Axes3D, self.fig.add_axes((0.0, 0.0, 0.72, 1.0), projection="3d"))
        self.ax.set_xlim(-300, 300)
        self.ax.set_ylim(-300, 300)
        self.ax.set_zlim(0, 400)
        self.ax.set_box_aspect((1, 1, 0.67))
        self.ax.set_xlabel("x (mm)")
        self.ax.set_ylabel("y (mm)")
        self.ax.set_zlabel("z (mm)")
        self.ax.view_init(elev=22, azim=-60)

        # Chão: um círculo, e uma risca no grau 0 da base (para a gente se orientar).
        giro = np.linspace(0, 2 * np.pi, 80)
        self.ax.plot(200 * np.cos(giro), 200 * np.sin(giro), 0, color="#9aa5b1", lw=1)
        self.ax.plot([0, 200], [0, 0], 0, color="#e67e22", lw=2)
        self.ax.text(215, 0, 0, "grau 0", color="#e67e22", fontsize=9)

        # Peças do modelo 3D (uma coleção só; a cada quadro trocamos os polígonos).
        self.colecao = Poly3DCollection([], edgecolors=(0, 0, 0, 0.25), linewidths=0.4)
        self.ax.add_collection3d(self.colecao)

        # Pivos debug
        self.pontos_pivos = cast(Path3DCollection, self.ax.scatter(
            [0], [0], 0, s=300, c="red",
            edgecolors="white", linewidths=1.2,
            depthshade=False, zorder=100,
        ))
        self.pontos_pivos.set_visible(self.mostrar_pivos)

        # Os "palitos" (esqueleto), úteis para conferir a geometria.
        self.linha_corpo = Line3D([], [], [], marker="o", color="#0b6e8f", lw=6, ms=9)
        self.linha_dedo_a = Line3D([], [], [], color="#14202b", lw=4)
        self.linha_dedo_b = Line3D([], [], [], color="#14202b", lw=4)
        self.linha_barra = Line3D([], [], [], color="#14202b", lw=4)
        for linha in (self.linha_corpo, self.linha_dedo_a, self.linha_dedo_b, self.linha_barra):
            self.ax.add_line(linha)
            linha.set_visible(self.esqueleto)

        # Painel de texto à direita.
        self.botoes = {}
        self.txt_controle = None
        if isinstance(self.fonte, LeitorSerial):
            for comando, x, cor in (("start", 0.76, "#ccebd9"), ("estop", 0.87, "#ffc4c4")):
                botao = Button(self.fig.add_axes((x, 0.95, 0.105, 0.036)), CONTROLES[comando], color=cor)
                botao.on_clicked(lambda _evento, comando=comando: self.enviar_controle(comando))
                self.botoes[comando] = botao
            self.txt_controle = self.fig.text(0.76, 0.94, "", fontsize=8, va="top")
        self.txt_estado = self.fig.text(0.76, 0.86, "", fontsize=18, fontweight="bold")
        self.txt_info = self.fig.text(0.76, 0.84, "", fontsize=11, va="top", family="monospace")
        self.txt_aviso = self.fig.text(0.76, 0.54, "", fontsize=9, va="top", color="#7a4b00", wrap=True)

        # Caixa para digitar comandos (clique nela, escreva e aperte Enter).
        self.fig.text(0.76, 0.375, "Comando (ex.: garra 150):", fontsize=9)
        self.caixa = TextBox(self.fig.add_axes((0.76, 0.31, 0.22, 0.05)), "", initial="")
        self.caixa.on_submit(self._enviou)
        self.txt_resposta = self.fig.text(0.76, 0.295, "", fontsize=8.5, va="top", color="#14202b", wrap=True)

    # ----------------------------------------------------------------- dados
    def enviar_controle(self, comando):
        if not isinstance(self.fonte, LeitorSerial) or comando in self.pendentes:
            return
        try:
            identificador = self.fonte.enviar(comando)
        except OSError:
            self.retorno_controle[comando] = "falha no envio"
            return
        self.pendentes[comando] = (identificador, time.monotonic() + 2)
        self.retorno_controle[comando] = "aguardando resposta"

    def atualizar_controles(self, respostas):
        agora = time.monotonic()
        for comando, (identificador, prazo) in list(self.pendentes.items()):
            resposta = next((r for r in respostas if r.get("id") == identificador), None)
            if resposta is not None:
                codigo = resposta.get("code", "?")
                resultado = "aceito pelo ESP32" if resposta.get("ok") is True else f"recusado ({codigo})"
            elif agora >= prazo:
                resultado = "sem confirmação do ESP32"
            else:
                continue
            self.retorno_controle[comando] = resultado
            del self.pendentes[comando]

    def receber(self):
        """Pega o que chegou da fonte e guarda a mais recente."""
        try:
            telemetrias = self.fonte.ler()
        except OSError:
            self.erro_serial = True
            self.atualizar_controles([])
            return
        self.erro_serial = False
        if isinstance(self.fonte, LeitorSerial):
            self.atualizar_controles(self.fonte.respostas)
        for tel in telemetrias:
            self.telemetria = tel
            self.ultima_msg = time.monotonic()
            angulos = extrair_angulos(tel)
            self.tem_angulos = angulos is not None
            if not self.manual:                 # no modo manual, os ângulos vêm dos comandos
                self.angulos_alvo = angulos if angulos is not None else list(POSE_NEUTRA_SERVO)

    # --------------------------------------------------------------- comandos
    def _enviou(self, texto):
        """Chamado quando você aperta Enter na caixa de comando."""
        if not texto.strip():
            return
        self.executar(texto)
        self.caixa.set_val("")                  # limpa a caixa para o próximo comando

    def executar(self, texto):
        """Aplica o que foi digitado ao modelo e guarda a resposta para mostrar na tela."""
        respostas = []
        for acao in interpretar(texto):
            tipo = acao["tipo"]
            if tipo == "mover":
                if not self.manual:                 # entra no modo manual a partir da pose atual
                    self.manual = True
                    self.angulos_manual = list(self.angulos_alvo)
                self.angulos_manual[acao["indice"]] = acao["graus"]
                respostas.append(f"ok: {acao['nome']} = {acao['graus']:g}°")
            elif tipo == "neutro":
                self.manual = True
                self.angulos_manual = list(POSE_NEUTRA_SERVO)
                respostas.append("ok: posição neutra (pose do STL)")
            elif tipo == "idle":
                self.manual = True
                self.angulos_manual = list(POSE_IDLE_SERVO)
                respostas.append("ok: pose IDLE do firmware")
            elif tipo == "auto":
                self.manual = False
                respostas.append("ok: seguindo o ESP32/simulador")
            elif tipo == "ajuda":
                respostas.append(AJUDA)
            elif tipo == "erro":
                respostas.append("erro: " + acao["mensagem"])
        if self.manual:
            self.angulos_alvo = list(self.angulos_manual)
        self.resposta = "\n".join("\n".join(respostas).split("\n")[-10:])   # só as últimas 10 linhas
        return self.resposta

    def suavizar(self):
        """Aproxima o desenho do alvo aos poucos (a telemetria chega de 2 a 10 vezes por segundo)."""
        self.angulos_desenho = [
            atual + (alvo - atual) * SUAVIZACAO
            for atual, alvo in zip(self.angulos_desenho, self.angulos_alvo)
        ]

    # ---------------------------------------------------------------- desenho
    def desenhar(self):
        angulos_modelo = servo_para_modelo(self.angulos_desenho)
        poligonos, cores = self.modelo.poligonos(angulos_modelo)
        self.colecao.set_verts(poligonos)
        self.colecao.set_facecolor(cores)

        if self.mostrar_pivos:
            _, pontos = self.modelo.pontos_juntas(angulos_modelo)
            self.pontos_pivos.set_offsets(pontos[:, :2])
            self.pontos_pivos.set_3d_properties(pontos[:, 2], "z")

        pose = calcular_pose(angulos_modelo)
        corpo = pose["corpo"]
        self.linha_corpo.set_data_3d(corpo[:, 0], corpo[:, 1], corpo[:, 2])
        for linha, (ini, fim) in ((self.linha_dedo_a, pose["dedos"][0]), (self.linha_dedo_b, pose["dedos"][1])):
            linha.set_data_3d([ini[0], fim[0]], [ini[1], fim[1]], [ini[2], fim[2]])
        a, b = pose["barra"]
        self.linha_barra.set_data_3d([a[0], b[0]], [a[1], b[1]], [a[2], b[2]])

        # Texto do painel.
        self.txt_resposta.set_text(self.resposta)
        if self.txt_controle is not None:
            self.txt_controle.set_text("\n".join(
                f"{CONTROLES[comando]}: {retorno}" for comando, retorno in self.retorno_controle.items()
            ))
        if self.telemetria is None and self.manual:
            self.txt_estado.set_text("MANUAL")
            self.txt_estado.set_color("#0b6e8f")
            self.txt_info.set_text("\n".join(f"{n:<10}{v:6.1f}°" for n, v in zip(
                ["base", "ombro", "cotovelo", "punho rot", "punho flex", "garra"], self.angulos_alvo)))
            self.txt_aviso.set_text("Falha de leitura da porta serial." if self.erro_serial else
                                    "Modo manual. Digite 'auto' para voltar aos dados do ESP32.")
            return
        if self.telemetria is None:
            self.txt_estado.set_text("Aguardando dados...")
            self.txt_estado.set_color("#566574")
            self.txt_info.set_text("")
            self.txt_aviso.set_text("Falha de leitura da porta serial." if self.erro_serial else
                                    "Nenhuma telemetria recebida ainda.")
            return
        tel = self.telemetria
        codigo = tel.get("state_code")
        if not isinstance(codigo, int):
            codigo = -1
        self.txt_estado.set_text(ESTADOS.get(codigo, str(tel.get("state"))))
        self.txt_estado.set_color(CORES.get(codigo, "#566574"))
        distancia = tel.get("distance_mm")
        nomes = ["base", "ombro", "cotovelo", "punho rot", "punho flex", "garra"]
        linhas = [f"distância: {'--' if distancia is None else str(distancia) + ' mm'}", "",
                  "graus do servo:"]
        linhas += [f"{n:<10}{v:6.1f}°" for n, v in zip(nomes, self.angulos_alvo)]
        self.txt_info.set_text("\n".join(linhas))
        avisos = []
        if codigo == 4:
            falha = tel.get("pickup_fault")
            busca = tel.get("search_status")
            if falha and falha != "none":
                avisos.append(f"Falha na pega: {falha}")
            elif busca in {"sensor_fault", "no_object"}:
                avisos.append(f"Busca interrompida: {busca}")
        if self.manual:
            avisos.append("MODO MANUAL: o modelo obedece aos seus comandos.\nDigite 'auto' para voltar aos dados do ESP32.")
        elif not self.tem_angulos:
            avisos.append("A telemetria veio sem ângulos (angles_deg).\nBraço mostrado na posição neutra.")
        elif base_sem_calibracao(tel):
            avisos.append("Base sem calibração: desenhada em 0°.\nVer BASE_GRAUS_POR_S em calibracao.py.")
        if self.erro_serial:
            avisos.append("Falha de leitura da porta serial.")
        elif self.ultima_msg is not None and time.monotonic() - self.ultima_msg > 2:
            avisos.append("Sem dados há mais de 2 s. Cabo ou firmware?")
        self.txt_aviso.set_text("\n\n".join(avisos))

    def quadro(self, _numero):
        """Chamado pelo matplotlib várias vezes por segundo."""
        self.receber()
        self.suavizar()
        self.desenhar()
        return []


def main():
    ap = argparse.ArgumentParser(description="Gêmeo digital do MagniBot")
    ap.add_argument("--porta", help="porta serial do ESP32, ex.: /dev/ttyACM0 ou COM3")
    ap.add_argument("--simular", action="store_true", help="usa dados inventados, sem o braço")
    ap.add_argument("--imagem", help="em vez de abrir a janela, salva uma figura PNG (só com --simular)")
    ap.add_argument("--esqueleto", action="store_true", help="mostra também os palitos por cima do modelo")
    ap.add_argument("--comando", action="append", help="comando para o modelo na figura, ex.: \"garra 150\" (pode repetir)")
    ap.add_argument("--em", type=float, default=12.0, help="instante do ciclo simulado para a figura, em segundos")
    ap.add_argument("--pivos", action="store_true", help="mostra um ponto vermelho no pivô de cada junta (debug)")
    args = ap.parse_args()

    if args.imagem:                              # modo figura: útil para o relatório
        gemeo = Gemeo(Simulador(), args.esqueleto, args.pivos)
        tel = telemetria_em(args.em)
        gemeo.telemetria = tel
        gemeo.ultima_msg = time.monotonic()
        angulos = extrair_angulos(tel)
        gemeo.tem_angulos = angulos is not None
        gemeo.angulos_alvo = gemeo.angulos_desenho = angulos or list(POSE_NEUTRA_SERVO)
        for comando in args.comando or []:
            print(comando, "->", gemeo.executar(comando))
            gemeo.angulos_desenho = list(gemeo.angulos_alvo)
        gemeo.desenhar()
        gemeo.fig.savefig(args.imagem, dpi=130)
        print("Figura salva em", args.imagem)
        return

    if args.simular:
        fonte = Simulador()
    elif args.porta:
        fonte = abrir_serial(args.porta)
    else:
        ap.error("informe --porta ou --simular")

    gemeo = Gemeo(fonte, args.esqueleto, args.pivos)
    animacao = FuncAnimation(gemeo.fig, gemeo.quadro, interval=50, cache_frame_data=False)  # 20 quadros/s
    try:
        plt.show()
    finally:
        fonte.fechar()
    return animacao


if __name__ == "__main__":
    main()
