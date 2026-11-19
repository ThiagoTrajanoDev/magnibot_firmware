"""Gêmeo digital do MagniBot: desenha em 3D o braço que está sendo comandado pelo ESP32.

Como usar:
    python gemeo.py --simular                 # sem o braço, com dados inventados
    python gemeo.py --porta /dev/ttyACM0      # com o ESP32 (no Windows, algo como COM3)
    python gemeo.py --simular --imagem figura.png --em 12   # salva uma figura (12 s do ciclo)
"""
import argparse
import time

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import TextBox
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from braco import POSE_NEUTRA, calcular_pose
from comandos import AJUDA, interpretar
from modelo import ModeloBraco
from leitor import ESTADOS, Simulador, abrir_serial, extrair_angulos, telemetria_em

# Cor de cada estado (as mesmas cores planejadas para o LED da placa).
CORES = {0: "#2ecc71", 1: "#3b82f6", 2: "#d4a90a", 3: "#c026d3", 4: "#ef4444"}

SUAVIZACAO = 0.3   # a cada quadro, o desenho anda 30% do caminho até o ângulo recebido


class Gemeo:
    def __init__(self, fonte, esqueleto=False):
        self.esqueleto = esqueleto              # True: desenha também os "palitos" por cima
        self.modelo = ModeloBraco()
        self.fonte = fonte                      # LeitorSerial ou Simulador
        self.angulos_alvo = list(POSE_NEUTRA)   # último ângulo recebido
        self.angulos_desenho = list(POSE_NEUTRA)  # ângulo que está desenhado agora (suavizado)
        self.telemetria = None                  # última telemetria recebida
        self.tem_angulos = False
        self.ultima_msg = None                  # horário da última telemetria
        self.manual = False                     # True: o modelo obedece aos comandos digitados
        self.angulos_manual = list(POSE_NEUTRA)  # ângulos pedidos pelos comandos
        self.resposta = ""                      # última resposta aos comandos digitados

        self._montar_janela()

    # ------------------------------------------------------------------ janela
    def _montar_janela(self):
        self.fig = plt.figure(figsize=(11, 7))
        self.fig.canvas.manager.set_window_title("Gêmeo digital - MagniBot") if self.fig.canvas.manager else None
        self.ax = self.fig.add_axes([0.0, 0.0, 0.72, 1.0], projection="3d")
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

        # Os "palitos" (esqueleto), úteis para conferir a geometria.
        self.linha_corpo, = self.ax.plot([], [], [], "-o", color="#0b6e8f", lw=6, ms=9)
        self.linha_dedo_a, = self.ax.plot([], [], [], "-", color="#14202b", lw=4)
        self.linha_dedo_b, = self.ax.plot([], [], [], "-", color="#14202b", lw=4)
        self.linha_barra, = self.ax.plot([], [], [], "-", color="#14202b", lw=4)
        for linha in (self.linha_corpo, self.linha_dedo_a, self.linha_dedo_b, self.linha_barra):
            linha.set_visible(self.esqueleto)

        # Painel de texto à direita.
        self.txt_estado = self.fig.text(0.76, 0.88, "", fontsize=18, fontweight="bold")
        self.txt_info = self.fig.text(0.76, 0.84, "", fontsize=11, va="top", family="monospace")
        self.txt_aviso = self.fig.text(0.76, 0.54, "", fontsize=9, va="top", color="#7a4b00", wrap=True)

        # Caixa para digitar comandos (clique nela, escreva e aperte Enter).
        self.fig.text(0.76, 0.375, "Comando (ex.: garra 150):", fontsize=9)
        self.caixa = TextBox(self.fig.add_axes([0.76, 0.31, 0.22, 0.05]), "", initial="")
        self.caixa.on_submit(self._enviou)
        self.txt_resposta = self.fig.text(0.76, 0.295, "", fontsize=8.5, va="top", color="#14202b", wrap=True)

    # ----------------------------------------------------------------- dados
    def receber(self):
        """Pega o que chegou da fonte e guarda a mais recente."""
        for tel in self.fonte.ler():
            self.telemetria = tel
            self.ultima_msg = time.time()
            angulos = extrair_angulos(tel)
            self.tem_angulos = angulos is not None
            if not self.manual:                 # no modo manual, os ângulos vêm dos comandos
                self.angulos_alvo = angulos if angulos is not None else list(POSE_NEUTRA)

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
                self.angulos_manual = list(POSE_NEUTRA)
                respostas.append("ok: posição neutra")
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
        """Aproxima o desenho do alvo aos poucos (a telemetria real chega só 2 vezes por segundo)."""
        self.angulos_desenho = [
            atual + (alvo - atual) * SUAVIZACAO
            for atual, alvo in zip(self.angulos_desenho, self.angulos_alvo)
        ]

    # ---------------------------------------------------------------- desenho
    def desenhar(self):
        poligonos, cores = self.modelo.poligonos(self.angulos_desenho)
        self.colecao.set_verts(poligonos)
        self.colecao.set_facecolor(cores)

        pose = calcular_pose(self.angulos_desenho)
        corpo = pose["corpo"]
        self.linha_corpo.set_data_3d(corpo[:, 0], corpo[:, 1], corpo[:, 2])
        for linha, (ini, fim) in ((self.linha_dedo_a, pose["dedos"][0]), (self.linha_dedo_b, pose["dedos"][1])):
            linha.set_data_3d([ini[0], fim[0]], [ini[1], fim[1]], [ini[2], fim[2]])
        a, b = pose["barra"]
        self.linha_barra.set_data_3d([a[0], b[0]], [a[1], b[1]], [a[2], b[2]])

        # Texto do painel.
        self.txt_resposta.set_text(self.resposta)
        if self.telemetria is None and self.manual:
            self.txt_estado.set_text("MANUAL")
            self.txt_estado.set_color("#0b6e8f")
            self.txt_info.set_text("\n".join(f"{n:<10}{v:6.1f}°" for n, v in zip(
                ["base", "ombro", "cotovelo", "punho rot", "punho flex", "garra"], self.angulos_alvo)))
            self.txt_aviso.set_text("Modo manual. Digite 'auto' para voltar aos dados do ESP32.")
            return
        if self.telemetria is None:
            self.txt_estado.set_text("Aguardando dados...")
            self.txt_estado.set_color("#566574")
            self.txt_info.set_text("")
            self.txt_aviso.set_text("Nenhuma telemetria recebida ainda.")
            return
        tel = self.telemetria
        codigo = tel.get("state_code")
        self.txt_estado.set_text(ESTADOS.get(codigo, str(tel.get("state"))))
        self.txt_estado.set_color(CORES.get(codigo, "#566574"))
        distancia = tel.get("distance_mm")
        nomes = ["base", "ombro", "cotovelo", "punho rot", "punho flex", "garra"]
        linhas = [f"distância: {'--' if distancia is None else str(distancia) + ' mm'}", ""]
        linhas += [f"{n:<10}{v:6.1f}°" for n, v in zip(nomes, self.angulos_alvo)]
        self.txt_info.set_text("\n".join(linhas))
        avisos = []
        if self.manual:
            avisos.append("MODO MANUAL: o modelo obedece aos seus comandos.\nDigite 'auto' para voltar aos dados do ESP32.")
        elif not self.tem_angulos:
            avisos.append("O firmware ainda envia angles_deg = null.\nBraço mostrado na posição neutra.")
        if self.ultima_msg is not None and time.time() - self.ultima_msg > 2:
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
    args = ap.parse_args()

    if args.imagem:                              # modo figura: útil para o relatório
        gemeo = Gemeo(Simulador(), args.esqueleto)
        tel = telemetria_em(args.em)
        gemeo.telemetria = tel
        gemeo.ultima_msg = time.time()
        angulos = extrair_angulos(tel)
        gemeo.tem_angulos = angulos is not None
        gemeo.angulos_alvo = gemeo.angulos_desenho = angulos or list(POSE_NEUTRA)
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

    gemeo = Gemeo(fonte, args.esqueleto)
    animacao = FuncAnimation(gemeo.fig, gemeo.quadro, interval=50, cache_frame_data=False)  # 20 quadros/s
    try:
        plt.show()
    finally:
        fonte.fechar()
    return animacao


if __name__ == "__main__":
    main()
