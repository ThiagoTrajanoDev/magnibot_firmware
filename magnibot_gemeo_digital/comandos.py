"""Comandos digitados para mexer no modelo 3D, por exemplo:  garra 150   ou   base -30; ombro 100"""
import unicodedata

# Nome de cada junta -> posição na lista [base, ombro, cotovelo, punho_rot, punho_flex, garra].
JUNTAS = {
    "base": 0,
    "ombro": 1,
    "cotovelo": 2,
    "punho_rot": 3, "rotacao": 3,
    "punho_flex": 4, "flexao": 4,
    "garra": 5,
}
NOMES_BONITOS = ["base", "ombro", "cotovelo", "punho_rot", "punho_flex", "garra"]

# Limites aceitos, em graus do servo (a mesma unidade do terminal do firmware: "ombro 127").
# Os servos de 180° vão de 0 a 180; a base (giro contínuo) é livre.
LIMITE_BASE = (-180.0, 180.0)
LIMITE_SERVO = (0.0, 180.0)

AJUDA = (
    "Comandos (vários com ;),\n"
    "em graus do servo:\n"
    "  garra 150\n"
    "  base -30\n"
    "  neutro   (pose do STL)\n"
    "  idle     (pose IDLE)\n"
    "  auto     (segue o ESP32)\n"
    "Juntas: base, ombro,\n"
    "  cotovelo, punho_rot,\n"
    "  punho_flex, garra"
)


def _limpar(texto):
    """Minúsculas, sem acentos e sem espaços sobrando: 'Rotação ' vira 'rotacao'."""
    sem_acento = unicodedata.normalize("NFD", texto.lower())
    sem_acento = "".join(c for c in sem_acento if unicodedata.category(c) != "Mn")
    return " ".join(sem_acento.split())


def interpretar(texto):
    """Transforma o texto digitado numa lista de ações.

    Cada ação é um dicionário:
      {"tipo": "mover", "indice": 5, "nome": "garra", "graus": 150.0}
      {"tipo": "neutro"} | {"tipo": "idle"} | {"tipo": "auto"} | {"tipo": "ajuda"}
      {"tipo": "erro", "mensagem": "..."}
    """
    acoes = []
    for pedaco in _limpar(texto).split(";"):
        pedaco = pedaco.strip()
        if not pedaco:
            continue
        if pedaco in ("neutro", "idle", "auto", "ajuda"):
            acoes.append({"tipo": pedaco})
            continue
        partes = pedaco.split()
        if len(partes) < 2:
            acoes.append({"tipo": "erro", "mensagem": f"'{pedaco}': faltou o ângulo. Exemplo: garra 150"})
            continue
        nome = "_".join(partes[:-1])             # "punho rot 90" vira "punho_rot"
        if nome not in JUNTAS:
            acoes.append({"tipo": "erro", "mensagem": f"junta desconhecida: '{' '.join(partes[:-1])}'. Digite 'ajuda'."})
            continue
        try:
            graus = float(partes[-1].replace(",", "."))   # aceita 12,5 e 12.5
        except ValueError:
            acoes.append({"tipo": "erro", "mensagem": f"'{partes[-1]}' não é um número."})
            continue
        indice = JUNTAS[nome]
        minimo, maximo = LIMITE_BASE if indice == 0 else LIMITE_SERVO
        if not minimo <= graus <= maximo:
            acoes.append({"tipo": "erro", "mensagem": f"{NOMES_BONITOS[indice]}: use de {minimo:.0f} a {maximo:.0f}."})
            continue
        acoes.append({"tipo": "mover", "indice": indice, "nome": NOMES_BONITOS[indice], "graus": graus})
    if not acoes:
        acoes.append({"tipo": "vazio"})
    return acoes
