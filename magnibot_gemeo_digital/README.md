# MagniBot – gêmeo digital

Programa de computador que desenha o braço em 3D e mostra o estado, a distância e os
ângulos, usando a telemetria que o ESP32-S3 já envia pela USB
(protocolo MB1, em `magnibot_agent_configurations/docs/serial-protocol.md`).

Este programa **só lê**: não manda nenhum comando ao braço.

## Instalar (uma vez)

No terminal do VS Code, dentro desta pasta:

```sh
python -m venv .venv
source .venv/bin/activate        # no Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

## Rodar

Sem o braço (dados inventados, para treinar e desenvolver):

```sh
python gemeo.py --simular
```

Com o ESP32 ligado pela USB:

```sh
python gemeo.py --porta /dev/ttyACM0      # no Windows: --porta COM3
```

Salvar uma figura para o relatório (sem abrir janela):

```sh
python gemeo.py --simular --imagem figura.png --em 17
```

Testes (se não aparecer erro, está tudo certo):

```sh
python test_gemeo.py
```

## Mexer no modelo com comandos

Dentro da janela, clique na caixa **Comando**, escreva e aperte Enter. Os comandos mexem **só no modelo 3D**
(não são enviados ao braço de verdade). Os ângulos são **graus do servo**, a mesma unidade do terminal do
firmware: `ombro 145` no gêmeo e no ESP32 querem dizer a mesma posição.

| Digite | O que acontece |
|---|---|
| `garra 150` | move a garra para 150° |
| `base -30` | gira a base para -30° |
| `ombro 130; cotovelo 70` | vários comandos de uma vez, separados por `;` |
| `neutro` | pose em que o STL foi montado |
| `idle` | pose IDLE do firmware |
| `auto` | volta a seguir o ESP32 (ou o simulador) |
| `ajuda` | mostra a lista de comandos |

Juntas aceitas: `base`, `ombro`, `cotovelo`, `punho_rot` (ou `rotacao`), `punho_flex` (ou `flexao`) e `garra`.
Servos de 180° aceitam de 0 a 180; a base aceita de -180 a 180. Maiúsculas, acentos e vírgula decimal são aceitos.
O primeiro comando liga o **modo manual**: o modelo para de seguir os dados do ESP32 até você digitar `auto`.
Para gerar uma figura com comandos (relatório): `python gemeo.py --simular --imagem figura.png --comando "garra 150" --comando "base 40"`.

## Cuidados com a porta USB

- **Só um programa por vez usa a porta.** Feche o monitor do ESP-IDF (`Ctrl+]`) antes de abrir o gêmeo.
- Abrir a porta pode **reiniciar** o ESP32 (acontece nesta placa). É normal.
- No Linux, se aparecer "Permission denied", o seu usuário precisa estar no grupo `dialout`.

## Arquivos

| Arquivo | Para quê |
|---|---|
| `gemeo.py` | Janela e desenho (programa principal) |
| `braco.py` | Contas da geometria: ângulos dos servos viram pontos e posições 3D |
| `comandos.py` | Entende o que você digita (`garra 150`) |
| `modelo.py` | Desenho do braço com caixas e cilindros (cores do modelo do GrabCAD) |
| `leitor.py` | Lê a porta serial e filtra a telemetria; também tem o simulador |
| `calibracao.py` | Converte os graus do servo (firmware) nos ângulos do modelo 3D |
| `test_gemeo.py` | Testes automáticos |

## O que ainda precisa ser decidido ou medido

1. **Calibração (`calibracao.py`).** O firmware manda na telemetria os ângulos **comandados** (não medidos),
   em graus do servo: `"angles_deg": {"ombro": 145.0, ...}` (10 vezes por segundo enquanto algo se mexe; 2 parado).
   Para cada junta, `neutro` é o grau do servo em que a peça fica igual ao STL e `sinal` diz o sentido. Os valores
   iniciais vêm do pulso neutro do firmware (`joint_pwm`, em `bench_servo.c`) e **precisam ser conferidos no braço**:
   mande o mesmo comando ao ESP32 e ao gêmeo e compare.
   A **base** não tem encoder: o firmware só manda `base_drive_ms` (tempo de giro). Meça quantos graus ela gira
   por segundo e coloque em `BASE_GRAUS_POR_S`; até lá ela fica em 0°.
2. **Medidas do braço.** Os comprimentos no topo de `braco.py` são provisórios. Troque pelas medidas do modelo do GrabCAD.
3. **Sentido dos ângulos.** Foi assumido: base em graus a partir do "grau 0"; nos servos de 180°, o valor 90 deixa a
   peça alinhada com a anterior (braço em pé) e valores maiores inclinam para a frente. Se algum servo for
   no sentido contrário (o cotovelo tem sentido invertido na calibração), é só inverter o sinal em `calcular_pose`.
4. **Modelo 3D real.** O braço é desenhado com formas simples (`modelo.py`), de medidas aproximadas tiradas das fotos do modelo do
   GrabCAD. A pasta `robotic-arm-magnibot-1.snapshot.7` é do SolidWorks (`.SLDPRT`/`.SLDASM`), que o Python não lê. Para usar as peças
   reais é preciso exportá-las para STL no SolidWorks. A geometria das articulações (`calcular_quadros`) já está separada do desenho,
   então trocar as caixas por STL só mexe em `modelo.py`.
5. **Palitos.** `python gemeo.py --simular --esqueleto` desenha também o esqueleto, útil para conferir as articulações.
