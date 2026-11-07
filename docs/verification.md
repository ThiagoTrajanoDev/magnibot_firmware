# Verificação M1 — 07/11/2026

| Verificação | Resultado |
|---|---|
| ESP-IDF 6.1.0, ESP32-S3, GCC Xtensa 15.2.0, bancada padrão | PASS — 214688 bytes |
| Variante de compilação com entrada ISR habilitada | PASS — 216240 bytes |
| Testes C nativos com `-Wall -Wextra -Werror` e UBSan | PASS |
| `git diff --check` | PASS |
| Flash, botão/LED/ISR físicos, ADC e sensores | Não executados |
| Servos, corte PWM ≤10 µs e segurança mecânica | Não implementados/validados |

Testes cobrem ciclo operacional da máquina de estados, ERROR retido em todos os estados, bloqueio dos comandos de bancada, retorno de falha GPIO, argumentos inválidos, debounce, cliques simples/duplo/longo e cancelamento do clique pendente durante segundo toque longo.

A variante ISR existe apenas como evidência de compilação. O binário padrão é `build/bench/magnibot_firmware.bin`, com emergência desconectada e sem qualquer inicialização de PWM. GPIO17 usado na variante não está aprovado para ligação.

Detalhes e roteiro físico em [exec.md](../../magnibot_agent_configurations/exec.md).

## Atualização: validação real na placa

Em 07/11/2026, após confirmação do módulo ESP32-S3-N16R8/dev-kit e botão GPIO15, firmware M1 gravado em `/dev/ttyACM0`; hashes de escrita verificados pelo esptool. Esta atualização substitui o status "não executado" anterior somente para gravação, boot e telemetria.

Captura de 12 segundos: 24 frames JSON; período 500–504 ms; todos IDLE/bancada, movimento desabilitado e emergência não configurada; sensores/ângulos/abertura null. Nenhum panic, brownout ou erro de inicialização identificado nesse intervalo. Evidências em [captura serial](hardware/2026-11-07-bench-serial.txt) e [resultado](hardware/2026-11-07-bench-result.json).

Flash configurada para 2 MiB apesar da capacidade maior do módulo, preservando o particionamento atual; PSRAM não utilizada. Revisar configuração antes de ampliar recursos. Nenhum servo recebeu PWM.

Teste físico adicional com participação do usuário: botão GPIO15 reconheceu exatamente um clique simples, um duplo e uma pressão longa. Os 180 frames da captura de 90 segundos permaneceram IDLE, com movimento desabilitado. LED verde confirmado pelo usuário; demais cores não validadas. [Evidência dos cliques](hardware/2026-11-07-button-serial.txt). Emergência física e ADC continuam pendentes.

## Fechamento para commits

Testes separados em `tests/test_button.c` e `tests/test_state.c`, ambos executados por `tests/run.sh`: PASS. Build repetido após separação: PASS, 214688 bytes. Nenhuma nova gravação; evidências físicas referem-se ao binário gravado durante o teste anterior. Builds posteriores podem alterar metadados de versão Git e hash sem alterar o comportamento testado.

Potenciômetro e LEDs externos são recursos opcionais de bancada, conforme esclarecimento do usuário, e não requisitos do produto final. Próximo marco: protocolo serial bidirecional e emergência remota.

Capturas textuais versionadas com finais de linha normalizados para LF e espaços finais removidos; conteúdo dos registros preservado.
