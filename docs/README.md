# MagniBot — bancada

Plano, lacunas, marcos e registro de testes: [exec.md](../../magnibot_agent_configurations/exec.md).

Firmware somente de bancada: não inicializa PWM nem libera servos. Cliques são detectados e recusados para operação. Telemetria JSON a cada 500 ms usa o console UART 115200; logs compartilham o canal. Filtre objetos com `type=telemetry` e `version=1`. Campos `null` não são medições nem posição zero.

Com o ambiente ESP-IDF 6.1 ativo:

```sh
./tools/build.sh
./tests/run.sh
```

Saída: `build/bench/magnibot_firmware.bin`. Gravação, boot, telemetria e botão validados na placa em 07/11/2026; LED verde confirmado pelo usuário. Veja [resultados e limitações](verification.md).

Emergência física desabilitada por padrão (`CONFIG_MAGNIBOT_ESTOP_GPIO=-1`). Depois de validar placa e ligação, configure em `idf.py -B build/bench menuconfig` → MagniBot bancada. Ativa baixa, pull-up interno, latch até reinicialização. Não é corte de potência nem validação do requisito de 10 µs. Não ligar atuadores nesta etapa.
