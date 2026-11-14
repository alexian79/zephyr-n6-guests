# zephyr-n6-guests

Zephyr RTOS guest applications for the QEMU `stm32n6570-dk` co-sim
machine (see the umbrella workspace repo for full setup).

| App | Drives |
|---|---|
| `transport/` | sheet-transport plant: TIM2 CH1 / TIM5 CH1 PWM domains, GPIOD 0..3 DIR/EN, GPIOD 4..7 IN/S1/S2/EXIT sensors — the Zephyr port of the FreeRTOS `transport-n6` controller, same event log format |
| `motor/` | PWM motor plant: TIM2 CH2 drive, raw-register TIM3 tach capture, GPIOD DIR/EN, ADC1 AIN0 current sense |

Build (west workspace root must be `$HOME`; needs upstream Zephyr
`v4.3.0` + modules `hal_stm32 cmsis cmsis_6`, `arm-none-eabi-gcc`):

```bash
cd ~ && PATH=$HOME/bin:$PATH ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb \
  GNUARMEMB_TOOLCHAIN_PATH=/usr ZEPHYR_BASE=$HOME/zephyr \
  ~/zenv/bin/west build -b stm32n6570_dk \
  -d ~/build/zephyr-transport ~/zephyr-n6-guests/transport
```

Run the images with the launchers from the cosim repo
(`run-n6-zephyr-transport.sh`, `run-n6-zephyr-motor.sh`).

Board-DT traps encoded in the sources/overlays: `GPIO_INPUT` (there is
no `GPIO_IN`), `GPIO_DISCONNECTED` = analog which the QEMU GPIO model
deliberately does not feed into IDR, `pwm_stm32` channels are 1-based,
enabling a timers node requires the binding-required `st,prescaler`,
and `CONFIG_LOG=y` is needed for `LOG_INF` to exist.
