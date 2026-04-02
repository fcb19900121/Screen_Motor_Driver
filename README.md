# Screen Motor Driver

Firmware for an STM32F103RC-based dual DC motor controller that drives a motorized screen rotation mechanism (portrait/landscape) and a linear actuator for pitch control.

## Hardware

| Item | Detail |
|------|--------|
| MCU | STM32F103RC (Cortex-M3, 72 MHz) |
| Motor driver | BTN9970 half-bridge × 2 (full-bridge per motor) |
| Motor 1 | DC brushed motor — screen rotation (portrait ↔ landscape), gear ratio 1:1400 |
| Motor 2 | DC brushed linear actuator — pitch axis, gear ratio 1:1000 |
| Encoder | AB quadrature encoder, 11 lines × 4× = 44 PPR |
| Limit switches | LIMIT1 (portrait), LIMIT2 (landscape) via GPIO EXTI |
| USB | USB 2.0 Full Speed, CDC ACM (virtual COM port) |
| IDE | Keil MDK-ARM |

## Firmware Architecture

```
Core/           — STM32CubeMX generated HAL init (main.c, IRQ handlers)
MotorLib/       — Low-level drivers
  motor_driver  — TIM8 PWM generation (20 kHz), BTN9970 enable/brake
  hall_sensor   — Quadrature encoder counting & speed calculation
  current_sensor— Motor current monitoring via ADC
MotorApp/       — Application logic
  motor_app     — Homing FSM, position control (P-controller), screen orientation state
  usb_cmd       — USB CDC command parser
  eeprom_emul   — Flash-based EEPROM emulation (persistent orientation state)
USB_DEVICE/     — STM32 USB CDC middleware
```

## USB Command Protocol

Connection: USB CDC ACM (virtual COM port, any baud rate). Commands are ASCII, terminated with `\r\n`.

Full protocol reference: [`docs/USB_Protocol.md`](docs/USB_Protocol.md)

### Quick Reference

| Command | Example | Description |
|---------|---------|-------------|
| `PITCH:<0-100>` | `PITCH:50\r\n` | Move linear actuator to % of full stroke (requires homing complete) |
| `SCREEN:PORTR` | `SCREEN:PORTR\r\n` | Rotate screen to portrait (LIMIT1) |
| `SCREEN:LANDS` | `SCREEN:LANDS\r\n` | Rotate screen to landscape (LIMIT2) |

### Pitch Axis Details

- Range: encoder counts 0 – 40960 (mapped from 0 – 100%)
- Homing runs automatically on power-up; times out after 10 s → FAULT state
- Position control: proportional (Kp = 8), dead-band ±20 counts

## Building

Open `MDK-ARM/Motor_Driver.uvprojx` in Keil µVision and build the `Motor_Driver` target.

## License

STM32 HAL / CMSIS drivers are provided under their respective ST and ARM licenses (see `Drivers/*/LICENSE.txt`).
