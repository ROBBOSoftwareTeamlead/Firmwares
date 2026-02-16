# Прошивка коптера (Crazyflie 2.x)

Исходники прошивки **Bitcraze Crazyflie** для платформ Crazyflie 2.x, Bolt, Roadrunner (Tag). Версия в этой папке: **2023.07**.

- **Репозиторий:** [bitcraze/crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware)  
- **Лицензия:** LGPL-3.0 (см. `LICENSE.txt`)

## Платформа

- МК: STM32F405 (ARM Cortex-M4)
- ОСРВ: FreeRTOS
- Связь с ПК: протокол CRTP по радио (Crazyradio) или USB

## Сборка и прошивка

- Требуется тулчейн ARM (arm-none-eabi-gcc) и окружение для сборки (см. [официальную документацию](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/)).
- Сборка: `make` с нужным target (например, `cf2`, `bolt`, `tag`).
- Прошивка: через [cfclient](https://github.com/bitcraze/crazyflie-clients-python) или скрипты в `tools/`.

Подробности: [Building and flashing](https://github.com/bitcraze/crazyflie-firmware/tree/master/docs/building-and-flashing) в папке `docs/`.

## Готовые релизы

Готовые zip для прошивки без сборки: [crazyflie-release](https://github.com/bitcraze/crazyflie-release) (ветка/тег 2023.07).

## Связь с Robbo Scratch

Управление коптером из Robbo Scratch идёт по CRTP (порты Setpoint, Param, Log). Реализация на стороне клиента — в проекте RobboScratch3 (QuadcopterControlAPI, crazyradio).
