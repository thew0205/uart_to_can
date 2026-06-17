# UART to CAN Bridge

A Zephyr RTOS firmware module that implements a **UART-to-CAN bridge**, translating serial commands (based on the [LAWICEL CAN232 protocol](https://ostrovni-elektrarny.cz/static-content/documents/Lawicell_can232_v3.pdf?srsltid=AfmBOopETLooJFBtfLHFFEZLCtcwBkRXRtW8PBn7YZgROGUoGJqEalxK)) into CAN bus frames and vice versa.

---

## Overview

This firmware runs on an STM32-based board and acts as a transparent bridge between a host system (connected via UART) and a CAN bus network. Commands are received over UART, parsed, and executed on the CAN bus. CAN frames received from the bus are serialised and forwarded back to the host over UART.

The project follows a **test-driven development (TDD)** approach using Zephyr's native simulator (`native_sim`) target.

---

## Features

- Start / stop the CAN bus
- Set CAN bus bitrate
- Send CAN frames (standard 11-bit and extended 29-bit IDs)
- Receive CAN frames and forward to UART
- Visual status indicator via on-board LED
- Strict compiler warnings enforced (`-Werror -Wall -Wextra -Wshadow`)

---

## Project Structure

```
uart_to_can/
├── src/
│   ├── main.c                      # Application entry point
│   ├── uart_to_can.c               # UART command parsing & CAN bridge logic
│   └── uart_to_can_core_utility.c  # Low-level CAN/UART utility functions
├── include/
│   └── uart_to_can_core_utility.h  # Core types, constants, and function declarations
├── tests/
│   ├── uart_to_can/                # Integration tests for the bridge logic
│   └── uart_to_can_core_utility/   # Unit tests for core utility functions
├── boards/                         # Board-specific overlays / config
├── prj.conf                        # Zephyr project configuration
├── CMakeLists.txt                  # Build system configuration
└── west.yml                        # West workspace manifest
```

---

## Requirements

- [Zephyr SDK](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) (v3.x or later)
- [West](https://docs.zephyrproject.org/latest/develop/west/index.html) build tool
- CMake ≥ 3.20
- A compatible STM32 board (with CAN peripheral and UART)

---

## Building

### Hardware Target

```bash
west build -b <your_board> .
west flash
```

Replace `<your_board>` with your board identifier (e.g. `nucleo_g474re`).

### Native Simulator (for testing)

```bash
west build -b native_sim . -t run
```

---

## Running Tests

Tests are located in the `tests/` directory and target the `native_sim` platform.

```bash
# Run core utility unit tests
west build -b native_sim tests/uart_to_can_core_utility -t run

# Run bridge integration tests
west build -b native_sim tests/uart_to_can -t run
```

---

## Configuration

Key Zephyr Kconfig options defined in `prj.conf`:

| Option | Value | Description |
|--------|-------|-------------|
| `CONFIG_LOG` | `y` | Enable Zephyr logging subsystem |
| `CONFIG_SERIAL` | `y` | Enable UART serial driver |
| `CONFIG_UART_ASYNC_API` | `y` | Use async UART API |
| `CONFIG_UART_USE_RUNTIME_CONFIGURE` | `y` | Allow runtime UART reconfiguration |
| `CONFIG_CAN` | `y` | Enable CAN driver |
| `CONFIG_MAIN_STACK_SIZE` | `4096` | Main thread stack size (bytes) |
| `CONFIG_DEBUG_OPTIMIZATIONS` | `y` | Enable debug-friendly optimisations |

---

## Protocol Reference

This firmware implements a subset of the **LAWICEL CAN232 / CANUSB** serial protocol. Commands are ASCII strings terminated with `\r`.

| Command | Description |
|---------|-------------|
| `Sn`    | Set CAN bitrate (`n` = bitrate code) |
| `O`     | Open / start CAN bus |
| `C`     | Close / stop CAN bus |
| `tIIILDD...` | Send standard (11-bit) CAN frame |
| `TIIIIIIILLDD...` | Send extended (29-bit) CAN frame |
| `V`     | Query hardware/software version |

Responses: `\r` on success, `\a` (bell) on error.

See the [LAWICEL CAN232 protocol specification](https://ostrovni-elektrarny.cz/static-content/documents/Lawicell_can232_v3.pdf?srsltid=AfmBOopETLooJFBtfLHFFEZLCtcwBkRXRtW8PBn7YZgROGUoGJqEalxK) for full details.

---

## Version

| | Major | Minor |
|---|---|---|
| **Hardware** | 0 | 1 |
| **Software** | 0 | 1 |
