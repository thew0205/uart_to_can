#ifndef __UART_TO_CAN_H__
#define __UART_TO_CAN_H__

#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include "uart_to_can_core_utility.h"
/* Enum definition from supported commands. */
enum UART_CAN_COMMANDS {
  UART_CAN_COMMANDS_START_CAN = 'O',
  UART_CAN_COMMANDS_STOP_CAN = 'C',
  UART_CAN_COMMANDS_SET_BITRATE = 'S',
  UART_CAN_COMMANDS_SEND_11_BIT_CAN = 't',
  UART_CAN_COMMANDS_SEND_29_BIT_CAN = 'T',
  UART_CAN_COMMANDS_VERSION = 'V',
  UART_CAN_COMMANDS_HELP = 'h',
  UART_CAN_COMMANDS_RESET = 'L',
  UART_CAN_COMMANDS_GET_STATE = 'G',
  UART_CAN_COMMANDS_ADD_FILTER_11_BIT = 'm',
  UART_CAN_COMMANDS_ADD_FILTER_29_BIT = 'M',
};

/**
 * @brief Initialize the UART to CAN module.
 *
 * @return 0 if successful, <0 on error.
 */
int init_uart_to_can(void);
int send_uart_data_to_dev(const struct uart_message *message);
#endif /* __UART_TO_CAN_H__ */
