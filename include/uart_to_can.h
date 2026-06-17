#ifndef __UART_TO_CAN_H__
#define __UART_TO_CAN_H__

#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include "uart_to_can_core_utility.h"

/**
 * @brief Initialize the UART to CAN module.
 *
 * @return 0 if successful, <0 on error.
 */
int init_uart_to_can(void);
int send_uart_data_to_dev(const struct uart_message *message);
#endif /* __UART_TO_CAN_H__ */
