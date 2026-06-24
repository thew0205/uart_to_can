#include "uart_to_can.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/uart.h>

#include "uart_to_can_core_utility.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
extern struct k_msgq uart_message_msgq;
K_MEM_SLAB_DEFINE(my_slab, sizeof(struct uart_message), 15, 4);
int main(void) {

  int err;
  err = init_uart_to_can();
  if (err < 0) {
    LOG_ERR("Failed to init uart to can");
    return 0;
  }

  if (!gpio_is_ready_dt(&led)) {
    LOG_ERR("LED not ready");
    return 0;
  }

  err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
  if (err < 0) {
    LOG_WRN("Failed to indicator LED");
  }

  bool led_state = true;
  struct uart_message message;
  while (1) {

    err = k_msgq_get(&uart_message_msgq, &message, K_FOREVER);

    struct uart_message *block_ptr;
    err = k_mem_slab_alloc(&my_slab, (void **)&block_ptr, K_NO_WAIT);

    if (err != 0) {
      LOG_ERR("Failed to get event from msgq");
      continue;
    }
    block_ptr->buffer_size = message.buffer_size;
    memcpy(block_ptr->buffer, message.buffer, message.buffer_size);

    LOG_INF("Recieve uart message with len %d", block_ptr->buffer_size);

    LOG_INF("%.*s", (int)block_ptr->buffer_size, block_ptr->buffer);

    err = send_uart_data_to_dev(block_ptr);
    if (err != 0) {
      LOG_ERR("Failed to send data to uart dev");
    }

    err = gpio_pin_toggle_dt(&led);
    if (err < 0) {
      return 0;
    }

    led_state = !led_state;
    // LOG_INF("LED state: %s\n", led_state ? "ON" : "OFF");

    // err = uart_tx(uart_dev, "Hello\n", 7, 100);
    // if (err < 0) {
    //   LOG_ERR("Failed to send message to UART");
    //   return 0;
    // }
    // uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf), 100);
    // k_msleep(2000);
  }
}
