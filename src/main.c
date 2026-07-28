#include "uart_to_can.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/uart.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
int main(void)
{

  int err;
  err = init_uart_to_can();
  if (err < 0)
  {
    LOG_ERR("Failed to init uart to can");
    return 0;
  }

  if (!gpio_is_ready_dt(&led))
  {
    LOG_ERR("LED not ready");
  }

  err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
  if (err < 0)
  {
    LOG_WRN("Failed to indicator LED");
  }

  while (1)
  {

    err = gpio_pin_toggle_dt(&led);
    k_msleep(2000);
  }
}
