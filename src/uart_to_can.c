#include "uart_to_can.h"

#include "zephyr/sys/ring_buffer.h"
#include "zephyr/toolchain.h"
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/uart.h>

#include "uart_to_can_core_utility.h"

#ifdef CONFIG_ZTEST
#define CONFIG_UART_TO_CAN_STATIC
#else
#define CONFIG_UART_TO_CAN_STATIC static
#endif

LOG_MODULE_REGISTER(uart_to_can, LOG_LEVEL_INF);

#define CAN_NODE DT_ALIAS(uart_can_can)
static const struct device *can_dev = DEVICE_DT_GET(CAN_NODE);

#define UART_NODE DT_ALIAS(uart_can_uart)
static const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);

#define DMA_BUF_SIZE 32
#define RX_BUF_SIZE 1024
/* 2. Declare Memory Buffers */
static uint8_t dma_buf_a[DMA_BUF_SIZE];
static uint8_t dma_buf_b[DMA_BUF_SIZE];

RING_BUF_DECLARE(rx_ring_buffer, RX_BUF_SIZE);

K_MSGQ_DEFINE(uart_message_msgq, sizeof(struct uart_message), 10, 1);
static void uart_rx_reset_buffer_timer_fn(struct k_timer *timer_id) {
  ARG_UNUSED(timer_id);
  // Resetting the buffer is allowed here becasue only unprocessed command will
  // be in the buffer at this point.
  ring_buf_reset(&rx_ring_buffer);
}
K_TIMER_DEFINE(uart_rx_reset_buffer_timer, uart_rx_reset_buffer_timer_fn, NULL);

CONFIG_UART_TO_CAN_STATIC int
parse_and_send_can_message_no_wait(struct ring_buf *const buf,
                                   const uint32_t can_id, bool is_extended_id) {
  int err;
  unsigned long temp_long;
  uint8_t dlc;
  uint8_t can_data[8];
  if (ring_buf_size_get(buf) < 1) {
    err = -1;
    LOG_ERR("Not enough data in buffer");
    goto parse_and_send_can_message_no_wait_11bit_return;
  }
  if (ring_buf_get_char_to_uint(buf, 1, 16, &temp_long)) {
    dlc = temp_long;
  } else {
    err = -1;
    LOG_ERR("Invalid dlc");
    goto parse_and_send_can_message_no_wait_11bit_return;
  }
  if (ring_buf_size_get(buf) < 2 * dlc) {
    err = -1;
    LOG_ERR("Not enough data in buffer");
    goto parse_and_send_can_message_no_wait_11bit_return;
  }
  for (uint32_t i = 0; i < dlc; i++) {
    if (!ring_buf_get_hex_to_uint8_t(buf, &can_data[i])) {
      err = -1;
      LOG_ERR("Invalid data");
      goto parse_and_send_can_message_no_wait_11bit_return;
    }
  }
  err = send_can_message_no_wait(can_dev, can_id, can_data, dlc, is_extended_id,
                                 false);
parse_and_send_can_message_no_wait_11bit_return:
  return err;
}

CONFIG_UART_TO_CAN_STATIC int
parse_and_send_can_message_no_wait_11bit(struct ring_buf *const buf) {
  int err = 0;
  unsigned long temp_long;
  uint16_t can_id = 0;
  if (ring_buf_size_get(buf) < (CAN_ID_11_BIT_BYTE_LENGHT + 1)) {
    err = -1;
    LOG_ERR("Not enough data in buffer");
    goto parse_and_send_can_message_no_wait_11bit_return;
  }
  if (ring_buf_get_char_to_uint(buf, CAN_ID_11_BIT_BYTE_LENGHT, MESSAGE_BASE,
                                &temp_long)) {
    can_id = temp_long;
  } else {
    err = -1;
    LOG_ERR("Invalid CAN ID");
    goto parse_and_send_can_message_no_wait_11bit_return;
  }
  err = parse_and_send_can_message_no_wait(buf, can_id, false);
parse_and_send_can_message_no_wait_11bit_return:
  return err;
}

CONFIG_UART_TO_CAN_STATIC int
parse_and_send_can_message_no_wait_29bit(struct ring_buf *const buf) {
  int err = 0;
  unsigned long temp_long;
  uint32_t can_id = 0;
  if (ring_buf_size_get(buf) < (CAN_ID_29_BIT_BYTE_LENGHT + 1)) {
    err = -1;
    LOG_ERR("Not enough data in buffer");
    goto parse_and_send_can_message_no_wait_11bit_return;
  }
  if (ring_buf_get_char_to_uint(buf, CAN_ID_29_BIT_BYTE_LENGHT, MESSAGE_BASE,
                                &temp_long)) {
    can_id = temp_long;
  } else {
    err = -1;
    LOG_ERR("Invalid CAN ID");
    goto parse_and_send_can_message_no_wait_11bit_return;
  }
  err = parse_and_send_can_message_no_wait(buf, can_id, true);
parse_and_send_can_message_no_wait_11bit_return:
  return err;
}

enum UART_CAN_COMMANDS {
  START_CAN = 'O',
  STOP_CAN = 'C',
  SET_BITRATE = 'S',
  SEND_11_BIT_CAN = 't',
  SEND_29_BIT_CAN = 'T',
  VERSION = 'V',
  HELP = 'h',
};

void print_buffer_without_clearing(struct ring_buf *buf) {
  uint8_t buffer[500];
  size_t bytes_read = ring_buf_peek(buf, buffer, 500);
  LOG_HEXDUMP_INF(buffer, bytes_read, "Dumping :");
}
/**
 * @brief Processes data from the UART ring buffer
 *
 * @param buf Pointer to the ring buffer
 */
CONFIG_UART_TO_CAN_STATIC void process_data_uart_data(struct ring_buf *buf) {
  int err = -1;
  const char *command_response;
  struct uart_message uart_message;
  print_buffer_without_clearing(buf);
  if (search_r_consider_wrap(buf) != INT_MIN) {
    uint8_t command = ring_buf_get_char(buf);
    switch (command) {

    case START_CAN:
      LOG_INF("Starting CAN");
      err = start_can_device(can_dev);
      command_response =
          err == 0 ? COMMAND_RESPONSE_OKAY : COMMAND_RESPONSE_ERROR;
      uart_message = string_to_uart_message(command_response);
      err = send_command_status_via_uart(&uart_message);
      break;
    case STOP_CAN:
      LOG_INF("Stopping CAN");
      err = stop_can_device(can_dev);
      command_response =
          err == 0 ? COMMAND_RESPONSE_OKAY : COMMAND_RESPONSE_ERROR;
      uart_message = string_to_uart_message(command_response);
      err = send_command_status_via_uart(&uart_message);
      break;
    case SET_BITRATE:
      LOG_INF("Setting CAN Bitrate");
      err = set_bitrate(can_dev, ring_buf_get_char(buf));
      command_response =
          err == 0 ? COMMAND_RESPONSE_OKAY : COMMAND_RESPONSE_ERROR;
      uart_message = string_to_uart_message(command_response);
      err = send_command_status_via_uart(&uart_message);
      break;
    case 's':
      // TODO (Matthew)
      break;
    case SEND_11_BIT_CAN:
      LOG_INF("Send data to standard 11 bit CAN");
      err = parse_and_send_can_message_no_wait_11bit(buf);
      command_response =
          err == 0 ? COMMAND_RESPONSE_OKAY : COMMAND_RESPONSE_ERROR;
      uart_message = string_to_uart_message(command_response);
      err = send_command_status_via_uart(&uart_message);
      break;
    case SEND_29_BIT_CAN:
      LOG_INF("Send data to standard 29 bit CAN");
      err = parse_and_send_can_message_no_wait_29bit(buf);
      command_response =
          err == 0 ? COMMAND_RESPONSE_OKAY : COMMAND_RESPONSE_ERROR;
      uart_message = string_to_uart_message(command_response);
      err = send_command_status_via_uart(&uart_message);
      break;
    case VERSION:
      LOG_INF("Version");
      err = send_version(uart_dev);
      break;
    case HELP:
      LOG_INF("Help");
      // TODO (Matthew)
      break;
    default:
      LOG_ERR("Unrecongnised command");
      err = -ENOENT;
      command_response =
          err == 0 ? COMMAND_RESPONSE_OKAY : COMMAND_RESPONSE_ERROR;
      uart_message = string_to_uart_message(command_response);
      err = send_command_status_via_uart(&uart_message);
      break;
    }
    clear_buf_till_r(buf);
  }
}

int copy_data_to_ring_buf(struct ring_buf *ring_buf, const uint8_t *const data,
                          size_t data_len) {
  uint8_t *temp_data;
  int err;
  uint32_t bytes_written2 = 0;

  uint32_t bytes_written = ring_buf_put_claim(ring_buf, &temp_data, data_len);
  memcpy(temp_data, data, bytes_written);
  if (ring_buf_put_finish(ring_buf, bytes_written) != 0) {
    err = -1;
    goto copy_data_to_ring_buf_return;
  }

  if (bytes_written < data_len && ring_buf_size_get(ring_buf) > 0) {
    bytes_written2 =
        ring_buf_put_claim(ring_buf, &temp_data, data_len - bytes_written);
    memcpy(temp_data, &data[bytes_written], bytes_written2);
    if (ring_buf_put_finish(ring_buf, bytes_written) != 0) {
      err = -1;
      goto copy_data_to_ring_buf_return;
    }
  }
  err = bytes_written + bytes_written2;

copy_data_to_ring_buf_return:
  return err;
}

extern struct k_mem_slab my_slab;

CONFIG_UART_TO_CAN_STATIC void uart_cb(const struct device *dev,
                                       __maybe_unused struct uart_event *evt,
                                       __maybe_unused void *user_data) {
  switch (evt->type) {

  case UART_TX_DONE:
    // do something
    LOG_INF("UART event UART_TX_DONE type: %d\n", evt->type);
// What you derived
    struct uart_message *msg = (struct uart_message *)evt->data.tx.buf;
    LOG_INF("Derived Struct Address: %p", (void *)msg);

    // STOP HERE: Compare these printed addresses to the addresses 
    // generated during k_mem_slab_alloc in your transmission function.
    // If they do not match exactly, DO NOT call k_mem_slab_free yet.
    k_mem_slab_free(&my_slab, msg);
    break;

  case UART_TX_ABORTED:
    // do something
    LOG_INF("UART event UART_TX_ABORTED type: %d\n", evt->type);

    break;

  case UART_RX_RDY:

    LOG_INF("UART event UART_RX_RDY type: %d\n", evt->type);
    int bytes_copied = copy_data_to_ring_buf(
        &rx_ring_buffer, &evt->data.rx.buf[evt->data.rx.offset],
        evt->data.rx.len);
    if (bytes_copied < 0) {
      LOG_ERR("Error with copying data to ring buf");

    } else {
      // Perform complete commands possible
      process_data_uart_data(&rx_ring_buffer);
      // clear command that might have possible values dropped.
      if ((size_t)bytes_copied < evt->data.rx.len) {
        LOG_ERR("Ring buffer full! Dropped %d bytes.",
                (evt->data.rx.len - bytes_copied));
        clear_buf_till_r(&rx_ring_buffer);
        LOG_INF("Clearing ring buffer");
        // TODO (Matthew): Send error
      }
    }
    clear_buf_till_r(&rx_ring_buffer);

    k_timer_start(&uart_rx_reset_buffer_timer, K_MSEC(1000), K_FOREVER);
    break;

  case UART_RX_BUF_REQUEST:
    LOG_INF("UART event UART_RX_BUF_REQUEST type: %d\n", evt->type);

    if (evt->data.rx_buf.buf == dma_buf_a) {
      uart_rx_buf_rsp(dev, dma_buf_b, DMA_BUF_SIZE);
    } else {
      uart_rx_buf_rsp(dev, dma_buf_a, DMA_BUF_SIZE);
    }

    break;

  case UART_RX_BUF_RELEASED:
    // do something
    LOG_INF("UART event UART_RX_BUF_RELEASED type: %d\n", evt->type);

    break;

  case UART_RX_DISABLED:
    // do something
    LOG_INF("UART event UART_RX_DISABLED type: %d\n", evt->type);
    break;

  case UART_RX_STOPPED:
    // do something
    LOG_INF("UART event UART_RX_STOPPED type: %d\n", evt->type);
    break;

  default:
    break;
  }
}

int init_uart_to_can(void) {
  int err = 0;
  if (!device_is_ready(uart_dev)) {
    LOG_ERR("UART device not ready yet!");
    return -EIO;
  }

  if (!device_is_ready(can_dev)) {
    LOG_ERR("CAN device not ready yet!");
    return -EIO;
  }

  /* Register the async interrupt handler */
  err = uart_callback_set(uart_dev, uart_cb, (void *)uart_dev);

  if (err) {
    LOG_ERR("Failed to register UART callback (err %d)", err);
    return err;
  }

  err = uart_rx_enable(uart_dev, dma_buf_a, DMA_BUF_SIZE, 20);
  if (err) {
    LOG_ERR("Failed to enable UART reception (err %d)", err);
    return err;
  }

  return err;
}

int send_can_message_to_uart(const struct can_frame *frame) {
  struct uart_message message = can_frame_to_uart_message(frame);

  return send_command_status_via_uart(&message);
}

int send_uart_data_to_dev(const struct uart_message *message) {
  int err = uart_tx(uart_dev, message->buffer, message->buffer_size,
                    500 * message->buffer_size);
  return err;
}