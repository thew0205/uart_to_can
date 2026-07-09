#include "uart_to_can_core_utility.h"

#include "zephyr/sys/ring_buffer.h"
#include "zephyr/toolchain.h"
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

#ifdef CONFIG_ZTEST
#define CONFIG_UART_TO_CAN_STATIC
#else
#define CONFIG_UART_TO_CAN_STATIC static
#endif

LOG_MODULE_REGISTER(uart_to_can_core_utility, LOG_LEVEL_INF);

extern struct k_mem_slab filter_id_list_slab;
extern int *filter_ids_ptr_map[CONFIG_CAN_MAX_STD_ID_FILTERS +
                               CONFIG_CAN_MAX_EXT_ID_FILTERS];

void can_rx_callback(const struct device *dev, struct can_frame *frame,
                     void *user_data);

void can_tx_callback(__maybe_unused const struct device *dev, int err,
                     void *user_data);

bool ring_buf_has_wrapped(struct ring_buf *buf) {
  uint32_t free_space = ring_buf_size_get(buf);
  uint8_t *data;
  uint32_t claim_size = ring_buf_get_claim(buf, &data, free_space);
  int err = ring_buf_get_finish(buf, 0);
  __ASSERT(err == 0, "Failed to free ring buffer");
  ARG_UNUSED(err);
  return claim_size < free_space;
}

int search_r_consider_wrap(struct ring_buf *buf) {

  uint8_t *data;
  uint8_t *data_remain;
  bool buf_has_wrapped = ring_buf_has_wrapped(buf);
  uint32_t get_size = ring_buf_size_get(buf);
  uint32_t get_claim_size = ring_buf_get_claim(buf, &data, get_size);
  int ret_val = INT_MIN;
  for (uint32_t i = 0; i < get_claim_size; i++) {
    if (data[i] == '\r') {
      LOG_DBG("Found \\r at index %d", i);
      ret_val = i;
      break;
    }
  }
  if (buf_has_wrapped) {
    uint32_t get_claim_size2 =
        ring_buf_get_claim(buf, &data_remain, get_size - get_claim_size);
    for (uint32_t i = 0; i < get_claim_size2; i++) {
      if (data_remain[i] == '\r') {
        LOG_DBG("Found \\r at index %d", -1 * i);
        ret_val = -1 * i;
        break;
      }
    }
  }
  if (ring_buf_get_finish(buf, 0) != 0) {
    LOG_ERR("Failed to free ring buffer");
  }
  return ret_val;
}

int start_can_device(const struct device *can_dev) {
  int err = 0;
  err = can_start(can_dev);
  if (err != 0) {
    LOG_ERR("Error starting CAN controller (err %d)", err);
  }

  return err;
}

int stop_can_device(const struct device *can_dev) {
  int err = 0;
  err = can_stop(can_dev);
  if (err != 0) {
    LOG_ERR("Error stopping CAN controller (err %d)", err);
  }
  return err;
}

int set_bitrate(const struct device *can_dev, uint8_t bitrate_code) {
  int err;
  struct can_timing timing;
  uint32_t bitrate_k;
  // If the sample point is set to 0, this function defaults to a sample point
  // of 75.0% for bitrates over 800 kbit/s, 80.0% for bitrates over 500 kbit/s,
  // and 87.5% for all other bitrates.
  switch (bitrate_code) {
  case '0':
    bitrate_k = 10;
    break;
  case '1':
    bitrate_k = 20;
    break;
  case '2':
    bitrate_k = 50;
    break;
  case '3':
    bitrate_k = 100;
    break;
  case '4':
    bitrate_k = 125;
    break;
  case '5':
    bitrate_k = 200;
    break;
  case '6':
    bitrate_k = 250;
    break;
  case '7':
    bitrate_k = 500;
    break;
  case '8':
    bitrate_k = 1000;
    break;
  default:
    err = -EINVAL;
    LOG_ERR("Invalid bitrate code: %d", bitrate_code);
    goto set_bitrate_return;
  }
  err = can_calc_timing(can_dev, &timing, bitrate_k * 1000, 0);
  if (err > 0) {
    LOG_INF("Sample-Point error: %d", err);
  }

  if (err < 0) {
    LOG_ERR("Failed to calc timing for speed %d", bitrate_k * 1000);
    goto set_bitrate_return;
  }
  err = can_set_timing(can_dev, &timing);
  if (err != 0) {
    LOG_ERR("Failed to set timing");
    goto set_bitrate_return;
  }

set_bitrate_return:
  return err;
}

struct uart_message can_frame_to_uart_message(const struct can_frame *frame,
                                              int filter_id) {
  struct uart_message message;
  size_t offset = 0;
  int err = 0;
  if (frame->flags & CAN_FRAME_IDE) {
    message.buffer[offset++] = 'R';
    err = snprintf(&message.buffer[offset], 9, "%08x", frame->id);
    __ASSERT(err == 8, "Failed to write CAN ID to UART message buffer");
    offset += 8;
  } else {
    message.buffer[offset++] = 'r';
    err = snprintf(&message.buffer[offset], 4, "%03x", frame->id);
    __ASSERT(err == 3, "Failed to write CAN ID to UART message buffer");
    offset += 3;
  }

  err = snprintf(&message.buffer[offset], 3, "%02x", filter_id);
  __ASSERT(err == 2, "Failed to write CAN filter ID to UART message buffer");
  offset += 2;

  err = snprintf(&message.buffer[offset], 2, "%01x",
                 (uint8_t)(frame->dlc & 0x0F));
  __ASSERT(err == 1, "Failed to write CAN DLC to UART message buffer");
  offset += 1;

  for (size_t i = 0; i < frame->dlc; i++) {
    err = snprintf(&message.buffer[offset], 3, "%02x", frame->data[i]);
    __ASSERT(err == 2, "Failed to write CAN Data to UART message buffer");
    offset += 2;
  }
  message.buffer[offset++] = COMMAND_RESPONSE_OKAY[0];
  message.buffer_size = offset;
  __ASSERT(offset <= MAX_UART_CAN_FRAME, "UART message buffer overflow");
  return message;
}

int send_can_message_no_wait(const struct device *can_dev, uint32_t addr,
                             const uint8_t data[], uint8_t data_size,
                             bool is_extended_id, bool is_rtr) {
  int err = 0;

  if (data_size > CAN_MAX_DLEN) {
    LOG_ERR("Data size is too large");
    err = -1;
    goto send_can_message_return;
  }
  struct can_frame frame;
  frame.id = addr;
  frame.dlc = data_size;
  frame.flags = 0;
  frame.reserved = 0;
  memcpy(frame.data, data, data_size);

  if (is_extended_id) {
    frame.flags |= CAN_FRAME_IDE;
  }

  if (is_rtr) {
    frame.flags |= CAN_FRAME_RTR;
  }

  LOG_DBG("babbling on %s with %s (%d-bit) CAN ID 0x%0*x, RTR %d, CAN FD %d "
          "with data of size %d and buffer contains",
          can_dev->name,
          (frame.flags & CAN_FRAME_IDE) != 0 ? "extended" : "standard",
          (frame.flags & CAN_FRAME_IDE) != 0 ? 29 : 11,
          (frame.flags & CAN_FRAME_IDE) != 0 ? 8 : 3, frame.id,
          (frame.flags & CAN_FRAME_RTR) != 0 ? 1 : 0,
          (frame.flags & CAN_FRAME_FDF) != 0 ? 1 : 0, data_size);
  LOG_HEXDUMP_DBG(data, data_size, "My CAN Data Buffer:%s");

  err = can_send(can_dev, &frame, K_NO_WAIT, can_tx_callback,
                 is_extended_id ? "T" : "t");
  if (err != 0) {
    LOG_ERR("Failed to enqueue CAN frame (err %d)", err);
    // TODO (Matthew): What could cause CAN to fail
    goto send_can_message_return;
  }

send_can_message_return:
  return err;
}

int add_can_filter(const struct device *can_dev, uint32_t filter_id,
                   uint32_t filter_mask, bool is_extended_id) {
  const struct can_filter filter = {
      .id = filter_id,
      .mask = filter_mask,
      .flags = is_extended_id ? CAN_FILTER_IDE : 0,

  };

  int *temp_filter_id_ptr;
  int err = k_mem_slab_alloc(&filter_id_list_slab, (void **)&temp_filter_id_ptr,
                             K_NO_WAIT);
  if (err != 0) {
    LOG_ERR("Failed to allocate filter (err: %d)", err);
    goto add_can_filter_return;
  }

  err =
      can_add_rx_filter(can_dev, can_rx_callback, temp_filter_id_ptr, &filter);

  if (err < 0) {
    k_mem_slab_free(&filter_id_list_slab, temp_filter_id_ptr);
    LOG_ERR("Failed to bind CAN hardware filter (err: %d)", err);
    goto add_can_filter_return;
  }
  *temp_filter_id_ptr = err;
  filter_ids_ptr_map[err] = temp_filter_id_ptr;
add_can_filter_return:
  return err;
}

int remove_can_filter(const struct device *can_dev, int filter_id) {
  int err = 0;
  if (filter_id >=
      (CONFIG_CAN_MAX_STD_ID_FILTERS + CONFIG_CAN_MAX_EXT_ID_FILTERS)) {
    LOG_ERR("Invalid CAN filter ID");
    err = -1;
    goto remove_can_filter_return;
  }

  if (filter_ids_ptr_map[filter_id] == NULL) {
    LOG_ERR("Invalid CAN filter ID");
    err = -1;
    goto remove_can_filter_return;
  }

  can_remove_rx_filter(can_dev, filter_id);
  k_mem_slab_free(&filter_id_list_slab, filter_ids_ptr_map[filter_id]);
  filter_ids_ptr_map[filter_id] = NULL;
remove_can_filter_return:
  return err;
}

void clear_buf_till_r(struct ring_buf *buf) {
  uint32_t buf_size = ring_buf_size_get(buf);
  for (uint32_t i = 0; i < buf_size; i++) {
    if (ring_buf_get_char(buf) == '\r') {
      break;
    }
  }
}
