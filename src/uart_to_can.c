#include "uart_to_can.h"

#include "zephyr/sys/__assert.h"
#include "zephyr/sys/ring_buffer.h"
#include "zephyr/toolchain.h"
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

// int filter_ids_std[CONFIG_CAN_MAX_STD_ID_FILTERS];
// int filter_ids_ext[CONFIG_CAN_MAX_EXT_ID_FILTERS];

#define DMA_BUF_SIZE 32
#define RX_BUF_SIZE 1024
/* 2. Declare Memory Buffers */
static uint8_t dma_buf_a[DMA_BUF_SIZE];
static uint8_t dma_buf_b[DMA_BUF_SIZE];

struct UartProcessWork
{
  struct k_work work;
  bool buffer_overflowed;
};

static struct UartProcessWork uart_process_work;

K_MEM_SLAB_DEFINE(filter_id_list_slab, sizeof(int),
                  CONFIG_CAN_MAX_STD_ID_FILTERS + CONFIG_CAN_MAX_EXT_ID_FILTERS,
                  4);

int *filter_ids_ptr_map[CONFIG_CAN_MAX_STD_ID_FILTERS +
                        CONFIG_CAN_MAX_EXT_ID_FILTERS];

RING_BUF_DECLARE(rx_ring_buffer, RX_BUF_SIZE);

K_MEM_SLAB_DEFINE(uart_message_slab, sizeof(struct uart_message), 15, 4);
K_FIFO_DEFINE(uart_message_fifo);

int send_command_status_via_uart_handle_fifo(struct uart_message *message);

static void uart_rx_reset_buffer_timer_fn(struct k_timer *timer_id)
{
  ARG_UNUSED(timer_id);
  // Resetting the buffer is allowed here becasue only unprocessed command will
  // be in the buffer at this point.
  if (ring_buf_size_get(&rx_ring_buffer) > 0)
  {
    LOG_INF("Resetting buffer size: %d", ring_buf_size_get(&rx_ring_buffer));
    ring_buf_reset(&rx_ring_buffer);
  }
}
K_TIMER_DEFINE(uart_rx_reset_buffer_timer, uart_rx_reset_buffer_timer_fn, NULL);

CONFIG_UART_TO_CAN_STATIC int
parse_and_send_can_message_no_wait(struct ring_buf *const buf,
                                   const uint32_t can_id, bool is_extended_id)
{
  int err;
  unsigned long temp_long;
  uint8_t dlc;
  uint8_t can_data[8];
  if (ring_buf_size_get(buf) < 1)
  {
    err = -1;
    LOG_ERR("Not enough data in buffer");
    goto parse_and_send_can_message_no_wait_11bit_return;
  }
  if (ring_buf_get_char_to_uint(buf, 1, 16, &temp_long))
  {
    dlc = temp_long;
  }
  else
  {
    err = -1;
    LOG_ERR("Invalid dlc");
    goto parse_and_send_can_message_no_wait_11bit_return;
  }
  if (ring_buf_size_get(buf) < 2 * dlc)
  {
    err = -1;
    LOG_ERR("Not enough data in buffer");
    goto parse_and_send_can_message_no_wait_11bit_return;
  }
  for (uint32_t i = 0; i < dlc; i++)
  {
    if (!ring_buf_get_hex_to_uint8_t(buf, &can_data[i]))
    {
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
parse_and_send_can_message_no_wait_11bit(struct ring_buf *const buf)
{
  int err = 0;
  unsigned long temp_long;
  uint16_t can_id = 0;
  // First 1 for dlc
  if (ring_buf_size_get(buf) < (CAN_ID_11_BIT_BYTE_LENGHT + 1))
  {
    err = -1;
    LOG_ERR("Not enough data in buffer");
    goto parse_and_send_can_message_no_wait_11bit_return;
  }
  if (ring_buf_get_char_to_uint(buf, CAN_ID_11_BIT_BYTE_LENGHT, MESSAGE_BASE,
                                &temp_long))
  {
    can_id = temp_long;
  }
  else
  {
    err = -1;
    LOG_ERR("Invalid CAN ID");
    goto parse_and_send_can_message_no_wait_11bit_return;
  }
  err = parse_and_send_can_message_no_wait(buf, can_id, false);
parse_and_send_can_message_no_wait_11bit_return:
  return err;
}

CONFIG_UART_TO_CAN_STATIC int
parse_and_send_can_message_no_wait_29bit(struct ring_buf *const buf)
{
  int err = 0;
  unsigned long temp_long;
  uint32_t can_id = 0;
  // First 1 for dlc
  if (ring_buf_size_get(buf) < (CAN_ID_29_BIT_BYTE_LENGHT + 1))
  {
    err = -1;
    LOG_ERR("Not enough data in buffer");
    goto parse_and_send_can_message_no_wait_11bit_return;
  }
  if (ring_buf_get_char_to_uint(buf, CAN_ID_29_BIT_BYTE_LENGHT, MESSAGE_BASE,
                                &temp_long))
  {
    can_id = temp_long;
  }
  else
  {
    err = -1;
    LOG_ERR("Invalid CAN ID");
    goto parse_and_send_can_message_no_wait_11bit_return;
  }
  err = parse_and_send_can_message_no_wait(buf, can_id, true);
parse_and_send_can_message_no_wait_11bit_return:
  return err;
}

CONFIG_UART_TO_CAN_STATIC int
parse_and_add_can_filter(struct ring_buf *const buf, bool is_extended_id)
{
  int err = 0;
  uint32_t filter_id;
  uint32_t filter_mask;
  int a = ring_buf_size_get(buf);
  ARG_UNUSED(a);
  if (ring_buf_size_get(buf) < (CAN_FILTER_BYTE_LENGHT))
  {
    err = -1;
    LOG_ERR("Not enough data in buffer %d", a);
    goto parse_and_send_can_message_no_wait_11bit_return;
  }
  if (!ring_buf_get_hex_to_uint32_t(buf, &filter_id))
  {
    err = -1;
    LOG_ERR("Invalid CAN Filter ID");
    goto parse_and_send_can_message_no_wait_11bit_return;
  }
  if (!ring_buf_get_hex_to_uint32_t(buf, &filter_mask))
  {
    err = -1;
    LOG_ERR("Invalid CAN Filter mask");
    goto parse_and_send_can_message_no_wait_11bit_return;
  }
  err = add_can_filter(can_dev, filter_id, filter_mask, is_extended_id);
parse_and_send_can_message_no_wait_11bit_return:
  return err;
}

CONFIG_UART_TO_CAN_STATIC int
parse_and_add_can_filter_11bit(struct ring_buf *const buf)
{
  return parse_and_add_can_filter(buf, false);
}

CONFIG_UART_TO_CAN_STATIC int
parse_and_add_can_filter_29bit(struct ring_buf *const buf)
{
  return parse_and_add_can_filter(buf, true);
}

CONFIG_UART_TO_CAN_STATIC int
send_can_message_to_uart(const struct can_frame *frame, int filter_id)
{
  struct uart_message message = can_frame_to_uart_message(frame, filter_id);

  return send_command_status_via_uart(&message);
}

CONFIG_UART_TO_CAN_STATIC int
parse_and_remove_can_filter(struct ring_buf *const buf)
{
  int err = 0;
  unsigned long temp_long;
  uint8_t filter_id;
  if (ring_buf_size_get(buf) < 3)
  {
    err = -1;
    LOG_ERR("");
    goto parse_and_remove_can_filter_return;
  }

  if (ring_buf_get_char_to_uint(buf, 2, 16, &temp_long))
  {
    filter_id = temp_long;
  }
  else
  {
    err = -1;
    LOG_ERR("Invalid Filter ID");
    goto parse_and_remove_can_filter_return;
  }
  err = remove_can_filter(can_dev, filter_id);
parse_and_remove_can_filter_return:
  return err;
}

CONFIG_UART_TO_CAN_STATIC int remove_all_can_filters()
{
  int err = 0;
  for (int i = 0;
       i < CONFIG_CAN_MAX_STD_ID_FILTERS + CONFIG_CAN_MAX_EXT_ID_FILTERS; i++)
  {
    remove_can_filter(can_dev, i);
  }
  // int max_filters = can_get_max_filters(can_dev, false);
  // for (int i = 0; i < max_filters; i++) {
  //   can_remove_rx_filter(can_dev, i);
  // }
  // max_filters = can_get_max_filters(can_dev, true);
  // for (int i = 0; i < max_filters; i++) {
  //   can_remove_rx_filter(can_dev, i);
  // }
  return err;
}

CONFIG_UART_TO_CAN_STATIC int
parse_and_remove_all_can_filters(struct ring_buf *const buf)
{
  int err = 0;
  if (ring_buf_size_get(buf) < 1)
  {
    err = -1;
    LOG_ERR("");
    goto parse_and_remove_all_can_filters_return;
  }
  err = remove_all_can_filters();
parse_and_remove_all_can_filters_return:
  return err;
}

CONFIG_UART_TO_CAN_STATIC void send_command_response(int err,
                                                     char command_char)
{
  struct uart_message uart_message;

  uart_message.buffer[0] = command_char;
  err = snprintf(&uart_message.buffer[1], 3, "%02x", (uint8_t)((err) & 0xFF));
  __ASSERT(err == 2, "Error formatting error code");
  uart_message.buffer[3] = COMMAND_RESPONSE_OKAY[0];
  uart_message.buffer_size = 4;

  err = send_command_status_via_uart(&uart_message);
}

void can_rx_callback(const struct device *dev, struct can_frame *frame,
                     void *user_data)
{
  (void)user_data;
  int filter_id = *(int *)user_data;
  LOG_INF("Received CAN Frame from device %s! ID: 0x%03x, DLC: %d", dev->name,
          frame->id, frame->dlc);
  /* Safely print the incoming data payload using the hex helper */
  LOG_HEXDUMP_DBG(frame->data, frame->dlc, "Payload:");
  // k_msgq_put(&can_data_msgq, frame, K_NO_WAIT);
  send_can_message_to_uart(frame, filter_id);
}

void can_tx_callback(__maybe_unused const struct device *dev, int err,
                     void *user_data)
{
  char *sender = (char *)user_data;

  if (err != 0)
  {
    LOG_ERR("Sending failed from %s with [%d]", sender, err);
  }
  else
  {
    LOG_DBG("Sent succuss from %s with [%d]", sender, err);
  }
  send_command_response(err, sender[0]);
}

CONFIG_UART_TO_CAN_STATIC int send_version()
{
  struct uart_message message;

  snprintf(message.buffer, MAX_UART_CAN_FRAME, "%s", FULL_VERSION_RESPONSE);
  message.buffer_size = strlen(FULL_VERSION_RESPONSE) + 1;
  return send_command_status_via_uart(&message);
}

// CONFIG_UART_TO_CAN_STATIC int can_state_command() {
//   int err = 0;
//   struct uart_message message;
//   const char *command_response;
//   enum can_state state;
//   struct can_bus_err_cnt err_cnt;

//   err = can_get_state(can_dev, &state, &err_cnt);
//   if (err != 0) {
//     LOG_ERR("Error getting CAN state: %d", err);
//     goto reset_command_return;
//   }
//   switch (state) {
//   case CAN_STATE_ERROR_ACTIVE:
//     command_response = "A";
//     break;
//   case CAN_STATE_ERROR_WARNING:
//     command_response = "W";
//     break;
//   case CAN_STATE_ERROR_PASSIVE:
//     command_response = "P";
//     break;
//   case CAN_STATE_BUS_OFF:
//     command_response = "O";
//     break;
//   case CAN_STATE_STOPPED:
//     command_response = "S";
//     break;
//   default:
//     command_response = "U";
//     break;
//   }
//   message.buffer_size = strlen(command_response);
//   memcpy(message.buffer, command_response, message.buffer_size);
//   return send_command_status_via_uart(&message);
// reset_command_return:
//   return err;
// }

CONFIG_UART_TO_CAN_STATIC int reset_command()
{
  int err;
  enum can_state state;
  struct can_bus_err_cnt err_cnt;
  err = uart_tx_abort(uart_dev);
  if (err != 0)
  {
    LOG_ERR("Error abort UART TX: %d", err);
  }
  err = can_get_state(can_dev, &state, &err_cnt);
  if (err != 0)
  {
    LOG_ERR("Error getting CAN state: %d", err);
    goto reset_command_return;
  }
  if (state != CAN_STATE_STOPPED)
  {
    err = can_stop(can_dev);
    if (err != 0)
    {
      LOG_ERR("Error stopping CAN: %d", err);
      goto reset_command_return;
    }
  }
  remove_all_can_filters();

reset_command_return:
  return err;
}

CONFIG_UART_TO_CAN_STATIC void
print_buffer_without_clearing(struct ring_buf *buf)
{
  if (IS_ENABLED(CONFIG_LOG))
  {
    uint8_t buffer[500];
    size_t bytes_read = ring_buf_peek(buf, buffer, 500);
    LOG_HEXDUMP_INF(buffer, bytes_read, "Dumping recieve buffer:");
  }
}

// CONFIG_UART_TO_CAN_STATIC void
// send_command_add_filter_response(int filter_id, char command_char) {
//   struct uart_message uart_message;
//   uart_message.buffer[0] = command_char;

//   __ASSERT(snprintf(&uart_message.buffer[1], 3, "%02x",
//                     filter_id < 0 ? 0 : filter_id) == 2,
//            "Error formatting filter id");
//   uart_message.buffer[3] =
//       filter_id < 0 ? COMMAND_RESPONSE_ERROR[0] : COMMAND_RESPONSE_OKAY[0];
//   uart_message.buffer_size = 4;
//   send_command_status_via_uart(&uart_message);
// }

/**
 * @brief Processes data from the UART ring buffer
 *
 * @param buf Pointer to the ring buffer
 */
CONFIG_UART_TO_CAN_STATIC void
process_and_clear_command_data_uart_data(struct ring_buf *buf)
{
  int err = -1;
  print_buffer_without_clearing(buf);
  while (search_r_consider_wrap(buf) != INT_MIN)
  {
    enum UART_CAN_COMMANDS command = ring_buf_get_char(buf);
    switch (command)
    {

    case UART_CAN_COMMANDS_START_CAN:
      LOG_INF("Starting CAN");
      err = start_can_device(can_dev);
      send_command_response(err, command);
      break;
    case UART_CAN_COMMANDS_STOP_CAN:
      LOG_INF("Stopping CAN");
      err = stop_can_device(can_dev);
      send_command_response(err, command);
      break;
    case UART_CAN_COMMANDS_SET_BITRATE:
      LOG_INF("Setting CAN Bitrate");
      err = set_bitrate(can_dev, ring_buf_get_char(buf));
      send_command_response(err, command);
      break;
    // case 's':
    //   // TODO (Matthew)
    //   break;
    case UART_CAN_COMMANDS_SEND_11_BIT_CAN:
      LOG_INF("Send data to standard 11 bit CAN");
      err = parse_and_send_can_message_no_wait_11bit(buf);
      // if err then there is not enough space in the can_tx_mail_box and the issue is from the driver and the driver should clear it callback mailbox
      if (err != 0)
      {
        LOG_ERR("Failed to send CAN message: %d", err);
        err = 0xFF;
        send_command_response(err, command);
      }
      break;
    case UART_CAN_COMMANDS_SEND_29_BIT_CAN:
      LOG_INF("Send data to standard 29 bit CAN");
      err = parse_and_send_can_message_no_wait_29bit(buf);
      if (err != 0)
      {
        LOG_ERR("Failed to send CAN message: %d", err);
        err = 0xFF;
        send_command_response(err, command);
      }
      break;
    case UART_CAN_COMMANDS_VERSION:
      LOG_INF("Version");
      err = send_version(uart_dev);
      break;
    case UART_CAN_COMMANDS_HELP:
      LOG_INF("Help");
      // TODO (Matthew)
      break;
    case UART_CAN_COMMANDS_RESET:
      LOG_INF("Reset");
      err = reset_command();
      send_command_response(err, command);
      break;

    case UART_CAN_COMMANDS_GET_STATE:
      LOG_INF("Getting State");
      break;
    case UART_CAN_COMMANDS_ADD_FILTER_11_BIT:
      LOG_INF("Adding standard filter");
      err = parse_and_add_can_filter_11bit(buf);
      send_command_response(err, command);
      break;
    case UART_CAN_COMMANDS_ADD_FILTER_29_BIT:
      LOG_INF("Adding extended filter");
      err = parse_and_add_can_filter_29bit(buf);
      send_command_response(err, command);
      break;
    case UART_CAN_COMMANDS_REMOVE_FILTER:
      LOG_INF("Removing filter");
      err = parse_and_remove_can_filter(buf);
      send_command_response(err, command);
      break;
    case UART_CAN_COMMANDS_REMOVE_ALL_FILTERS:
      LOG_INF("Removing all filters");
      err = parse_and_remove_all_can_filters(buf);
      send_command_response(err, command);
      break;
    default:
      LOG_ERR("Unrecongnised command");
      err = -ENOENT;
      send_command_response(err, command);
      break;
    }
    clear_buf_till_r(buf);
  }
}

int copy_data_to_ring_buf(struct ring_buf *ring_buf, const uint8_t *const data,
                          size_t data_len)
{
  // TODO (Matthew): Change to a while loop implementation
  uint8_t *temp_data;
  int err;
  uint32_t bytes_written2 = 0;

  uint32_t bytes_written = ring_buf_put_claim(ring_buf, &temp_data, data_len);
  memcpy(temp_data, data, bytes_written);
  if (ring_buf_put_finish(ring_buf, bytes_written) != 0)
  {
    err = -1;
    goto copy_data_to_ring_buf_return;
  }

  if (bytes_written < data_len && ring_buf_space_get(ring_buf) > 0)
  {
    bytes_written2 =
        ring_buf_put_claim(ring_buf, &temp_data, data_len - bytes_written);
    memcpy(temp_data, &data[bytes_written], bytes_written2);
    if (ring_buf_put_finish(ring_buf, bytes_written2) != 0)
    {
      err = -1;
      goto copy_data_to_ring_buf_return;
    }
  }
  err = bytes_written + bytes_written2;

copy_data_to_ring_buf_return:
  return err;
}

void uart_process_work_handler(struct k_work *work)
{
  struct UartProcessWork *process_work =
      CONTAINER_OF(work, struct UartProcessWork, work);
  LOG_DBG("process_and_clear_command_data_uart_data");
  // Perform complete commands possible
  process_and_clear_command_data_uart_data(&rx_ring_buffer);
  // clear command that might have possible values dropped.
  if (process_work->buffer_overflowed)
  {
    LOG_ERR("Ring buffer full! Dropping remaining bytes");
    clear_buf_till_r(&rx_ring_buffer);
    LOG_INF("Clearing ring buffer");
    // TODO (Matthew): Send error
  }
  k_timer_start(&uart_rx_reset_buffer_timer, K_MSEC(1000), K_NO_WAIT);
}

CONFIG_UART_TO_CAN_STATIC void uart_cb(const struct device *dev,
                                       __maybe_unused struct uart_event *evt,
                                       __maybe_unused void *user_data)
{
  switch (evt->type)
  {
  case UART_TX_ABORTED:
    LOG_DBG("UART event UART_TX_ABORTED type: %d\n", evt->type);
    [[fallthrough]];
  case UART_TX_DONE:
    LOG_DBG("UART event UART_TX_DONE type: %d\n", evt->type);
    struct uart_message *msg =
        CONTAINER_OF((void *)(evt->data.tx.buf), struct uart_message, buffer);
    k_mem_slab_free(&uart_message_slab, (void *)msg);

    struct uart_message *msg_new =
        (struct uart_message *)k_fifo_get(&uart_message_fifo, K_NO_WAIT);
    if (msg_new != NULL)
    {
      LOG_INF("Backlogged message wait to be sent");
      send_command_status_via_uart_handle_fifo(msg_new);
    }
    break;

  case UART_RX_RDY:
    LOG_DBG("UART event UART_RX_RDY type: %d\n", evt->type);
    int bytes_copied = copy_data_to_ring_buf(
        &rx_ring_buffer, &evt->data.rx.buf[evt->data.rx.offset],
        evt->data.rx.len);
    if (bytes_copied < 0)
    {
      LOG_ERR("Error with copying data to ring buf");
    }
    k_work_submit(&uart_process_work.work);

    break;

  case UART_RX_BUF_REQUEST:
    LOG_DBG("UART event UART_RX_BUF_REQUEST type: %d\n", evt->type);

    if (evt->data.rx_buf.buf == dma_buf_a)
    {
      uart_rx_buf_rsp(dev, dma_buf_b, DMA_BUF_SIZE);
    }
    else
    {
      uart_rx_buf_rsp(dev, dma_buf_a, DMA_BUF_SIZE);
    }

    break;

  case UART_RX_BUF_RELEASED:
    LOG_DBG("UART event UART_RX_BUF_RELEASED type: %d\n", evt->type);
    break;

  case UART_RX_DISABLED:
    LOG_DBG("UART event UART_RX_DISABLED type: %d\n", evt->type);
    break;

  case UART_RX_STOPPED:
    LOG_DBG("UART event UART_RX_STOPPED type: %d\n", evt->type);
    break;

  default:
    break;
  }
}

int init_uart_to_can(void)
{
  int err = 0;
  if (!device_is_ready(uart_dev))
  {
    LOG_ERR("UART device not ready yet!");
    return -EIO;
  }

  if (!device_is_ready(can_dev))
  {
    LOG_ERR("CAN device not ready yet!");
    return -EIO;
  }

  /* Register the async interrupt handler */
  err = uart_callback_set(uart_dev, uart_cb, (void *)uart_dev);

  if (err)
  {
    LOG_ERR("Failed to register UART callback (err %d)", err);
    return err;
  }

  err = uart_rx_enable(uart_dev, dma_buf_a, DMA_BUF_SIZE, 20);
  if (err)
  {
    LOG_ERR("Failed to enable UART reception (err %d)", err);
    return err;
  }

  for (size_t i = 0;
       i < (CONFIG_CAN_MAX_STD_ID_FILTERS + CONFIG_CAN_MAX_EXT_ID_FILTERS);
       i++)
  {
    filter_ids_ptr_map[i] = NULL;
  }

  k_work_init(&uart_process_work.work, uart_process_work_handler);

  return err;
}

int send_command_status_via_uart_handle_fifo(struct uart_message *message)
{
  int err;

  err = uart_tx(uart_dev, message->buffer, message->buffer_size,
                500 * message->buffer_size);
  if (err == -EBUSY)
  {
    k_fifo_put(&uart_message_fifo, message);
    err = 0;
  }
  return err;
}

int send_command_status_via_uart(struct uart_message *message)
{
  struct uart_message *block_ptr;
  int err;
  err = k_mem_slab_alloc(&uart_message_slab, (void **)&block_ptr, K_NO_WAIT);

  if (err != 0)
  {
    LOG_ERR("Failed to get memory from slab");
    goto send_command_status_via_uart_return;
  }

  block_ptr->buffer_size = message->buffer_size;
  memcpy(block_ptr->buffer, message->buffer, message->buffer_size);

  LOG_INF("Sending uart message with len %d", block_ptr->buffer_size);

  LOG_HEXDUMP_INF(block_ptr->buffer, block_ptr->buffer_size, "Data");

  err = send_command_status_via_uart_handle_fifo(block_ptr);
  if (err < 0)
  {
    LOG_ERR("Failed to add message to UART msgq");
    k_mem_slab_free(&uart_message_slab, (void *)block_ptr);
    goto send_command_status_via_uart_return;
  }
send_command_status_via_uart_return:
  return err;
}
