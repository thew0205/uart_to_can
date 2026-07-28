#ifndef __UART_TO_CAN_CORE_UTILITY_H__
#define __UART_TO_CAN_CORE_UTILITY_H__

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/ring_buffer.h>

#define HARDWARE_VERSION_MAJOR 0
#define HARDWARE_VERSION_MINOR 1

#define SOFTWARE_VERSION_MAJOR 0
#define SOFTWARE_VERSION_MINOR 1

#define CAN_ID_11_BIT_BYTE_LENGHT 3
#define CAN_ID_29_BIT_BYTE_LENGHT 8
#define CAN_FILTER_BYTE_LENGHT (8 + 8)
#define MESSAGE_BASE 16
#define COMMAND_RESPONSE_OKAY "\r"
#define COMMAND_RESPONSE_ERROR "\a"

#define FULL_VERSION_RESPONSE       \
  STRINGIFY(HARDWARE_VERSION_MAJOR) \
  STRINGIFY(HARDWARE_VERSION_MINOR) \
  STRINGIFY(SOFTWARE_VERSION_MAJOR) \
  STRINGIFY(SOFTWARE_VERSION_MINOR) \
  COMMAND_RESPONSE_OKAY

#define MAX_UART_CAN_FRAME \
  (1 + CAN_ID_29_BIT_BYTE_LENGHT + 2 + 1 + CAN_MAX_DLEN * 2 + 1)

#if defined(CONFIG_CAN_STM32_BXCAN_MAX_STD_ID_FILTERS)
#define CONFIG_CAN_MAX_STD_ID_FILTERS CONFIG_CAN_STM32_BXCAN_MAX_STD_ID_FILTERS
#else
#define CONFIG_CAN_MAX_STD_ID_FILTERS 5
#endif

#if defined(CONFIG_CAN_STM32_BXCAN_MAX_EXT_ID_FILTERS)
#define CONFIG_CAN_MAX_EXT_ID_FILTERS CONFIG_CAN_STM32_BXCAN_MAX_EXT_ID_FILTERS
#else
#define CONFIG_CAN_MAX_EXT_ID_FILTERS 5
#endif

struct uart_message
{
  void *fifo_reserved; /* 1st word reserved for use by FIFO */
  uint8_t buffer[MAX_UART_CAN_FRAME];
  size_t buffer_size;
};

static inline struct uart_message string_to_uart_message(const char *string)
{
  struct uart_message uart_message;
  uart_message.buffer_size = strlen(string);
  assert((size_t)snprintf(uart_message.buffer, MAX_UART_CAN_FRAME, "%s",
                          string) == uart_message.buffer_size);
  return uart_message;
}

/**
 * @brief Return the next byte in the ring buffer, this function assume that you
 * are sure that data is in the buffer, it asserts if the buffer is empty
 * @param buf Pointer to the ring buffer
 * @return the next byte in the ring buffer
 */
static inline uint8_t ring_buf_get_char(struct ring_buf *buf)
{
  uint8_t data;
  assert(ring_buf_get(buf, &data, 1) == 1);
  return data;
}

static inline bool ring_buf_get_char_to_uint(struct ring_buf *buf,
                                             uint8_t num_of_bytes, int base,
                                             unsigned long *out_val)
{
  char data[num_of_bytes + 1];
  char *temp_data;
  unsigned long temp_val;
  for (uint32_t i = 0; i < num_of_bytes; i++)
  {
    data[i] = ring_buf_get_char(buf);
  }
  data[num_of_bytes] = '\0';
  temp_val = strtoul(data, &temp_data, base);

  if ((uintptr_t)temp_data != (uintptr_t)data)
  {
    *out_val = temp_val;
    return true;
  }
  return false;
}

static inline bool ring_buf_get_hex_to_uint8_t(struct ring_buf *buf,
                                               uint8_t *out_val)
{
  char data[2 + 1];
  char *temp_data;
  unsigned long temp_val;
  for (uint32_t i = 0; i < 2; i++)
  {
    data[i] = ring_buf_get_char(buf);
  }
  data[2] = '\0';
  temp_val = strtoul(data, &temp_data, 16);

  if ((uintptr_t)temp_data != (uintptr_t)data)
  {
    *out_val = temp_val;
    return true;
  }
  return false;
}

static inline bool ring_buf_get_hex_to_uint32_t(struct ring_buf *buf,
                                                uint32_t *out_val)
{
  char data[8 + 1];
  char *temp_data;
  unsigned long temp_val;
  for (uint32_t i = 0; i < 8; i++)
  {
    data[i] = ring_buf_get_char(buf);
  }
  data[8] = '\0';
  temp_val = strtoul(data, &temp_data, 16);

  if ((uintptr_t)temp_data != (uintptr_t)data)
  {
    *out_val = temp_val;
    return true;
  }
  return false;
}
/**
 * @brief Checks if the ring buffer has wrapped
 *
 * @param buf Pointer to the ring buffer
 * @return true if the ring buffer has wrapped, false otherwise
 */
bool ring_buf_has_wrapped(struct ring_buf *buf);

/**
 * @brief Starts the CAN device
 *
 * @param can_dev Pointer to the CAN device
 * @return 0 on success, negative error code on failure
 */
int start_can_device(const struct device *can_dev);

/**
 * @brief Stops the CAN device
 *
 * @param can_dev Pointer to the CAN device
 * @return 0 on success, negative error code on failure
 */
int stop_can_device(const struct device *can_dev);

/**
 * @brief Sets the bitrate for the CAN device
 *
 * @param can_dev Pointer to the CAN device
 * @param bitrate_code uint8_t the encoded value of the bitrate
 * @return 0 on success, negative error code on failure
 */
int set_bitrate(const struct device *can_dev, uint8_t bitrate_code);

/**
 * @brief Adds a filter to the CAN device
 *
 * @param can_dev Pointer to the CAN device
 * @param filter_id The filter ID
 * @param filter_mask The filter mask
 * @param is_extended_id Whether the filter is for an extended ID
 * @return 0 on success, negative error code on failure
 */
int add_can_filter(const struct device *can_dev, uint32_t filter_id,
                   uint32_t filter_mask, bool is_extended_id);

/**
 * @brief Sends a message to the CAN bus
 *
 * @param can_dev Pointer to the CAN device
 * @param addr The CAN ID of the message
 * @param data Pointer to the data to send
 * @param data_size The size of the data to send
 * @param is_extended_id Whether the message is an extended ID
 * @param is_rtr Whether the message is an remote data
 * @return true if the message was sent successfully, false otherwise
 */
int send_can_message_no_wait(const struct device *can_dev, uint32_t addr,
                             const uint8_t data[], uint8_t data_size,
                             bool is_extended_id, bool is_rtr);

/**
 * @brief Searches for '\r' in the ring buffer, considering wrap-around
 *
 * @param buf Pointer to the ring buffer
 * @return The index of '\r' if found (positive index) or if wrap around
 *         occured (negative index), INT_MIN otherwise
 */
int search_r_consider_wrap(struct ring_buf *buf);

int send_command_status_via_uart(struct uart_message *message);

/**
@breif Clear all the character in a ring buffer before the char '\r' and if no
'\r', it clears the whole buffer
*/
void clear_buf_till_r(struct ring_buf *buf);

/**
 * @brief Converts a CAN frame to a uart message
 *
 * @param frame The CAN frame to convert
 * @return The uart message
 */
struct uart_message can_frame_to_uart_message(const struct can_frame *frame, int filter_id);
int remove_can_filter(const struct device *can_dev, int filter_id);
#endif /* __UART_TO_CAN_CORE_UTILITY_H__ */
