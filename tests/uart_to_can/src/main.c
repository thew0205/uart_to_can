/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "uart_to_can.h"
#include "uart_to_can_core_utility.h"
#include "zephyr/logging/log.h"
#include "zephyr/sys/ring_buffer.h"
#include "zephyr/toolchain.h"
#include <assert.h>
#include <stdbool.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/fff.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include <zephyr/drivers/serial/serial_fake.h>

#define PRINTK_HEX_DUMP(ptr, len, str)                                         \
  printk("%s: ", str);                                                         \
  for (int i = 0; i < len; i++) {                                              \
    printk("0x%02x ", ptr[i]);                                                 \
  }                                                                            \
  printk("\n");

DEFINE_FFF_GLOBALS;

FAKE_VALUE_FUNC(int, search_r_consider_wrap, struct ring_buf *);
FAKE_VOID_FUNC(clear_buf_till_r, struct ring_buf *);
FAKE_VALUE_FUNC(int, start_can_device, const struct device *);
FAKE_VALUE_FUNC(int, stop_can_device, const struct device *);
FAKE_VALUE_FUNC(int, send_can_message_no_wait, const struct device *, uint32_t,
                const uint8_t *, uint8_t, bool, bool);
FAKE_VALUE_FUNC(int, set_bitrate, const struct device *, uint8_t);
FAKE_VALUE_FUNC(int, add_can_filter, const struct device *, uint32_t, uint32_t,
                bool);

void process_and_clear_command_data_uart_data(struct ring_buf *buf);
int parse_and_send_can_message_no_wait_11bit(struct ring_buf *buf);
int parse_and_send_can_message_no_wait_29bit(struct ring_buf *buf);
int parse_and_add_can_filter_11bit(struct ring_buf *const buf);
int parse_and_add_can_filter_29bit(struct ring_buf *const buf);

RING_BUF_DECLARE(ring_buffer, 10);
RING_BUF_DECLARE(ring_buffer_longer, 50);

static void reset_ring_buffer_before(const struct ztest_unit_test *test,
                                       void *fixture) {
  ARG_UNUSED(test);
  ARG_UNUSED(fixture);

  ring_buf_reset(&ring_buffer);
  ring_buf_reset(&ring_buffer_longer);
}

ZTEST_RULE(reset_ring_buffer_reset_rule, reset_ring_buffer_before, NULL);

static uint8_t captured_send_can_message_no_wait_custom_fake[256];
int send_can_message_no_wait_custom_fake(const struct device *can_dev,
                                         uint32_t addr, const uint8_t data[],
                                         uint8_t data_size, bool is_extended_id,
                                         bool is_rtr) {
  assert(data_size <= 256);
  memcpy(captured_send_can_message_no_wait_custom_fake, data, data_size);
  return send_can_message_no_wait_fake.return_val;
}

static void setup_fn(__maybe_unused void *temp) {
  ring_buf_reset(&ring_buffer);
  ring_buf_reset(&ring_buffer_longer);

  RESET_FAKE(search_r_consider_wrap);
  RESET_FAKE(clear_buf_till_r);
  RESET_FAKE(start_can_device);
  RESET_FAKE(stop_can_device);
  RESET_FAKE(set_bitrate);
  RESET_FAKE(send_can_message_no_wait);
  RESET_FAKE(add_can_filter);

  send_can_message_no_wait_fake.custom_fake =
      send_can_message_no_wait_custom_fake;

  FFF_RESET_HISTORY();
}

ZTEST_SUITE(process_and_clear_command_data_uart_data, NULL, NULL, setup_fn,
            NULL, NULL);

ZTEST(process_and_clear_command_data_uart_data, test_no_r_found) {
  ring_buf_put(&ring_buffer, "123", 3);

  search_r_consider_wrap_fake.return_val = INT_MIN;
  process_and_clear_command_data_uart_data(&ring_buffer);

  zassert_equal(fake_serial_uart_tx_fake.call_count, 0,
                "uart_tx should have been called zero times");
}

ZTEST(process_and_clear_command_data_uart_data, test_invalid_command) {
  ring_buf_put(&ring_buffer, "8\r8", 3);
  search_r_consider_wrap_fake.return_val = 1;
  process_and_clear_command_data_uart_data(&ring_buffer);

  zassert_equal(fake_serial_uart_tx_fake.call_count, 1,
                "uart_tx should have been called once");
  zassert_equal(fake_serial_uart_tx_custom_buffer_data_len, 1,
                "string of len 1 should have been passed to "
                "send_command_status_via_uart due to error");
  zassert_mem_equal(fake_serial_uart_tx_custom_buffer, COMMAND_RESPONSE_ERROR,
                    strlen(COMMAND_RESPONSE_ERROR),
                    "\a should have been passed to "
                    "send_command_status_via_uart due to error");
  zassert_equal(clear_buf_till_r_fake.call_count, 1,
                "clear_buf_till_r should have been called once");
}

ZTEST(process_and_clear_command_data_uart_data, test_start_can_command) {
  ring_buf_put(&ring_buffer, "O\r", 2);
  search_r_consider_wrap_fake.return_val = 1;
  fake_serial_uart_tx_fake.return_val = 0;
  process_and_clear_command_data_uart_data(&ring_buffer);
  zassert_equal(start_can_device_fake.call_count, 1,
                "start_can_device should have been called once");
  zassert_equal(fake_serial_uart_tx_fake.call_count, 1,
                "send_command_status_via_uart should have been called once");
  zassert_equal(fake_serial_uart_tx_custom_buffer_data_len,
                strlen(COMMAND_RESPONSE_OKAY),
                "string of len 1 should have been passed to "
                "send_command_status_via_uart due to error");
  zassert_mem_equal(fake_serial_uart_tx_custom_buffer, COMMAND_RESPONSE_OKAY,
                    strlen(COMMAND_RESPONSE_OKAY),
                    "\a should have been passed to "
                    "send_command_status_via_uart due to error");
}

ZTEST(process_and_clear_command_data_uart_data, test_stop_can_command) {
  ring_buf_put(&ring_buffer, "C\r", 2);
  search_r_consider_wrap_fake.return_val = 1;
  fake_serial_uart_tx_fake.return_val = 0;
  process_and_clear_command_data_uart_data(&ring_buffer);
  zassert_equal(stop_can_device_fake.call_count, 1,
                "stop_can_device should have been called once");
  zassert_equal(fake_serial_uart_tx_fake.call_count, 1,
                "send_command_status_via_uart should have been called once");
  zassert_equal(fake_serial_uart_tx_custom_buffer_data_len,
                strlen(COMMAND_RESPONSE_OKAY),
                "string of len 1 should have been passed to "
                "send_command_status_via_uart due to error");
  zassert_mem_equal(fake_serial_uart_tx_custom_buffer, COMMAND_RESPONSE_OKAY,
                    strlen(COMMAND_RESPONSE_OKAY),
                    "\a should have been passed to "
                    "send_command_status_via_uart due to error");
}

ZTEST(process_and_clear_command_data_uart_data, test_set_bitrate_can_command) {
  ring_buf_put(&ring_buffer, "S0\r", 3);
  search_r_consider_wrap_fake.return_val = 2;
  fake_serial_uart_tx_fake.return_val = 0;
  process_and_clear_command_data_uart_data(&ring_buffer);
  zassert_equal(set_bitrate_fake.call_count, 1,
                "set_bitrate should have been called once");
  zassert_equal(set_bitrate_fake.arg1_val, '0',
                "0 should have been passed to "
                "set_bitrate");
  zassert_equal(fake_serial_uart_tx_fake.call_count, 1,
                "send_command_status_via_uart should have been called once");
  zassert_equal(fake_serial_uart_tx_custom_buffer_data_len,
                strlen(COMMAND_RESPONSE_OKAY),
                "string of len 1 should have been passed to "
                "send_command_status_via_uart due to error");
  zassert_mem_equal(fake_serial_uart_tx_custom_buffer, COMMAND_RESPONSE_OKAY,
                    strlen(COMMAND_RESPONSE_OKAY),
                    "\a should have been passed to "
                    "send_command_status_via_uart due to error");
}

ZTEST(process_and_clear_command_data_uart_data,
      test_set_bitrate_can_command_wrong_bitrate_code) {
  ring_buf_put(&ring_buffer, "Sy\r", 3);
  search_r_consider_wrap_fake.return_val = 2;
  set_bitrate_fake.return_val = -EINVAL;
  fake_serial_uart_tx_fake.return_val = 0;
  process_and_clear_command_data_uart_data(&ring_buffer);
  zassert_equal(set_bitrate_fake.call_count, 1,
                "set_bitrate should have been called once");
  zassert_equal(set_bitrate_fake.arg1_val, 'y',
                "y should have been passed to "
                "set_bitrate");
  zassert_equal(fake_serial_uart_tx_fake.call_count, 1,
                "send_command_status_via_uart should have been called once");
  zassert_equal(fake_serial_uart_tx_custom_buffer_data_len,
                strlen(COMMAND_RESPONSE_ERROR),
                "string of len 1 should have been passed to "
                "send_command_status_via_uart due to error");
  zassert_mem_equal(fake_serial_uart_tx_custom_buffer, COMMAND_RESPONSE_ERROR,
                    strlen(COMMAND_RESPONSE_ERROR),
                    "\a should have been passed to "
                    "send_command_status_via_uart due to error");
}

ZTEST_SUITE(parse_and_send_can_message_no_wait_11bit, NULL, NULL, setup_fn,
            NULL, NULL);

ZTEST(parse_and_send_can_message_no_wait_11bit,
      test_correct_data_buffer_123178) {
  const char *message_data = "123178\r";
  assert(ring_buf_put(&ring_buffer, message_data, strlen(message_data)) ==
         (int)strlen(message_data));
  parse_and_send_can_message_no_wait_11bit(&ring_buffer);

  zassert_equal(send_can_message_no_wait_fake.call_count, 1,
                "send_can_message_no_wait should have been called once");
  zassert_equal(send_can_message_no_wait_fake.arg1_val, 0x123,
                "CAN ID should be 0x123");
  const uint8_t temp_data[] = {0x78};
  zassert_mem_equal(captured_send_can_message_no_wait_custom_fake, temp_data, 1,
                    "Buffers do not match!");

  zassert_equal(send_can_message_no_wait_fake.arg3_val, 1, "Should be 1");
  zassert_equal(send_can_message_no_wait_fake.arg4_val, false,
                "Should be false");
  zassert_equal(send_can_message_no_wait_fake.arg5_val, false,
                "Should be false");
}

ZTEST(parse_and_send_can_message_no_wait_11bit,
      test_correct_data_buffer_no_dlc_1230) {
  const char *message_data = "1230\r";
  assert(ring_buf_put(&ring_buffer, message_data, strlen(message_data)) ==
         (int)strlen(message_data));
  parse_and_send_can_message_no_wait_11bit(&ring_buffer);

  zassert_equal(send_can_message_no_wait_fake.call_count, 1,
                "send_can_message_no_wait should have been called once");
  zassert_equal(send_can_message_no_wait_fake.arg1_val, 0x123,
                "CAN ID should be 0x123");
  const uint8_t temp_data[] = {};
  zassert_mem_equal(captured_send_can_message_no_wait_custom_fake, temp_data, 0,
                    "Buffers do not match!");

  zassert_equal(send_can_message_no_wait_fake.arg3_val, 0, "Should be 1");
  zassert_equal(send_can_message_no_wait_fake.arg4_val, false,
                "Should be false");
  zassert_equal(send_can_message_no_wait_fake.arg5_val, false,
                "Should be false");
}

ZTEST(parse_and_send_can_message_no_wait_11bit,
      test_correct_data_buffer_full_dlc_12381234567890123456) {
  const char *message_data = "12381234567890123456\r";
  assert(ring_buf_put(&ring_buffer_longer, message_data, strlen(message_data)) ==
         (int)strlen(message_data));
  parse_and_send_can_message_no_wait_11bit(&ring_buffer_longer);

  zassert_equal(send_can_message_no_wait_fake.call_count, 1,
                "send_can_message_no_wait should have been called once");
  zassert_equal(send_can_message_no_wait_fake.arg1_val, 0x123,
                "CAN ID should be 0x123");
  const uint8_t temp_data[] = {0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56};
  zassert_mem_equal(captured_send_can_message_no_wait_custom_fake, temp_data, 8,
                    "Buffers do not match!");

  zassert_equal(send_can_message_no_wait_fake.arg3_val, 8, "Should be 1");
  zassert_equal(send_can_message_no_wait_fake.arg4_val, false,
                "Should be false");
  zassert_equal(send_can_message_no_wait_fake.arg5_val, false,
                "Should be false");
}

ZTEST(parse_and_send_can_message_no_wait_11bit,
      test_correct_data_buffer_4563112233) {
  const char *message_data = "4563112233\r";
  ring_buf_put(&ring_buffer, message_data, strlen(message_data));
  parse_and_send_can_message_no_wait_11bit(&ring_buffer);
  zassert_equal(send_can_message_no_wait_fake.call_count, 1,
                "send_can_message_no_wait should have been called once");
  zassert_equal(send_can_message_no_wait_fake.arg1_val, 0x456,
                "CAN ID should be 0x456");
  const uint8_t temp_data[] = {0x11, 0x22, 0x33};
  zassert_mem_equal(captured_send_can_message_no_wait_custom_fake, temp_data, 3,
                    "Buffers do not match!");
  zassert_equal(send_can_message_no_wait_fake.arg3_val, 3, "Should be 3");
  zassert_equal(send_can_message_no_wait_fake.arg4_val, false,
                "Should be false");
  zassert_equal(send_can_message_no_wait_fake.arg5_val, false,
                "Should be false");
}

ZTEST(parse_and_send_can_message_no_wait_11bit,
      test_wrong_data_buffer_wrong_data_length_12317) {
  const char *message_data = "12317\r";
  ring_buf_put(&ring_buffer, message_data, strlen(message_data));
  parse_and_send_can_message_no_wait_11bit(&ring_buffer);
  ztest_test_skip();
  zassert_equal(send_can_message_no_wait_fake.call_count, 0,
                "send_can_message_no_wait should not have been called once");
}

ZTEST(parse_and_send_can_message_no_wait_11bit,
      test_wrong_data_buffer_invalid_characters_u231789) {
  const char *message_data = "u231789\r";
  ring_buf_put(&ring_buffer, message_data, strlen(message_data));
  parse_and_send_can_message_no_wait_11bit(&ring_buffer);
  zassert_equal(send_can_message_no_wait_fake.call_count, 0,
                "send_can_message_no_wait should not have been called once");
}

ZTEST(parse_and_send_can_message_no_wait_11bit,
      test_wrong_data_buffer_invalid_characters_123u1789) {
  const char *message_data = "123u1789\r";
  ring_buf_put(&ring_buffer, message_data, strlen(message_data));
  parse_and_send_can_message_no_wait_11bit(&ring_buffer);
  zassert_equal(send_can_message_no_wait_fake.call_count, 0,
                "send_can_message_no_wait should not have been called once");
}

ZTEST(parse_and_send_can_message_no_wait_11bit,
      test_wrong_data_buffer_wrong_data_length_29_bit_can_message_12345678178) {
  const char *message_data = "12345678178\r";
  ring_buf_put(&ring_buffer, message_data, strlen(message_data));
  parse_and_send_can_message_no_wait_11bit(&ring_buffer);

  zassert_equal(send_can_message_no_wait_fake.call_count, 0,
                "send_can_message_no_wait should have not been called");
}

ZTEST_SUITE(parse_and_send_can_message_no_wait_29bit, NULL, NULL, setup_fn,
            NULL, NULL);

ZTEST(parse_and_send_can_message_no_wait_29bit,
      test_correct_data_buffer_12345678178) {
  const char *message_data = "12345678178\r";
  assert(ring_buf_put(&ring_buffer_longer, message_data,
                      strlen(message_data)) == (int)strlen(message_data));
  parse_and_send_can_message_no_wait_29bit(&ring_buffer_longer);

  zassert_equal(send_can_message_no_wait_fake.call_count, 1,
                "send_can_message_no_wait should have been called once");
  zassert_equal(send_can_message_no_wait_fake.arg1_val, 0x12345678,
                "CAN ID should be 0x12345678");
  const uint8_t temp_data[] = {0x78};
  zassert_mem_equal(captured_send_can_message_no_wait_custom_fake, temp_data, 1,
                    "Buffers do not match!");

  zassert_equal(send_can_message_no_wait_fake.arg3_val, 1, "Should be 1");
  zassert_equal(send_can_message_no_wait_fake.arg4_val, true, "Should be true");
  zassert_equal(send_can_message_no_wait_fake.arg5_val, false,
                "Should be false");
}

ZTEST(parse_and_send_can_message_no_wait_29bit,
      test_wrong_data_buffer_wrong_data_length_11_bit_can_message_123178) {
  const char *message_data = "123178\r";
  assert(ring_buf_put(&ring_buffer_longer, message_data,
                      strlen(message_data)) == (int)strlen(message_data));
  parse_and_send_can_message_no_wait_29bit(&ring_buffer_longer);

  zassert_equal(send_can_message_no_wait_fake.call_count, 0,
                "send_can_message_no_wait should have not been called");
}

ZTEST(parse_and_send_can_message_no_wait_29bit,
      test_correct_data_buffer_no_dlc_123456780) {
  const char *message_data = "123456780\r";
  assert(ring_buf_put(&ring_buffer_longer, message_data,
                      strlen(message_data)) == (int)strlen(message_data));
  parse_and_send_can_message_no_wait_29bit(&ring_buffer_longer);

  zassert_equal(send_can_message_no_wait_fake.call_count, 1,
                "send_can_message_no_wait should have been called once");
  zassert_equal(send_can_message_no_wait_fake.arg1_val, 0x12345678,
                "CAN ID should be 0x12345678");
  const uint8_t temp_data[] = {};
  zassert_mem_equal(captured_send_can_message_no_wait_custom_fake, temp_data, 0,
                    "Buffers do not match!");

  zassert_equal(send_can_message_no_wait_fake.arg3_val, 0, "Should be 1");
  zassert_equal(send_can_message_no_wait_fake.arg4_val, true, "Should be true");
  zassert_equal(send_can_message_no_wait_fake.arg5_val, false,
                "Should be false");
}

ZTEST_SUITE(parse_and_add_can_filter_11bit, NULL, NULL, setup_fn, NULL, NULL);

ZTEST(parse_and_add_can_filter_11bit, test_correct_data_buffer_1234567801234567) {
  const char *message_data = "1234567801234567\r";
  assert(ring_buf_put(&ring_buffer_longer, message_data, strlen(message_data)) ==
         (int)strlen(message_data));
  parse_and_add_can_filter_11bit(&ring_buffer_longer);

  zassert_equal(add_can_filter_fake.call_count, 1,
                "add_can_filter should have been called once");
  zassert_equal(add_can_filter_fake.arg1_val, 0x12345678,
                "CAN ID should be 0x12345678");
  zassert_equal(add_can_filter_fake.arg2_val, 0x01234567,
                "CAN ID should be 0x01234567");
  zassert_equal(add_can_filter_fake.arg3_val, false, "Should be false");
}

ZTEST(parse_and_add_can_filter_11bit, test_incomplete_data_buffer_12345678012345) {
  const char *message_data = "12345678012345\r";
  assert(ring_buf_put(&ring_buffer_longer, message_data, strlen(message_data)) ==
         (int)strlen(message_data));
  parse_and_add_can_filter_11bit(&ring_buffer_longer);

  zassert_equal(add_can_filter_fake.call_count, 0,
                "add_can_filter should have been called once");
}



ZTEST_SUITE(parse_and_add_can_filter_29bit, NULL, NULL, setup_fn, NULL, NULL);

ZTEST(parse_and_add_can_filter_29bit, test_correct_data_buffer_1234567801234567) {
  const char *message_data = "1234567801234567\r";
  assert(ring_buf_put(&ring_buffer_longer, message_data, strlen(message_data)) ==
         (int)strlen(message_data));
  parse_and_add_can_filter_29bit(&ring_buffer_longer);

  zassert_equal(add_can_filter_fake.call_count, 1,
                "add_can_filter should have been called once");
  zassert_equal(add_can_filter_fake.arg1_val, 0x12345678,
                "CAN ID should be 0x12345678");
  zassert_equal(add_can_filter_fake.arg2_val, 0x01234567,
                "CAN ID should be 0x01234567");
  zassert_equal(add_can_filter_fake.arg3_val, true, "Should be false");
}

ZTEST(parse_and_add_can_filter_29bit, test_incomplete_data_buffer_12345678012345) {
  const char *message_data = "12345678012345\r";
  assert(ring_buf_put(&ring_buffer_longer, message_data, strlen(message_data)) ==
         (int)strlen(message_data));
  parse_and_add_can_filter_29bit(&ring_buffer_longer);

  zassert_equal(add_can_filter_fake.call_count, 0,
                "add_can_filter should have been called once");
}

// /*ZTEST(process_and_clear_command_data_uart_data, test_start_can_command) {
//   ring_buf_put(&ring_buffer, "O\r", 2);

//   process_and_clear_command_data_uart_data(&ring_buffer);
//   zassert_equal(start_can_fake.call_count, 1,
//                 "start_can should have been called once");
// }
// */