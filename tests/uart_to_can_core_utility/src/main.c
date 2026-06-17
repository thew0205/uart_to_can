/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "uart_to_can_core_utility.h"

#include "zephyr/drivers/can.h"
#include "zephyr/sys/ring_buffer.h"
#include <assert.h>
#include <string.h>
#include <zephyr/ztest.h>

bool ring_buf_has_wrapped(struct ring_buf *buf);
int search_r_consider_wrap(struct ring_buf *buf);
void process_data_uart_data(struct ring_buf *buf);

RING_BUF_DECLARE(ring_buffer, 10);

static void setup_fn(__maybe_unused void *temp) {
  ring_buf_reset(&ring_buffer);
}

ZTEST_SUITE(buffer_has_wrapped, NULL, NULL, setup_fn, NULL, NULL);

ZTEST(buffer_has_wrapped, test_init_buf_has_no_wrap) {

  zassert(!ring_buf_has_wrapped(&ring_buffer), "Should not have wrapped");
}

ZTEST(buffer_has_wrapped, test_buf_has_data_no_wrap) {
  ring_buf_put(&ring_buffer, (const uint8_t[]){1, 2, 3, 4, 5}, 5);
  zassert(!ring_buf_has_wrapped(&ring_buffer), "Should not have wrapped");
}

ZTEST(buffer_has_wrapped, test_buf_has_data_then_wrap_then_unwrap) {
  ring_buf_put(&ring_buffer, (const uint8_t[]){1, 2, 3, 4, 5, 6, 7, 8}, 8);
  uint8_t temp[6];
  ring_buf_get(&ring_buffer, temp, 6);
  ring_buf_put(&ring_buffer, (const uint8_t[]){1, 2, 3, 4, 5, 6, 7}, 7);
  ring_buf_get(&ring_buffer, temp, 5);

  zassert(!ring_buf_has_wrapped(&ring_buffer), "Should have unwrapped");
}

ZTEST_SUITE(search_r_consider_wrap, NULL, NULL, setup_fn, NULL, NULL);

ZTEST(search_r_consider_wrap, test_search_t_no_wrap) {
  ring_buf_put(&ring_buffer, "Hello\r", 6);

  zassert_equal(search_r_consider_wrap(&ring_buffer), 5,
                "Should find\r at index 5");
}

ZTEST(search_r_consider_wrap, test_search_t_with_wrap) {

  ring_buf_put(&ring_buffer, (const uint8_t[]){1, 2, 3, 4, 5, 6, 7, 8}, 8);
  uint8_t temp[8];
  ring_buf_get(&ring_buffer, temp, 8);

  ring_buf_put(&ring_buffer, "Hello\r", 6);

  zassert_equal(search_r_consider_wrap(&ring_buffer), -3,
                "Should find\r at index -3");
}

ZTEST_SUITE(clear_buf_till_r, NULL, NULL, setup_fn, NULL, NULL);

ZTEST(clear_buf_till_r, test_empty_buffer) {
  clear_buf_till_r(&ring_buffer);
  zassert_equal(ring_buf_size_get(&ring_buffer), 0, "Should be empty");
}

ZTEST(clear_buf_till_r, test_clear_to_remian_1) {
  ring_buf_put(&ring_buffer, "Hello\r7", 7);
  clear_buf_till_r(&ring_buffer);
  zassert_equal(ring_buf_size_get(&ring_buffer), 1, "Should have one element");

  zassert_equal(ring_buf_get_char(&ring_buffer), '7',
                "Should have a value of \'7\'");
}

ZTEST(clear_buf_till_r, test_clear_to_remian_0) {
  ring_buf_put(&ring_buffer, "Hello\r", 6);
  clear_buf_till_r(&ring_buffer);
  zassert_equal(ring_buf_size_get(&ring_buffer), 0, "Should be empty");
}

ZTEST(clear_buf_till_r, test_clear_no_r) {
  ring_buf_put(&ring_buffer, "Hello9", 6);
  clear_buf_till_r(&ring_buffer);
  zassert_equal(ring_buf_size_get(&ring_buffer), 0, "Should be empty");
}

ZTEST(clear_buf_till_r, test_clear_with_two_r) {
  ring_buf_put(&ring_buffer, "Hello\r7\r8", 9);
  clear_buf_till_r(&ring_buffer);
  zassert_equal(ring_buf_size_get(&ring_buffer), 3,
                "Should have three element");
  clear_buf_till_r(&ring_buffer);
  zassert_equal(ring_buf_size_get(&ring_buffer), 1, "Should have one element");
}

// // TODO (Matthew): Test set_bitrate

ZTEST_SUITE(can_frame_to_uart_message, NULL, NULL, setup_fn, NULL, NULL);

ZTEST(can_frame_to_uart_message, test_can_id_11bit_empty_buffer) {
  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));

  struct uart_message message = can_frame_to_uart_message(&frame);
  zassert_equal(message.buffer_size, 6, "Should be empty");
  zassert_mem_equal(message.buffer, "t0000" COMMAND_RESPONSE_OKAY, 6, "");
}

ZTEST(can_frame_to_uart_message, test_can_id_29bit_empty_buffer) {
  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));
  frame.flags |= CAN_FRAME_IDE;

  struct uart_message message = can_frame_to_uart_message(&frame);
  zassert_equal(message.buffer_size, 11, "Should be empty");
  zassert_mem_equal(message.buffer, "T000000000" COMMAND_RESPONSE_OKAY, 11,
                    "Buffer should match");
}

ZTEST(can_frame_to_uart_message, test_can_id_11bit_with_data) {
  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));
  frame.id = 0x123;
  frame.dlc = 2;
  memcpy(frame.data, (const uint8_t[]){0x55, 0xaa}, 2);

  struct uart_message message = can_frame_to_uart_message(&frame);
  zassert_equal(message.buffer_size, 10, "Should be empty");
  zassert_mem_equal(message.buffer, "t123255aa" COMMAND_RESPONSE_OKAY, 10,
                    "Buffer should match");
}

ZTEST(can_frame_to_uart_message, test_can_id_29bit_with_data) {
  struct can_frame frame;
  memset(&frame, 0, sizeof(frame));
  frame.flags |= CAN_FRAME_IDE;

  frame.id = 0x12345678;
  frame.dlc = 2;
  memcpy(frame.data, (const uint8_t[]){0x55, 0xaa}, 2);

  struct uart_message message = can_frame_to_uart_message(&frame);

  zassert_equal(message.buffer_size, 15, "Should be empty");
  zassert_mem_equal(message.buffer, "T12345678255aa" COMMAND_RESPONSE_OKAY, 15,
                    "Buffer should match");
}