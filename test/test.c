#include "slip_reader.h"
#include "unity.h"

void setUp(void) {}

void tearDown(void) {}

void test_slip_reader_jittery_response(void) {
  uint8_t expected[] = {0x01, 0x08, 0x04, 0x00, 0x12, 0x20,
                        0x55, 0x55, 0x00, 0x00, 0x00, 0x00};
  uint8_t output_buf[128];
  slip_reader_t reader;
  slip_reader_init(&reader, output_buf, sizeof(output_buf));

  uint8_t expected_message_counts = 8;
  uint8_t expected_message_length = 12;
  uint8_t message_counts = 0;

  const uint8_t transmit_1[] = {0xC0, 0x01, 0x08, 0x04, 0x00, 0x12, 0x20, 0x55,
                                0x55, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0x01};
  for (size_t i = 0; i < sizeof(transmit_1); i++) {
    slip_reader_state_t state =
        slip_reader_process_byte(&reader, transmit_1[i]);
    if (state == SLIP_READER_END) {
      TEST_ASSERT_TRUE(slip_reader_data_ready(&reader));
      TEST_ASSERT_EQUAL(expected_message_length, reader.length);
      TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, reader.buf,
                                    expected_message_length);
      slip_reader_reset(&reader);
      message_counts++;
    }
  }

  // processing the second message, incomplete
  TEST_ASSERT_EQUAL(SLIP_READER_DATA, reader.state);
  TEST_ASSERT_FALSE(slip_reader_data_ready(&reader));

  const uint8_t transmit_2[] = {0x08, 0x04, 0x00, 0x12};
  for (size_t i = 0; i < sizeof(transmit_2); i++) {
    slip_reader_state_t state =
        slip_reader_process_byte(&reader, transmit_2[i]);
    if (state == SLIP_READER_END) {
      TEST_ABORT();
    }
  }

  TEST_ASSERT_EQUAL(SLIP_READER_DATA, reader.state);
  TEST_ASSERT_FALSE(slip_reader_data_ready(&reader));

  const uint8_t transmit_3[] = {0x20, 0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0xC0,
                                0xC0, 0x01, 0x08, 0x04, 0x00, 0x12, 0x20, 0x55};
  for (size_t i = 0; i < sizeof(transmit_3); i++) {
    slip_reader_state_t state =
        slip_reader_process_byte(&reader, transmit_3[i]);
    if (state == SLIP_READER_END) {
      TEST_ASSERT_TRUE(slip_reader_data_ready(&reader));
      TEST_ASSERT_EQUAL(reader.length, expected_message_length);
      TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, reader.buf,
                                    expected_message_length);
      slip_reader_reset(&reader);
      message_counts++;
    }
  }

  TEST_ASSERT_EQUAL(SLIP_READER_DATA, reader.state);
  TEST_ASSERT_FALSE(slip_reader_data_ready(&reader));

  const uint8_t transmit_4[] = {0x55, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0x01};
  for (size_t i = 0; i < sizeof(transmit_4); i++) {
    slip_reader_state_t state =
        slip_reader_process_byte(&reader, transmit_4[i]);
    if (state == SLIP_READER_END) {
      TEST_ASSERT_TRUE(slip_reader_data_ready(&reader));
      TEST_ASSERT_EQUAL(expected_message_length, reader.length);
      TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, reader.buf,
                                    expected_message_length);
      slip_reader_reset(&reader);
      message_counts++;
    }
  }

  TEST_ASSERT_EQUAL(SLIP_READER_DATA, reader.state);
  TEST_ASSERT_FALSE(slip_reader_data_ready(&reader));

  const uint8_t transmit_5[] = {0x08, 0x04, 0x00, 0x12, 0x20, 0x55, 0x55, 0x00,
                                0x00, 0x00, 0x00, 0xC0, 0xC0, 0x01, 0x08, 0x04};
  for (size_t i = 0; i < sizeof(transmit_5); i++) {
    slip_reader_state_t state =
        slip_reader_process_byte(&reader, transmit_5[i]);
    if (state == SLIP_READER_END) {
      TEST_ASSERT_TRUE(slip_reader_data_ready(&reader));
      TEST_ASSERT_EQUAL(reader.length, expected_message_length);
      TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, reader.buf,
                                    expected_message_length);
      slip_reader_reset(&reader);
      message_counts++;
    }
  }

  TEST_ASSERT_EQUAL(SLIP_READER_DATA, reader.state);
  TEST_ASSERT_FALSE(slip_reader_data_ready(&reader));

  const uint8_t transmit_6[] = {0x00, 0x12, 0x20, 0x55, 0x55, 0x00};
  for (size_t i = 0; i < sizeof(transmit_6); i++) {
    slip_reader_state_t state =
        slip_reader_process_byte(&reader, transmit_6[i]);
    if (state == SLIP_READER_END) {
      TEST_ABORT();
    }
  }

  TEST_ASSERT_EQUAL(SLIP_READER_DATA, reader.state);
  TEST_ASSERT_FALSE(slip_reader_data_ready(&reader));

  const uint8_t transmit_7[] = {0x00, 0x00, 0x00, 0xC0, 0xC0, 0x01, 0x08, 0x04,
                                0x00, 0x12, 0x20, 0x55, 0x55, 0x00, 0x00, 0x00};
  for (size_t i = 0; i < sizeof(transmit_7); i++) {
    slip_reader_state_t state =
        slip_reader_process_byte(&reader, transmit_7[i]);
    if (state == SLIP_READER_END) {
      TEST_ASSERT_TRUE(slip_reader_data_ready(&reader));
      TEST_ASSERT_EQUAL(expected_message_length, reader.length);
      TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, reader.buf,
                                    expected_message_length);
      slip_reader_reset(&reader);
      message_counts++;
    }
  }
  TEST_ASSERT_EQUAL(SLIP_READER_DATA, reader.state);
  TEST_ASSERT_FALSE(slip_reader_data_ready(&reader));

  const uint8_t transmit_8[] = {0x00, 0xC0, 0xC0, 0x01, 0x08, 0x04, 0x00, 0x12};
  for (size_t i = 0; i < sizeof(transmit_8); i++) {
    slip_reader_state_t state =
        slip_reader_process_byte(&reader, transmit_8[i]);
    if (state == SLIP_READER_END) {
      TEST_ABORT();
    }
  }

  TEST_ASSERT_EQUAL(SLIP_READER_DATA, reader.state);
  TEST_ASSERT_FALSE(slip_reader_data_ready(&reader));

  const uint8_t transmit_9[] = {0x20, 0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0xC0,
                                0xC0, 0x01, 0x08, 0x04, 0x00, 0x12, 0x20, 0x55};
  for (size_t i = 0; i < sizeof(transmit_9); i++) {
    slip_reader_state_t state =
        slip_reader_process_byte(&reader, transmit_9[i]);
    if (state == SLIP_READER_END) {
      TEST_ASSERT_TRUE(slip_reader_data_ready(&reader));
      TEST_ASSERT_EQUAL(expected_message_length, reader.length);
      TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, reader.buf,
                                    expected_message_length);
      slip_reader_reset(&reader);
      message_counts++;
    }
  }
  TEST_ASSERT_EQUAL(SLIP_READER_DATA, reader.state);
  TEST_ASSERT_FALSE(slip_reader_data_ready(&reader));

 
  const uint8_t transmit_10[] = {0x55, 0x00, 0x00, 0x00, 0x00, 0xC0};
  for (size_t i = 0; i < sizeof(transmit_10); i++) {
    slip_reader_state_t state =
        slip_reader_process_byte(&reader, transmit_10[i]);
    if (state == SLIP_READER_END) {
      TEST_ASSERT_TRUE(slip_reader_data_ready(&reader));
      TEST_ASSERT_EQUAL(expected_message_length, reader.length);
      TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, reader.buf,
                                    expected_message_length);
      slip_reader_reset(&reader);
      message_counts++;
    }
  }

  TEST_ASSERT_EQUAL(SLIP_READER_START, reader.state);
  TEST_ASSERT_EQUAL(expected_message_counts, message_counts);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_slip_reader_jittery_response);
  return UNITY_END();
}