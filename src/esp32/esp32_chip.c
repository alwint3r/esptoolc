#include "esp32_chip.h"

#include "esp_chip.h"

esp_error_t esp32_read_efuse_nth_word(int port, uint32_t *out, size_t n_word) {
  uint32_t addr = ESP32_EFUSE_RD_REG_BASE + (4 * n_word);
  uint32_t value_out;
  esp_error_t err = esp_chip_read_reg(port, addr, &value_out, NULL, NULL);
  if (err != ESP_SUCCESS) {
    return err;
  }

  *out = value_out;
  return ESP_SUCCESS;
}
esp_error_t esp32_read_mac(int port, uint8_t *out) {
  uint32_t mac[2];
  esp_error_t err;
  err = esp32_read_efuse_nth_word(port, &mac[0], 2);
  if (err != ESP_SUCCESS) {
    return err;
  }

  err = esp32_read_efuse_nth_word(port, &mac[1], 1);
  if (err != ESP_SUCCESS) {
    return err;
  }

  // first two bytes are the CRC, we can ignore them
  out[0] = (mac[0] >> 8) & 0xFF;
  out[1] = mac[0] & 0xFF;
  out[2] = (mac[1] >> 24) & 0xFF;
  out[3] = (mac[1] >> 16) & 0xFF;
  out[4] = (mac[1] >> 8) & 0xFF;
  out[5] = mac[1] & 0xFF;

  return ESP_SUCCESS;
}
