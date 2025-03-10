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
  err = esp32_read_efuse_nth_word(port, &mac[0], ESP32_EFUSE_RD_OFF_MAC_1);
  if (err != ESP_SUCCESS) {
    return err;
  }

  err = esp32_read_efuse_nth_word(port, &mac[1], ESP32_EFUSE_RD_OFF_MAC_2);
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

esp_error_t esp32_get_pkg_version(int port, uint32_t *out) {
  uint32_t pkg_ver;
  esp_error_t err =
      esp32_read_efuse_nth_word(port, &pkg_ver, ESP32_EFUSE_RD_OFF_PKG_VER);
  if (err != ESP_SUCCESS) {
    return err;
  }

  *out = (pkg_ver >> 9) & 0x07;
  *out += ((pkg_ver >> 2) & 0x1) << 3;

  return ESP_SUCCESS;
}

esp_error_t esp32_get_minor_revision(int port, uint32_t *out) {
  uint32_t minor_rev;
  esp_error_t err =
      esp32_read_efuse_nth_word(port, &minor_rev, ESP32_EFUSE_RD_OFF_MINOR_REV);
  if (err != ESP_SUCCESS) {
    return err;
  }
  *out = (minor_rev >> 24) & 0x3;
  return ESP_SUCCESS;
}

esp_error_t esp32_get_major_revision(int port, uint32_t *out) {
  esp_error_t err;
  uint32_t word3;
  uint32_t word5;
  err = esp32_read_efuse_nth_word(port, &word3, ESP32_EFUSE_RD_OFF_PKG_VER);
  if (err != ESP_SUCCESS) {
    return err;
  }

  err = esp32_read_efuse_nth_word(port, &word5, ESP32_EFUSE_RD_OFF_MINOR_REV);
  if (err != ESP_SUCCESS) {
    return err;
  }

  uint32_t apb_ctl;
  err = esp_chip_read_reg(port, ESP32_APB_CTL_DATE_ADDR, &apb_ctl, NULL, NULL);
  if (err != ESP_SUCCESS) {
    return err;
  }

  uint8_t rev_bit0 = (word3 >> 15) & 0x1;
  uint8_t rev_bit1 = (word5 >> 20) & 0x1;
  uint8_t rev_bit2 = (apb_ctl >> ESP32_APB_CTL_DATE_S) & ESP32_APB_CTL_DATE_V;
  uint8_t combined = (rev_bit2 << 2) | (rev_bit1 << 1) | rev_bit0;

  switch (combined) {
    case 0:
      *out = 0;
      break;
    case 1:
      *out = 1;
      break;
    case 3:
      *out = 2;
      break;
    case 7:
      *out = 3;
      break;
    default:
      *out = 0;
      break;
  }

  return ESP_SUCCESS;
}

esp_error_t esp32_is_single_core(int port, bool *out) {
  uint32_t word3;
  esp_error_t err =
      esp32_read_efuse_nth_word(port, &word3, ESP32_EFUSE_RD_OFF_PKG_VER);
  if (err != ESP_SUCCESS) {
    return err;
  }

  *out = ((word3) & (1 << 0)) == 1;
  return ESP_SUCCESS;
}

esp_error_t esp32_get_chip_desc(int port, esp32_chip_desc_t *out) {
  uint32_t pkg_version;
  uint32_t major_rev;
  uint32_t minor_rev;
  bool is_single_core;
  bool is_rev3;

  esp_error_t err = esp32_get_pkg_version(port, &pkg_version);
  if (err != ESP_SUCCESS) {
    return err;
  }

  err = esp32_get_major_revision(port, &major_rev);
  if (err != ESP_SUCCESS) {
    return err;
  }

  err = esp32_get_minor_revision(port, &minor_rev);
  if (err != ESP_SUCCESS) {
    return err;
  }

  err = esp32_is_single_core(port, &is_single_core);
  if (err != ESP_SUCCESS) {
    return err;
  }

  is_rev3 = major_rev == 3;
  esp32_chip_name_t chip_name = ESP32_NAME_ESP32_UNKNOWN;
  switch (pkg_version) {
    case 0:
      if (is_single_core) {
        chip_name = ESP32_NAME_ESP32_S0WDQ6;
      } else {
        chip_name =
            is_rev3 ? ESP32_NAME_ESP32_D0WDQ6_V3 : ESP32_NAME_ESP32_D0WDQ6;
      }
      break;
    case 1:
      if (is_single_core) {
        chip_name = ESP32_NAME_ESP32_S0WD;
      } else {
        chip_name = is_rev3 ? ESP32_NAME_ESP32_D0WD_V3 : ESP32_NAME_ESP32_D0WD;
      }
      break;
    case 2:
      chip_name = ESP32_NAME_ESP32_D2WD;
      break;
    case 4:
      chip_name = ESP32_NAME_ESP32_U4WDH;
      break;
    case 5:
      chip_name = is_rev3 ? ESP32_NAME_ESP32_PICO_V3 : ESP32_NAME_ESP32_PICO_D4;
      break;
    case 6:
      chip_name = ESP32_NAME_ESP32_PICO_V3_02;
    case 7:
      chip_name = ESP32_NAME_ESP32_D0WDR2_V3;
      break;
    default:
      return ESP_ERR_INVALID_ARG;
  }

  out->name = chip_name;
  out->major_rev = major_rev;
  out->minor_rev = minor_rev;
  return ESP_SUCCESS;
}

const char *esp32_chip_name_str(esp32_chip_name_t name) {
  switch (name) {
    case ESP32_NAME_ESP32_S0WDQ6:
      return "ESP32-S0WDQ6";
    case ESP32_NAME_ESP32_D0WDQ6_V3:
      return "ESP32-D0WDQ6-V3";
    case ESP32_NAME_ESP32_D0WDQ6:
      return "ESP32-D0WDQ6";
    case ESP32_NAME_ESP32_S0WD:
      return "ESP32-S0WD";
    case ESP32_NAME_ESP32_D0WD_V3:
      return "ESP32-D0WD-V3";
    case ESP32_NAME_ESP32_D0WD:
      return "ESP32-D0WD";
    case ESP32_NAME_ESP32_D2WD:
      return "ESP32-D2WD";
    case ESP32_NAME_ESP32_U4WDH:
      return "ESP32-U4WDH";
    case ESP32_NAME_ESP32_PICO_V3:
      return "ESP32-PICO-V3";
    case ESP32_NAME_ESP32_PICO_D4:
      return "ESP32-PICO-D4";
    case ESP32_NAME_ESP32_PICO_V3_02:
      return "ESP32-PICO-V3-02";
    case ESP32_NAME_ESP32_D0WDR2_V3:
      return "ESP32-D0WDR2-V3";
    default:
      return "unknown ESP32";
  }
}
