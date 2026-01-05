#include "ip5306.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ip5306 {

static const char *const TAG = "ip5306";

static const uint8_t IP5306_REG_SYS_CTL0 = 0x00;
static const uint8_t IP5306_REG_SYS_CTL2 = 0x02;
static const uint8_t IP5306_REG_CHARGER_CTL0 = 0x20;
static const uint8_t IP5306_REG_CHARGER_CTL1 = 0x21;
static const uint8_t IP5306_REG_CHARGER_CTL2 = 0x22;
static const uint8_t IP5306_REG_READ0 = 0x70;
static const uint8_t IP5306_REG_LEVEL = 0x78;

void IP5306::setup() {
  ESP_LOGCONFIG(TAG, "Setting up IP5306...");

  // Switches
  if (this->low_load_shutdown_switch_ != nullptr) {
    uint8_t value;
    this->read_register(IP5306_REG_SYS_CTL0, &value, 1);
    this->low_load_shutdown_switch_->publish_state(value & 0x02);
  }

  if (this->charger_enable_switch_ != nullptr) {
    uint8_t value;
    this->read_register(IP5306_REG_SYS_CTL0, &value, 1);
    this->charger_enable_switch_->publish_state(value & 0x10);
  }

  if (this->charge_control_switch_ != nullptr) {
    uint8_t value;
    this->read_register(IP5306_REG_CHARGER_CTL0, &value, 1);
    this->charge_control_switch_->publish_state(value & 0x10);
  }

  // Load Shutdown Time (select)
  if (this->load_shutdown_time_select_ != nullptr) {
    this->load_shutdown_time_select_->traits.set_options({"8s", "32s", "16s", "64s"});

    uint8_t value;
    this->read_register(IP5306_REG_SYS_CTL2, &value, 1);
    std::string option;
    switch ((value >> 2) & 0x03) {
      case 0x00:
        option = "8s";
        break;
      case 0x01:
        option = "32s";
        break;
      case 0x02:
        option = "16s";
        break;
      case 0x03:
        option = "64s";
        break;
      default:
        option = "";
        ESP_LOGW(TAG, "Unknown Load Shutdown Time value: 0x%02X", (value >> 2) & 0x03);
    }
    this->load_shutdown_time_select_->publish_state(option);
  }

  // Charge Cutoff Voltage (select)
  if (this->charge_cutoff_voltage_select_ != nullptr) {
    this->charge_cutoff_voltage_select_->traits.set_options({"4.2V", "4.3V", "4.35V", "4.4V"});

    uint8_t value;
    this->read_register(IP5306_REG_CHARGER_CTL1, &value, 1);
    std::string option;
    switch (value & 0x03) {
      case 0x00:
        option = "4.2V";
        break;
      case 0x01:
        option = "4.3V";
        break;
      case 0x02:
        option = "4.35V";
        break;
      case 0x03:
        option = "4.4V";
        break;
      default:
        option = "";
        ESP_LOGW(TAG, "Unknown Charge Cutoff Voltage value: 0x%02X", value & 0x03);
    }
    this->charge_cutoff_voltage_select_->publish_state(option);
  }

  // Charge Termination Current (select)
  if (this->charge_termination_current_select_ != nullptr) {
    this->charge_termination_current_select_->traits.set_options({"200mA", "400mA", "500mA", "600mA"});

    uint8_t value;
    this->read_register(IP5306_REG_CHARGER_CTL2, &value, 1);
    std::string option;
    switch ((value >> 2) & 0x03) {
      case 0x00:
        option = "200mA";
        break;
      case 0x01:
        option = "400mA";
        break;
      case 0x02:
        option = "500mA";
        break;
      case 0x03:
        option = "600mA";
        break;
      default:
        option = "";
        ESP_LOGW(TAG, "Unknown Charge Termination Current value: 0x%02X", (value >> 2) & 0x03);
    }
    this->charge_termination_current_select_->publish_state(option);
  }
}

void IP5306::update() {
  uint8_t data[1];
  
  if (this->battery_level_ != nullptr) {
    if (this->read_register(IP5306_REG_LEVEL, data, 1) == i2c::ERROR_OK) {
      float value = 0;
      switch (data[0] & 0xF0) {
        case 0xE0:
          value = 25;
          break;
        case 0xC0:
          value = 50;
          break;
        case 0x80:
          value = 75;
          break;
        case 0x00:
          value = 100;
          break;
        default:
          value = 0;
      }
      this->battery_level_->publish_state(value);
    } else {
      ESP_LOGE(TAG, "Failed to read battery level.");
    }
  }
}
