#include "ip5306.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ip5306 {

static const char *const TAG = "ip5306";

static const uint8_t IP5306_REG_SYS_CTL0 = 0x00;
static const uint8_t IP5306_REG_SYS_CTL1 = 0x01;
static const uint8_t IP5306_REG_SYS_CTL2 = 0x02;
static const uint8_t IP5306_REG_CHARGER_CTL0 = 0x20;
static const uint8_t IP5306_REG_CHARGER_CTL1 = 0x21;
static const uint8_t IP5306_REG_CHARGER_CTL2 = 0x22;
static const uint8_t IP5306_REG_READ0 = 0x70;
static const uint8_t IP5306_REG_READ1 = 0x71;
static const uint8_t IP5306_REG_LEVEL = 0x72;

void IP5306::setup() {
  ESP_LOGCONFIG(TAG, "Setting up IP5306...");
  
  // Configure charge cutoff voltage (default: 4.2V)
  this->write_register_bits(IP5306_REG_CHARGER_CTL1, 0x03, 0, 0x00);  // Default to 4.2V

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

  if (this->boost_control_switch_ != nullptr) {
    uint8_t value;
    this->read_register(IP5306_REG_SYS_CTL0, &value, 1);
    this->boost_control_switch_->publish_state(value & 0x01);  // Boost enabled/disabled
  }

  // Load Shutdown Time (select)
  if (this->load_shutdown_time_select_ != nullptr) {
    this->load_shutdown_time_select_->traits.set_options({"8s", "32s", "16s", "64s"});

    uint8_t value;
    this->read_register(IP5306_REG_SYS_CTL2, &value, 1);
    // Map value to string options
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
    // Map value to string options
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
    // Map value to string options
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
  uint8_t data_status[1];
  if (this->read_register(IP5306_REG_READ0, data_status, 1) == i2c::ERROR_OK) {
    if (this->charger_connected_ != nullptr) {
      this->charger_connected_->publish_state(data_status[0] & 0x08);
    }

    if (this->charge_full_ != nullptr) {
      this->charge_full_->publish_state(data_status[0] & 0x10);
    }
  }

  uint8_t data_battery[1];
  if (this->battery_level_ != nullptr) {
    if (this->read_register(IP5306_REG_LEVEL, data_battery, 1) == i2c::ERROR_OK) {
      float value = 0;
      // Mapovanie batériovej úrovne
      switch (data_battery[0] & 0xF0) {
        case 0xE0:
          value = 25;  // 25%
          break;
        case 0xC0:
          value = 50;  // 50%
          break;
        case 0x80:
          value = 75;  // 75%
          break;
        case 0x00:
          value = 100; // 100%
          break;
        default:
          value = 0;
          ESP_LOGW(TAG, "Unknown battery level value: 0x%02X", data_battery[0]);
      }

      // Publikuj iba v prípade, že sa hodnota zmenila
      if (this->last_battery_level_ != value) {
        this->last_battery_level_ = value;
        this->battery_level_->publish_state(value);
      }
    } else {
      ESP_LOGE(TAG, "Failed to read battery level.");
    }  
  // Read battery voltage and publish state
  if (this->voltage_sensor_ != nullptr) {
    uint8_t data_volt[1];
    if (this->read_register(IP5306_REG_READ0, data_volt, 1) == i2c::ERROR_OK) {
      float voltage = (data_volt[0] & 0x7F) * 0.05;  // Voltage in volts
      this->voltage_sensor_->publish_state(voltage);
    } else {
      ESP_LOGE(TAG, "Failed to read battery voltage.");
    }
  }

  // Read output current and publish state
  if (this->current_sensor_ != nullptr) {
    uint8_t data_curr[1];
    if (this->read_register(IP5306_REG_READ1, data_curr, 1) == i2c::ERROR_OK) {
      float current = (data_curr[0] & 0x7F) * 0.02;  // Current in amps
      this->current_sensor_->publish_state(current);
    } else {
      ESP_LOGE(TAG, "Failed to read output current.");
    }
  }
}

void IP5306::shutdown() {
  if (this->write_register_bit(IP5306_REG_SYS_CTL1, 0x08, false) == i2c::ERROR_OK) {
    ESP_LOGD(TAG, "System shutdown command sent.");
  } else {
    ESP_LOGE(TAG, "Failed to send shutdown command.");
  }
}

void IP5306::write_register_bit(uint8_t reg, uint8_t mask, bool value) {
  uint8_t current;
  if (this->read_register(reg, &current, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to read register 0x%02X before writing!", reg);
    return;
  }

  if (value) {
    current |= mask;
  } else {
    current &= ~mask;
  }

  if (this->write_register(reg, &current, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to write to register 0x%02X!", reg);
  }
}

void IP5306::write_register_bits(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value) {
  uint8_t current;
  if (this->read_register(reg, &current, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to read register 0x%02X before writing!", reg);
    return;
  }

  current &= ~mask;            // Clear target bits
  current |= (value << shift); // Set bits with new value

  if (this->write_register(reg, &current, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to write to register 0x%02X!", reg);
  }
}

void IP5306Switch::write_state(bool state) {
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "Parent component is null! Failed to set state.");
    return;
  }

  switch (this->type_) {
    case IP5306_SWITCH_LOW_LOAD_SHUTDOWN:
      this->parent_->write_register_bit(IP5306_REG_SYS_CTL0, 0x02, state);
      break;
    case IP5306_SWITCH_CHARGER_ENABLE:
      this->parent_->write_register_bit(IP5306_REG_SYS_CTL0, 0x10, state);
      break;
    case IP5306_SWITCH_CHARGE_CONTROL:
      this->parent_->write_register_bit(IP5306_REG_CHARGER_CTL0, 0x10, state);
      break;
    case IP5306_SWITCH_BOOST_ENABLE:
      this->parent_->write_register_bit(IP5306_REG_SYS_CTL0, 0x01, state);
      break;
    default:
      ESP_LOGE(TAG, "Unknown switch type.");
      return;
  }

  this->publish_state(state);
}

void IP5306Select::control(const std::string &value) {
  ESP_LOGD(TAG, "Parent is %snull for %s.", this->parent_ == nullptr ? "" : "not ", this->get_name().c_str());
  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "Parent is null for select component!");
    return;
  }

  int index = atoi(value.c_str());  // Ošetrenie výstupu
  ESP_LOGD(TAG, "Setting Select '%s' to index %d", this->get_name().c_str(), index);

  if (this->type_ == IP5306_SELECT_LOAD_SHUTDOWN_TIME) {
    this->parent_->write_register_bits(IP5306_REG_SYS_CTL2, 0x0C, 2, index);
  } else if (this->type_ == IP5306_SELECT_CHARGE_CUTOFF_VOLTAGE) {
    this->parent_->write_register_bits(IP5306_REG_CHARGER_CTL1, 0x03, 0, index);
  } else if (this->type_ == IP5306_SELECT_CHARGE_TERMINATION_CURRENT) {
    this->parent_->write_register_bits(IP5306_REG_CHARGER_CTL2, 0x0C, 2, index);
  } else {
    ESP_LOGE(TAG, "Unknown Select type!");
  }
  this->publish_state(value);
}

float IP5306::get_setup_priority() const {
  return setup_priority::HARDWARE;
}

}  // namespace ip5306
}  // namespace esphome
