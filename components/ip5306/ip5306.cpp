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

void IP5306::setup() {
  ESP_LOGCONFIG(TAG, "Setting up IP5306...");

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

  if (this->load_shutdown_time_select_ != nullptr) {
    this->load_shutdown_time_select_->traits.set_options({"8s", "32s", "16s", "64s"});

    uint8_t value;
    this->read_register(IP5306_REG_SYS_CTL2, &value, 1);
    this->load_shutdown_time_select_->publish_state(std::to_string((value >> 2) & 0x03));
  }

  if (this->charge_cutoff_voltage_select_ != nullptr) {
    this->charge_cutoff_voltage_select_->traits.set_options({"4.2V", "4.3V", "4.35V", "4.4V"});

    uint8_t value;
    this->read_register(IP5306_REG_CHARGER_CTL1, &value, 1);
    this->charge_cutoff_voltage_select_->publish_state(std::to_string(value & 0x03));
  }

  if (this->charge_termination_current_select_ != nullptr) {
    this->charge_termination_current_select_->traits.set_options({"200mA", "400mA", "500mA", "600mA"});

    uint8_t value;
    this->read_register(IP5306_REG_CHARGER_CTL2, &value, 1);
    this->charge_termination_current_select_->publish_state(std::to_string((value >> 2) & 0x03));
  }
}

void IP5306::update() {
  uint8_t data[2];
  if (this->read_register(IP5306_REG_READ0, data, 1) == i2c::ERROR_OK) {
    if (this->charger_connected_ != nullptr) {
      this->charger_connected_->publish_state(data[0] & 0x08);
    }

    if (this->charge_full_ != nullptr) {
      this->charge_full_->publish_state(data[0] & 0x10);
    }
  }
}

void IP5306::write_register_bit(uint8_t reg, uint8_t mask, bool value) {
  uint8_t current;
  this->read_register(reg, &current, 1);

  if (value) {
    current |= mask;
  } else {
    current &= ~mask;
  }

  this->write_register(reg, &current, 1);
}

void IP5306::write_register_bits(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value) {
  uint8_t current;
  this->read_register(reg, &current, 1);

  current &= ~mask;            // Clear target bits
  current |= (value << shift); // Set new value

  this->write_register(reg, &current, 1);
}

void IP5306Switch::write_state(bool state) {
  if (this->type_ == IP5306_SWITCH_LOW_LOAD_SHUTDOWN) {
    this->parent_->write_register_bit(IP5306_REG_SYS_CTL0, 0x02, state);
  } else if (this->type_ == IP5306_SWITCH_CHARGER_ENABLE) {
    this->parent_->write_register_bit(IP5306_REG_SYS_CTL0, 0x10, state);
  } else if (this->type_ == IP5306_SWITCH_CHARGE_CONTROL) {
    this->parent_->write_register_bit(IP5306_REG_CHARGER_CTL0, 0x10, state);
  }
  this->publish_state(state);
}

void IP5306Select::control(const std::string &value) {
  uint8_t val = atoi(value.c_str());

  if (this->type_ == IP5306_SELECT_LOAD_SHUTDOWN_TIME) {
    this->parent_->write_register_bits(IP5306_REG_SYS_CTL2, 0x0C, 2, val);
  } else if (this->type_ == IP5306_SELECT_CHARGE_CUTOFF_VOLTAGE) {
    this->parent_->write_register_bits(IP5306_REG_CHARGER_CTL1, 0x03, 0, val);
  } else if (this->type_ == IP5306_SELECT_CHARGE_TERMINATION_CURRENT) {
    this->parent_->write_register_bits(IP5306_REG_CHARGER_CTL2, 0x0C, 2, val);
  }
  this->publish_state(value);
}

float IP5306::get_setup_priority() const {
  return setup_priority::HARDWARE;  // Nastavte prioritu na "HARDWARE" pre hardvérové komponenty
}

}  // namespace ip5306
}  // namespace esphome
