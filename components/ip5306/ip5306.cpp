#include "ip5306.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace ip5306 {

static const char *const TAG = "ip5306";

// Registers
static const uint8_t IP5306_REG_SYS_CTL0 = 0x00;
static const uint8_t IP5306_REG_SYS_CTL2 = 0x02;
static const uint8_t IP5306_REG_CHARGER_CTL1 = 0x21;
static const uint8_t IP5306_REG_CHARGER_CTL2 = 0x22;

static const uint8_t IP5306_REG_READ0 = 0x70;
static const uint8_t IP5306_REG_READ1 = 0x71;
static const uint8_t IP5306_REG_LEVEL = 0x78;

void IP5306Switch::write_state(bool state) {
  this->parent_->handle_switch_write(this->type_, state);
}

void IP5306Select::control(const std::string &value) {
  this->parent_->handle_select_control(this->type_, value);
}

float IP5306::get_setup_priority() const { return setup_priority::IO; }

void IP5306::setup() {
  ESP_LOGD(TAG, "Setting up ip5306...");
  
  // Default init - just check connection
  uint8_t dummy;
  if (this->read_register(IP5306_REG_SYS_CTL0, &dummy, 1) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "setup failed - read SYS_CTL0");
    this->mark_failed();
    return;
  }

  // --- Initialize Options for Selects ---
  if (this->light_load_shutdown_time_select_) {
    this->light_load_shutdown_time_select_->traits.set_options({"8s", "32s", "16s", "64s"});
    // Read current value
    uint8_t val;
    this->read_register(IP5306_REG_SYS_CTL2, &val, 1);
    uint8_t idx = (val >> 2) & 0x03;
    const auto &opts = this->light_load_shutdown_time_select_->traits.get_options();
    if (idx < opts.size()) this->light_load_shutdown_time_select_->publish_state(opts[idx]);
  }

  if (this->charge_cutoff_voltage_select_) {
    this->charge_cutoff_voltage_select_->traits.set_options({"4.2V", "4.3V", "4.35V", "4.4V"});
    uint8_t val;
    this->read_register(IP5306_REG_CHARGER_CTL1, &val, 1);
    uint8_t idx = (val >> 0) & 0x03;
    const auto &opts = this->charge_cutoff_voltage_select_->traits.get_options();
    if (idx < opts.size()) this->charge_cutoff_voltage_select_->publish_state(opts[idx]);
  }

  if (this->charge_termination_current_select_) {
    this->charge_termination_current_select_->traits.set_options({"200mA", "400mA", "500mA", "600mA"});
    uint8_t val;
    this->read_register(IP5306_REG_CHARGER_CTL2, &val, 1);
    uint8_t idx = (val >> 2) & 0x03;
    const auto &opts = this->charge_termination_current_select_->traits.get_options();
    if (idx < opts.size()) this->charge_termination_current_select_->publish_state(opts[idx]);
  }

  // --- Initialize Switches ---
  if (this->charger_enable_switch_ || this->low_load_shutdown_switch_) {
     uint8_t val;
     this->read_register(IP5306_REG_SYS_CTL0, &val, 1);
     
     if (this->charger_enable_switch_) {
        // Bit 4: Charger Enable (1=En, 0=Dis)
        this->charger_enable_switch_->publish_state(val & (1 << 4));
     }
     if (this->low_load_shutdown_switch_) {
        // Bit 1: Low Load Shutdown (1=En, 0=Dis)
        this->low_load_shutdown_switch_->publish_state(val & (1 << 1));
     }
  }
}

void IP5306::update() {
  uint8_t data[2];
  if (this->battery_level_ != nullptr) {
    if (this->read_register(IP5306_REG_LEVEL, data, 1) == i2c::ERROR_OK) {
      float value = 0;
      switch (data[0] & 0xF0) {
        case 0xE0: value = 25; break;
        case 0xC0: value = 50; break;
        case 0x80: value = 75; break;
        case 0x00: value = 100; break;
      }
      this->battery_level_->publish_state(value);
    }
  }
  if (this->read_register(IP5306_REG_READ0, data, 2) == i2c::ERROR_OK) {
    if (this->charger_connected_ != nullptr)
      this->charger_connected_->publish_state(data[0] & 0x08);
    if (this->charge_full_ != nullptr)
      this->charge_full_->publish_state(data[1] & 0x08);
  }
}

void IP5306::handle_switch_write(IP5306SwitchType type, bool state) {
  switch (type) {
    case IP5306_SWITCH_CHARGER_ENABLE:
      // SYS_CTL0 Bit 4
      this->update_register_bit_(IP5306_REG_SYS_CTL0, (1 << 4), state);
      if (this->charger_enable_switch_) this->charger_enable_switch_->publish_state(state);
      break;
    case IP5306_SWITCH_LOW_LOAD_SHUTDOWN:
      // SYS_CTL0 Bit 1
      this->update_register_bit_(IP5306_REG_SYS_CTL0, (1 << 1), state);
      if (this->low_load_shutdown_switch_) this->low_load_shutdown_switch_->publish_state(state);
      break;
  }
}

void IP5306::handle_select_control(IP5306SelectType type, const std::string &value) {
  // Simple helper to find index
  int index = -1;
  const std::vector<std::string> *opts = nullptr;
  select::Select *comp = nullptr;

  if (type == IP5306_SELECT_LIGHT_LOAD_SHUTDOWN_TIME) {
      comp = this->light_load_shutdown_time_select_;
  } else if (type == IP5306_SELECT_CHARGE_CUTOFF_VOLTAGE) {
      comp = this->charge_cutoff_voltage_select_;
  } else if (type == IP5306_SELECT_CHARGE_TERMINATION_CURRENT) {
      comp = this->charge_termination_current_select_;
  }

  if (!comp) return;
  opts = &comp->traits.get_options();
  for (size_t i = 0; i < opts->size(); i++) {
      if ((*opts)[i] == value) { index = i; break; }
  }
  if (index == -1) return;

  switch (type) {
    case IP5306_SELECT_LIGHT_LOAD_SHUTDOWN_TIME:
      // SYS_CTL2 Bits 2,3 -> index
      this->update_register_value_(IP5306_REG_SYS_CTL2, 0x0C, 2, index);
      break;
    case IP5306_SELECT_CHARGE_CUTOFF_VOLTAGE:
      // CHARGER_CTL1 Bits 0,1 -> index
      this->update_register_value_(IP5306_REG_CHARGER_CTL1, 0x03, 0, index);
      break;
    case IP5306_SELECT_CHARGE_TERMINATION_CURRENT:
      // CHARGER_CTL2 Bits 2,3 -> index
      this->update_register_value_(IP5306_REG_CHARGER_CTL2, 0x0C, 2, index);
      break;
  }
  comp->publish_state(value);
}

void IP5306::update_register_bit_(uint8_t reg, uint8_t mask, bool value) {
    uint8_t cur;
    if (this->read_register(reg, &cur, 1) != i2c::ERROR_OK) return;
    if (value) cur |= mask; else cur &= ~mask;
    this->write_register(reg, &cur, 1);
}

void IP5306::update_register_value_(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value) {
    uint8_t cur;
    if (this->read_register(reg, &cur, 1) != i2c::ERROR_OK) return;
    cur &= ~mask; // Clear bits
    cur |= (value << shift) & mask; // Set bits
    this->write_register(reg, &cur, 1);
}

}  // namespace ip5306
}  // namespace esphome