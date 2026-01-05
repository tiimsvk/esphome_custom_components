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
static const uint8_t IP5306_REG_READ2 = 0x72;
static const uint8_t IP5306_REG_LEVEL = 0x78;

void IP5306::setup() {
  ESP_LOGCONFIG(TAG, "Setting up IP5306...");
  
  uint8_t sys0, chg0, chg1, chg2, sys2;
  
  // Nacitanie a nastavenie Switchov
  if (this->read_register(IP5306_REG_SYS_CTL0, &sys0, 1) == i2c::ERROR_OK) {
      if (this->low_load_shutdown_switch_ != nullptr)
        this->low_load_shutdown_switch_->publish_state(sys0 & 0x02);
      if (this->charger_enable_switch_ != nullptr)
        this->charger_enable_switch_->publish_state(sys0 & 0x10);
      if (this->boost_control_switch_ != nullptr)
        this->boost_control_switch_->publish_state(sys0 & 0x20);
  }

  if (this->read_register(IP5306_REG_CHARGER_CTL0, &chg0, 1) == i2c::ERROR_OK) {
      if (this->charge_control_switch_ != nullptr)
        this->charge_control_switch_->publish_state(chg0 & 0x10);
  }

  if (this->software_shutdown_switch_ != nullptr)
     this->software_shutdown_switch_->publish_state(false);

  // --- Nastavenie Selectov ---
  
  // 1. Load Shutdown Time
  if (this->load_shutdown_time_select_ != nullptr) {
    if (this->read_register(IP5306_REG_SYS_CTL2, &sys2, 1) == i2c::ERROR_OK) {
       uint8_t val = (sys2 >> 2) & 0x03;
       std::string opts[] = {"8s", "32s", "16s", "64s"};
       if (val < 4) this->load_shutdown_time_select_->publish_state(opts[val]);
    }
  }

  // 2. Charge Cutoff Voltage
  if (this->charge_cutoff_voltage_select_ != nullptr) {
    if (this->read_register(IP5306_REG_CHARGER_CTL1, &chg1, 1) == i2c::ERROR_OK) {
       uint8_t val = chg1 & 0x03;
       std::string opts[] = {"4.2V", "4.3V", "4.35V", "4.4V"};
       if (val < 4) this->charge_cutoff_voltage_select_->publish_state(opts[val]);
    }
  }

  // 3. Charge Termination Current
  if (this->charge_termination_current_select_ != nullptr) {
    if (this->read_register(IP5306_REG_CHARGER_CTL2, &chg2, 1) == i2c::ERROR_OK) {
       uint8_t val = (chg2 >> 2) & 0x03;
       std::string opts[] = {"200mA", "400mA", "500mA", "600mA"};
       if (val < 4) this->charge_termination_current_select_->publish_state(opts[val]);
    }
  }
}

void IP5306::update() {
  uint8_t status_0;
  
  // 1. Charging Status & Full (Podla tvojho funkcneho kodu)
  if (this->read_register(IP5306_REG_READ0, &status_0, 1) == i2c::ERROR_OK) {
    // 0x08 = Bit 3 (Charging)
    // 0x10 = Bit 4 (Charge Full)
    bool is_charging = status_0 & 0x08; 
    bool is_full = status_0 & 0x10;

    if (this->charger_connected_ != nullptr) {
      // Ak nabija alebo je full, povazujeme nabijacku za pripojenu
      bool connected = is_charging || is_full;
      if (this->first_update_ || this->last_charger_connected_ != connected) {
        this->charger_connected_->publish_state(connected);
        this->last_charger_connected_ = connected;
      }
    }

    if (this->charge_full_ != nullptr) {
      if (this->first_update_ || this->last_charge_full_ != is_full) {
        this->charge_full_->publish_state(is_full);
        this->last_charge_full_ = is_full;
      }
    }
  }

  // 2. Battery Level
  if (this->battery_level_ != nullptr) {
    uint8_t data_battery;
    if (this->read_register(IP5306_REG_LEVEL, &data_battery, 1) == i2c::ERROR_OK) {
      float value = 0;
      switch (data_battery & 0xF0) {
        case 0xE0: value = 25; break;
        case 0xC0: value = 50; break;
        case 0x80: value = 75; break;
        case 0x00: value = 100; break;
        default: value = 0; break; 
      }
      
      if (this->first_update_ || this->last_battery_level_ != value) {
        this->battery_level_->publish_state(value);
        this->last_battery_level_ = value;
      }
    }
  }

  // 3. Load Status (Text Sensor)
  if (this->load_status_sensor_ != nullptr) {
      uint8_t status_2;
      if (this->read_register(IP5306_REG_READ2, &status_2, 1) == i2c::ERROR_OK) {
          bool light_load = (status_2 >> 2) & 0x01;
          std::string status = light_load ? "Light Load" : "Heavy Load";
          
          if (this->first_update_ || this->last_load_status_ != status) {
              this->load_status_sensor_->publish_state(status);
              this->last_load_status_ = status;
          }
      }
  }

  // 4. Output Current
  if (this->current_sensor_ != nullptr) {
    uint8_t data_curr;
    if (this->read_register(IP5306_REG_READ1, &data_curr, 1) == i2c::ERROR_OK) {
      float current = (data_curr & 0x7F) * 0.02; 
      
      if (this->first_update_ || std::abs(this->last_current_ - current) > 0.005) {
          this->current_sensor_->publish_state(current);
          this->last_current_ = current;
      }
    }
  }
  
  this->first_update_ = false;
}

void IP5306::write_register_bit(uint8_t reg, uint8_t mask, bool value) {
  uint8_t current;
  if (this->read_register(reg, &current, 1) != i2c::ERROR_OK) return;
  if (value) current |= mask; else current &= ~mask;
  this->write_register(reg, &current, 1);
}

void IP5306::write_register_bits(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value) {
  uint8_t current;
  if (this->read_register(reg, &current, 1) != i2c::ERROR_OK) return;
  current &= ~mask;
  current |= (value << shift);
  this->write_register(reg, &current, 1);
}

void IP5306Switch::write_state(bool state) {
  if (this->parent_ == nullptr) return;

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
      this->parent_->write_register_bit(IP5306_REG_SYS_CTL0, 0x20, state);
      break;
    case IP5306_SWITCH_SOFTWARE_SHUTDOWN:
      if (state) {
          this->parent_->write_register_bit(IP5306_REG_SYS_CTL1, 0x08, false);
          this->publish_state(false);
      }
      break;
  }
  if (this->type_ != IP5306_SWITCH_SOFTWARE_SHUTDOWN)
      this->publish_state(state);
}

void IP5306Select::control(const std::string &value) {
  if (this->parent_ == nullptr) return;

  int index = 0;
  auto &opts = this->traits.get_options();
  auto it = std::find(opts.begin(), opts.end(), value);
  if (it != opts.end()) {
      index = std::distance(opts.begin(), it);
  }

  switch (this->type_) {
    case IP5306_SELECT_LOAD_SHUTDOWN_TIME:
       this->parent_->write_register_bits(IP5306_REG_SYS_CTL2, 0x0C, 2, index);
       break;
    case IP5306_SELECT_CHARGE_CUTOFF_VOLTAGE:
       this->parent_->write_register_bits(IP5306_REG_CHARGER_CTL1, 0x03, 0, index);
       break;
    case IP5306_SELECT_CHARGE_TERMINATION_CURRENT:
       this->parent_->write_register_bits(IP5306_REG_CHARGER_CTL2, 0x0C, 2, index);
       break;
  }
  
  this->publish_state(value);
}

float IP5306::get_setup_priority() const {
  return setup_priority::HARDWARE;
}

}  // namespace ip5306
}  // namespace esphome
