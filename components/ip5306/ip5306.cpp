#include "ip5306.h"
#include "esphome/core/log.h"
#include <cmath> 

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

  uint8_t sys0, sys1, chg0;
  bool sys0_ok = (this->read_register(IP5306_REG_SYS_CTL0, &sys0, 1) == i2c::ERROR_OK);
  bool sys1_ok = (this->read_register(IP5306_REG_SYS_CTL1, &sys1, 1) == i2c::ERROR_OK);
  bool chg0_ok = (this->read_register(IP5306_REG_CHARGER_CTL0, &chg0, 1) == i2c::ERROR_OK);

  // --- Nacitanie stavov Switchov (Oprava) ---
  for (auto *sw : this->switches_) {
      switch (sw->get_type()) {
          case IP5306_SWITCH_LOW_LOAD_SHUTDOWN:
              if (sys0_ok) sw->publish_state(sys0 & 0x02);
              break;
          case IP5306_SWITCH_CHARGER_ENABLE:
              if (sys0_ok) sw->publish_state(sys0 & 0x10);
              break;
          case IP5306_SWITCH_BOOST_ENABLE:
              if (sys0_ok) sw->publish_state(sys0 & 0x20);
              break;
          case IP5306_SWITCH_CHARGE_CONTROL:
              if (chg0_ok) sw->publish_state(chg0 & 0x10);
              break;
          
          case IP5306_SWITCH_LOW_BAT_SHUTDOWN:
              if (sys1_ok) sw->publish_state(sys1 & 0x01);
              break;
          case IP5306_SWITCH_BOOST_ON_LOAD:
              if (sys1_ok) sw->publish_state(sys1 & 0x04);
              break;
          case IP5306_SWITCH_BUTTON_SHUTDOWN:
              if (sys1_ok) sw->publish_state(sys1 & 0x80);
              break;

          case IP5306_SWITCH_SOFTWARE_SHUTDOWN:
              sw->publish_state(false); // Vzdy false
              break;
          default:
              break;
      }
  }

  // --- Nacitanie hodnot Selectov ---
  for (auto *sel : this->selects_) {
      uint8_t val = 0;
      int index = 0;
      bool found = false;
      const auto &opts = sel->traits.get_options();
      if (opts.empty()) continue;

      if (opts[0] == "8s") { 
          if (this->read_register(IP5306_REG_SYS_CTL2, &val, 1) == i2c::ERROR_OK) {
            index = (val >> 2) & 0x03;
            found = true;
          }
      } 
      else if (opts[0] == "4.2V") { 
           if (this->read_register(IP5306_REG_CHARGER_CTL1, &val, 1) == i2c::ERROR_OK) {
             index = val & 0x03;
             found = true;
           }
      }
      else if (opts[0] == "200mA") { 
           if (this->read_register(IP5306_REG_CHARGER_CTL2, &val, 1) == i2c::ERROR_OK) {
             index = (val >> 2) & 0x03;
             found = true;
           }
      }

      if (found && index < (int)opts.size()) {
          sel->publish_state(opts[index]);
      }
  }
}

void IP5306::update() {
  uint8_t read1_data;
  uint8_t read2_data;
  uint8_t read_level;

  // 1. Charger Status
  uint8_t status_data[2];
  if (this->read_register(IP5306_REG_READ0, status_data, 2) == i2c::ERROR_OK) {
      bool connected = status_data[0] & 0x08; 
      bool full = status_data[1] & 0x08;      
      
      if (this->charger_connected_ != nullptr) {
          if (this->last_charger_connected_ != (int)connected) {
              this->charger_connected_->publish_state(connected);
              this->last_charger_connected_ = connected;
          }
      }
      if (this->charge_full_ != nullptr) {
          if (this->last_charge_full_ != (int)full) {
              this->charge_full_->publish_state(full);
              this->last_charge_full_ = full;
          }
      }
  }

  // 2. Output Current - S DEBOUNCINGOM
  if (this->current_sensor_ != nullptr) {
    if (this->read_register(IP5306_REG_READ1, &read1_data, 1) == i2c::ERROR_OK) {
        float raw_current = (float)read1_data * 0.02f;
        
        // Ak je zmena mensia ako 0.01A povazujeme to za sum a ignorujeme (ak nejde o uplnu nulu)
        if (std::abs(raw_current - this->pending_current_) < 0.005) {
             this->current_debounce_counter_++;
        } else {
             // Vyrazna zmena, resetujeme counter
             this->pending_current_ = raw_current;
             this->current_debounce_counter_ = 0;
        }

        // Pockame 20 cyklov kym potvrdime hodnotu
        if (this->current_debounce_counter_ >= 20) {
            // Posleme iba ak sa lisi od poslednej odoslanej
            if (std::abs(raw_current - this->last_current_) > 0.001) {
                this->current_sensor_->publish_state(raw_current);
                this->last_current_ = raw_current;
            }
            this->current_debounce_counter_ = 0;
        }
    }
  }

  // 3. Load Status
  if (this->load_status_sensor_ != nullptr) {
      if (this->read_register(IP5306_REG_READ2, &read2_data, 1) == i2c::ERROR_OK) {
          bool light_load_bit = (read2_data >> 2) & 0x01;
          std::string status = light_load_bit ? "Light Load" : "Heavy Load";
          
          if (status != this->last_load_status_) {
              this->load_status_sensor_->publish_state(status);
              this->last_load_status_ = status;
          }
      }
  }

  // 4. Battery Level - ZVYSENY DEBOUNCE (30 cyklov)
  if (this->battery_level_ != nullptr) {
    if (this->read_register(IP5306_REG_LEVEL, &read_level, 1) == i2c::ERROR_OK) {
      float raw_value = 0;
      switch (read_level & 0xF0) {
        case 0xE0: raw_value = 25; break;
        case 0xC0: raw_value = 50; break;
        case 0x80: raw_value = 75; break;
        case 0x00: raw_value = 100; break;
        default: raw_value = 0; break;
      }

      if (raw_value == this->last_battery_level_) {
           this->battery_debounce_counter_ = 0;
      } else {
          if (raw_value == this->pending_battery_level_) {
              this->battery_debounce_counter_++;
          } else {
              this->pending_battery_level_ = raw_value;
              this->battery_debounce_counter_ = 0;
          }

          // Zvysene na 30 cyklov pre vacsiu stabilitu
          if (this->battery_debounce_counter_ >= 30) {
              this->battery_level_->publish_state(raw_value);
              this->last_battery_level_ = raw_value;
              this->battery_debounce_counter_ = 0;
          }
      }
    }
  }
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
  current |= (value << shift) & mask;
  this->write_register(reg, &current, 1);
}

void IP5306Switch::write_state(bool state) {
  if (this->parent_ == nullptr) return;

  uint8_t reg = 0;
  uint8_t mask = 0;

  switch (this->type_) {
    case IP5306_SWITCH_BOOST_ENABLE:      reg = IP5306_REG_SYS_CTL0; mask = 0x20; break;
    case IP5306_SWITCH_LOW_LOAD_SHUTDOWN: reg = IP5306_REG_SYS_CTL0; mask = 0x02; break;
    case IP5306_SWITCH_CHARGER_ENABLE:    reg = IP5306_REG_SYS_CTL0; mask = 0x10; break;
    case IP5306_SWITCH_CHARGE_CONTROL:    reg = IP5306_REG_CHARGER_CTL0; mask = 0x10; break;
    
    case IP5306_SWITCH_LOW_BAT_SHUTDOWN:  reg = IP5306_REG_SYS_CTL1; mask = 0x01; break;
    case IP5306_SWITCH_BOOST_ON_LOAD:     reg = IP5306_REG_SYS_CTL1; mask = 0x04; break;
    case IP5306_SWITCH_BUTTON_SHUTDOWN:   reg = IP5306_REG_SYS_CTL1; mask = 0x80; break;
    
    case IP5306_SWITCH_SOFTWARE_SHUTDOWN:
       if (state) {
           this->parent_->write_register_bit(IP5306_REG_SYS_CTL0, 0x20, false); 
           this->publish_state(false);
       }
       return;

    default: return;
  }

  this->parent_->write_register_bit(reg, mask, state);
  this->publish_state(state);
}

void IP5306Select::control(const std::string &value) {
  if (this->parent_ == nullptr) return;
  
  if (this->traits.get_options().empty()) return;
  auto &opts = this->traits.get_options();
  auto it = std::find(opts.begin(), opts.end(), value);
  int index = (it != opts.end()) ? std::distance(opts.begin(), it) : 0;

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
