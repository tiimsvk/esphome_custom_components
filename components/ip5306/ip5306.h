#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/select/select.h"

namespace esphome {
namespace ip5306 {

enum IP5306SwitchType {
  IP5306_SWITCH_LOW_LOAD_SHUTDOWN,
  IP5306_SWITCH_CHARGER_ENABLE,
  IP5306_SWITCH_CHARGE_CONTROL,  // NEW: Charge control
};

enum IP5306SelectType {
  IP5306_SELECT_LOAD_SHUTDOWN_TIME,
  IP5306_SELECT_CHARGE_CUTOFF_VOLTAGE,
  IP5306_SELECT_CHARGE_TERMINATION_CURRENT,
};

class IP5306;

class IP5306Switch : public switch_::Switch, public Parented<IP5306> {
 public:
  void set_type(IP5306SwitchType type) { this->type_ = type; }
  void write_state(bool state) override;

 private:
  IP5306SwitchType type_;
};

class IP5306Select : public select::Select, public Parented<IP5306> {
 public:
  void set_type(IP5306SelectType type) { this->type_ = type; }
  void control(const std::string &value) override;

 private:
  IP5306SelectType type_;
};

class IP5306 : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;

  float get_setup_priority() const override;

  void set_low_load_shutdown_switch(IP5306Switch *low_load_shutdown) {
    this->low_load_shutdown_switch_ = low_load_shutdown;
  }
  void set_charger_enable_switch(IP5306Switch *charger_enable) {
    this->charger_enable_switch_ = charger_enable;
  }
  void set_charge_control_switch(IP5306Switch *charge_control) {
    this->charge_control_switch_ = charge_control;
  }
  void set_load_shutdown_time_select(IP5306Select *load_shutdown_time) {
    this->load_shutdown_time_select_ = load_shutdown_time;
  }
  void set_charge_cutoff_voltage_select(IP5306Select *charge_cutoff_voltage) {
    this->charge_cutoff_voltage_select_ = charge_cutoff_voltage;
  }
  void set_charge_termination_current_select(IP5306Select *charge_termination_current) {
    this->charge_termination_current_select_ = charge_termination_current;
  }

  // Helper methods
  void write_register_bit(uint8_t reg, uint8_t mask, bool value);
  void write_register_bits(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value);

 private:
  IP5306Switch *low_load_shutdown_switch_{nullptr};
  IP5306Switch *charger_enable_switch_{nullptr};
  IP5306Switch *charge_control_switch_{nullptr};
  IP5306Select *load_shutdown_time_select_{nullptr};
  IP5306Select *charge_cutoff_voltage_select_{nullptr};
  IP5306Select *charge_termination_current_select_{nullptr};
};

}  // namespace ip5306
}  // namespace esphome
