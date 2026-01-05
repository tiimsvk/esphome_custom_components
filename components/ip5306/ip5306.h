#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/select/select.h"

namespace esphome {
namespace ip5306 {

enum IP5306SwitchType {
  IP5306_SWITCH_LOW_LOAD_SHUTDOWN,
  IP5306_SWITCH_CHARGER_ENABLE,
  IP5306_SWITCH_CHARGE_CONTROL,
  IP5306_SWITCH_BOOST_ENABLE,
  IP5306_SWITCH_SOFTWARE_SHUTDOWN,
  IP5306_SWITCH_LOW_BAT_SHUTDOWN, 
  IP5306_SWITCH_BOOST_ON_LOAD,
  IP5306_SWITCH_SHORT_PRESS_BOOST,
  IP5306_SWITCH_BUTTON_SHUTDOWN
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

  void set_battery_level(sensor::Sensor *sensor) { this->battery_level_ = sensor; }
  void set_current_sensor(sensor::Sensor *current_sensor) { this->current_sensor_ = current_sensor; }
  void set_load_status_sensor(text_sensor::TextSensor *sensor) { this->load_status_sensor_ = sensor; }
  void set_charger_connected(binary_sensor::BinarySensor *sensor) { this->charger_connected_ = sensor; }
  void set_charge_full(binary_sensor::BinarySensor *sensor) { this->charge_full_ = sensor; }

  // Settery pre switche
  void set_low_load_shutdown_switch(IP5306Switch *sw) { 
      sw->set_type(IP5306_SWITCH_LOW_LOAD_SHUTDOWN);
      this->switches_.push_back(sw);
  }
  void set_charger_enable_switch(IP5306Switch *sw) { 
      sw->set_type(IP5306_SWITCH_CHARGER_ENABLE);
      this->switches_.push_back(sw);
  }
  void set_charge_control_switch(IP5306Switch *sw) { 
      sw->set_type(IP5306_SWITCH_CHARGE_CONTROL);
      this->switches_.push_back(sw);
  }
  void set_boost_control_switch(IP5306Switch *sw) { 
      sw->set_type(IP5306_SWITCH_BOOST_ENABLE);
      this->switches_.push_back(sw);
  }
  void set_software_shutdown_switch(IP5306Switch *sw) { 
      sw->set_type(IP5306_SWITCH_SOFTWARE_SHUTDOWN);
      this->switches_.push_back(sw);
  }

  // Settery pre selecty
  void set_load_shutdown_time_select(IP5306Select *sel) {
    sel->set_type(IP5306_SELECT_LOAD_SHUTDOWN_TIME);
    this->selects_.push_back(sel);
  }
  void set_charge_cutoff_voltage_select(IP5306Select *sel) {
    sel->set_type(IP5306_SELECT_CHARGE_CUTOFF_VOLTAGE);
    this->selects_.push_back(sel);
  }
  void set_charge_termination_current_select(IP5306Select *sel) {
    sel->set_type(IP5306_SELECT_CHARGE_TERMINATION_CURRENT);
    this->selects_.push_back(sel);
  }

  void write_register_bit(uint8_t reg, uint8_t mask, bool value);
  void write_register_bits(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value);

 private:
  sensor::Sensor *battery_level_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  text_sensor::TextSensor *load_status_sensor_{nullptr};
  binary_sensor::BinarySensor *charger_connected_{nullptr};
  binary_sensor::BinarySensor *charge_full_{nullptr};

  std::vector<IP5306Switch *> switches_;
  std::vector<IP5306Select *> selects_;

  // Premenne pre ukladanie stavu (Spam fix)
  float last_battery_level_{-1};
  float last_current_{-1};
  std::string last_load_status_{""};
  int last_charger_connected_{-1}; 
  int last_charge_full_{-1};
  
  // --- NOVE: Debounce pre bateriu ---
  float pending_battery_level_{-1}; // Hodnota, ktora caka na potvrdenie
  int battery_debounce_counter_{0}; // Pocitadlo opakovani
};

}  // namespace ip5306
}  // namespace esphome
