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
  IP5306_SWITCH_SOFTWARE_SHUTDOWN, // Pridane pre mapovanie
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
  
  // Novy text sensor
  void set_load_status_sensor(text_sensor::TextSensor *sensor) { this->load_status_sensor_ = sensor; }

  void set_charger_connected(binary_sensor::BinarySensor *sensor) { this->charger_connected_ = sensor; }
  void set_charge_full(binary_sensor::BinarySensor *sensor) { this->charge_full_ = sensor; }

  void set_low_load_shutdown_switch(IP5306Switch *sw) { 
      sw->set_type(IP5306_SWITCH_LOW_LOAD_SHUTDOWN);
      this->low_load_shutdown_switch_ = sw; 
  }
  void set_charger_enable_switch(IP5306Switch *sw) { 
      sw->set_type(IP5306_SWITCH_CHARGER_ENABLE);
      this->charger_enable_switch_ = sw; 
  }
  void set_charge_control_switch(IP5306Switch *sw) { 
      sw->set_type(IP5306_SWITCH_CHARGE_CONTROL);
      this->charge_control_switch_ = sw; 
  }
  void set_boost_control_switch(IP5306Switch *sw) { 
      sw->set_type(IP5306_SWITCH_BOOST_ENABLE);
      this->boost_control_switch_ = sw; 
  }
  void set_software_shutdown_switch(IP5306Switch *sw) { 
      sw->set_type(IP5306_SWITCH_SOFTWARE_SHUTDOWN);
      this->software_shutdown_switch_ = sw; 
  }

  void set_load_shutdown_time_select(IP5306Select *sel) {
    sel->set_type(IP5306_SELECT_LOAD_SHUTDOWN_TIME);
    this->load_shutdown_time_select_ = sel;
  }
  void set_charge_cutoff_voltage_select(IP5306Select *sel) {
    sel->set_type(IP5306_SELECT_CHARGE_CUTOFF_VOLTAGE);
    this->charge_cutoff_voltage_select_ = sel;
  }
  void set_charge_termination_current_select(IP5306Select *sel) {
    sel->set_type(IP5306_SELECT_CHARGE_TERMINATION_CURRENT);
    this->charge_termination_current_select_ = sel;
  }

  void write_register_bit(uint8_t reg, uint8_t mask, bool value);
  void write_register_bits(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value);

 private:
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *battery_level_{nullptr};
  text_sensor::TextSensor *load_status_sensor_{nullptr};
  
  binary_sensor::BinarySensor *charger_connected_{nullptr};
  binary_sensor::BinarySensor *charge_full_{nullptr};
  
  IP5306Switch *low_load_shutdown_switch_{nullptr};
  IP5306Switch *charger_enable_switch_{nullptr};
  IP5306Switch *charge_control_switch_{nullptr};
  IP5306Switch *boost_control_switch_{nullptr};
  IP5306Switch *software_shutdown_switch_{nullptr};
  
  IP5306Select *load_shutdown_time_select_{nullptr};
  IP5306Select *charge_cutoff_voltage_select_{nullptr};
  IP5306Select *charge_termination_current_select_{nullptr};
  
  float last_battery_level_{-1};
};

}  // namespace ip5306
}  // namespace esphome
