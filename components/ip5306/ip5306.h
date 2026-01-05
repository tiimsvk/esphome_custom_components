#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h" // Pridane pre textovy senzor
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/select/select.h"

namespace esphome {
namespace ip5306 {

enum IP5306SwitchType {
  IP5306_SWITCH_LOW_LOAD_SHUTDOWN, // SYS_CTL0 bit 1
  IP5306_SWITCH_CHARGER_ENABLE,    // SYS_CTL0 bit 4
  IP5306_SWITCH_BOOST_ENABLE,      // SYS_CTL0 bit 0
  IP5306_SWITCH_BUTTON_SHUTDOWN,   // SYS_CTL1 bit 7 (ENABLE=1) -> button can shutdown
  IP5306_SWITCH_BOOST_ON_LOAD,     // SYS_CTL1 bit 2
  IP5306_SWITCH_LOW_BAT_SHUTDOWN,  // SYS_CTL1 bit 0
  IP5306_SWITCH_SHORT_PRESS_BOOST, // SYS_CTL2 bit 4
};

enum IP5306SelectType {
  IP5306_SELECT_LOAD_SHUTDOWN_TIME,
  IP5306_SELECT_CHARGE_CUTOFF_VOLTAGE,
  IP5306_SELECT_CHARGE_TERMINATION_CURRENT,
  IP5306_SELECT_BATTERY_VOLTAGE_SETTING, // Pre 0x24 (4.2V/4.3V/4.35V/4.4V nominal)
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
  // void set_voltage_sensor(sensor::Sensor *voltage_sensor) { this->voltage_sensor_ = voltage_sensor; } // ODSTRANENE
  void set_current_sensor(sensor::Sensor *current_sensor) { this->current_sensor_ = current_sensor; }
  
  void set_charger_connected(binary_sensor::BinarySensor *sensor) { this->charger_connected_ = sensor; }
  void set_charge_full(binary_sensor::BinarySensor *sensor) { this->charge_full_ = sensor; }
  
  void set_load_status_sensor(text_sensor::TextSensor *sensor) { this->load_status_sensor_ = sensor; } // NOVE

  // Switches setters
  void set_switch(IP5306Switch *sw, IP5306SwitchType type) {
    sw->set_type(type);
    this->switches_.push_back(sw);
  }

  // Selects setters
  void set_select(IP5306Select *sel, IP5306SelectType type) {
    sel->set_type(type);
    this->selects_.push_back(sel);
  }

  void write_register_bit(uint8_t reg, uint8_t mask, bool value);
  void write_register_bits(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value);

 private:
  // Sensors
  sensor::Sensor *battery_level_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  
  // Binary Sensors
  binary_sensor::BinarySensor *charger_connected_{nullptr};
  binary_sensor::BinarySensor *charge_full_{nullptr};
  
  // Text Sensors
  text_sensor::TextSensor *load_status_sensor_{nullptr};

  // Helper vectors for automation components
  std::vector<IP5306Switch *> switches_;
  std::vector<IP5306Select *> selects_;

  float last_battery_level_{-1};
};

}  // namespace ip5306
}  // namespace esphome
