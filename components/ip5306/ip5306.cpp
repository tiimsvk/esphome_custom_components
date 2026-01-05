#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/select/select.h"

namespace esphome {
namespace ip5306 {

enum IP5306SwitchType {
  IP5306_SWITCH_CHARGER_ENABLE,
  IP5306_SWITCH_LOW_LOAD_SHUTDOWN,
};

enum IP5306SelectType {
  IP5306_SELECT_LIGHT_LOAD_SHUTDOWN_TIME,
  IP5306_SELECT_CHARGE_CUTOFF_VOLTAGE,
  IP5306_SELECT_CHARGE_TERMINATION_CURRENT,
};

class IP5306;

class IP5306Switch : public switch_::Switch, public Parented<IP5306> {
 public:
  void write_state(bool state) override;
  void set_type(IP5306SwitchType type) { this->type_ = type; }
 private:
  IP5306SwitchType type_;
};

class IP5306Select : public select::Select, public Parented<IP5306> {
 public:
  void control(const std::string &value) override;
  void set_type(IP5306SelectType type) { this->type_ = type; }
 private:
  IP5306SelectType type_;
};

class IP5306 : public i2c::I2CDevice, public PollingComponent {
 public:
  IP5306() : PollingComponent(60000) {}
  void setup() override;
  void update() override;

  float get_setup_priority() const override;

  void set_battery_level(sensor::Sensor *sensor) { this->battery_level_ = sensor; }
  void set_charger_connected(binary_sensor::BinarySensor *sensor) { this->charger_connected_ = sensor; }
  void set_charge_full(binary_sensor::BinarySensor *sensor) { this->charge_full_ = sensor; }

  void set_switch(IP5306SwitchType type, IP5306Switch *sw);
  void set_select(IP5306SelectType type, IP5306Select *sel);

  void handle_switch_write(IP5306SwitchType type, bool state);
  void handle_select_control(IP5306SelectType type, const std::string &value);

 protected:
  sensor::Sensor *battery_level_{nullptr};
  binary_sensor::BinarySensor *charger_connected_{nullptr};
  binary_sensor::BinarySensor *charge_full_{nullptr};

  std::vector<IP5306Switch *> switches_;
  std::vector<IP5306Select *> selects_;

  void update_register_bit_(uint8_t reg, uint8_t mask, bool value);
  void update_register_value_(uint8_t reg, uint8_t mask, uint8_t shift, uint8_t value);
};

}  // namespace ip5306
}  // namespace esphome
