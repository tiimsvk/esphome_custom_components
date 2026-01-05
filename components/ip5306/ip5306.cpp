#include "ip5306.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ip5306 {

static const char *const TAG = "ip5306";

static const uint8_t IP5306_REG_SYS_0    = 0x00;
static const uint8_t IP5306_REG_SYS_1    = 0x01;
static const uint8_t IP5306_REG_SYS_2    = 0x02;
static const uint8_t IP5306_REG_CHG_0    = 0x20;
static const uint8_t IP5306_REG_CHG_1    = 0x21;
static const uint8_t IP5306_REG_CHG_2    = 0x22;
static const uint8_t IP5306_REG_READ_0   = 0x70;
static const uint8_t IP5306_REG_READ_1   = 0x71;
static const uint8_t IP5306_REG_READ_2   = 0x72;
static const uint8_t IP5306_REG_READ_4   = 0x78;

void IP5306::setup() {
  ESP_LOGCONFIG(TAG, "Setting up IP5306...");

  // --- 1. Nacitanie Selectov (Oprava chybajucich hodnot) ---
  for (auto *sel : this->selects_) {
      // Zistime o aky typ selectu ide a precitame register
      // Kedze IP5306Select trieda nema public getter na type_,
      // musime to spravit trosku inak, alebo spoliehat na to, ze su v poli.
      // Najlepsie riesenie je pridat getter do IP5306Select, ale tu to spravime
      // iteraciou a kontrolou "options" pre identifikaciu, alebo len precitame vsetky
      // registre a aplikujeme na vsetky selecty.
      
      // Kedze v setupe nevieme lahko rozlisit konkretny select instanciu bez gettera,
      // idealne by bolo, keby si to select nacital sam, alebo tu prejdeme vsetky moznosti.
      // Ale kedze mame pointery ulozene v poli, musime vediet, ktory je ktory.
      // Zjednodusenie: Precitame registre a ak najdeme zhodu v options, nastavime to.
      
      uint8_t val = 0;
      int index = 0;
      bool found = false;

      // Skusime identifikovat select podla jeho moznosti (trosku hack, ale funguje bez zmeny hlavicky Selectu)
      const auto &opts = sel->traits.get_options();
      
      if (opts.size() > 0 && opts[0] == "8s") { // LOAD_SHUTDOWN_TIME
          this->read_register(IP5306_REG_SYS_2, &val, 1);
          index = (val >> 2) & 0x03; // Bity 2,3
          found = true;
      } 
      else if (opts.size() > 0 && opts[0] == "4.15V") { // CHARGE_CUTOFF_VOLTAGE
           this->read_register(IP5306_REG_CHG_1, &val, 1);
           index = val & 0x03; // Bity 0,1
           found = true;
      }
      else if (opts.size() > 0 && opts[0] == "200mA") { // CHARGE_TERMINATION_CURRENT
           this->read_register(IP5306_REG_CHG_2, &val, 1);
           index = (val >> 2) & 0x03; // Bity 2,3
           found = true;
      }

      if (found && index < opts.size()) {
          sel->publish_state(opts[index]);
      }
  }
}

void IP5306::update() {
  uint8_t read0_data;
  uint8_t read1_data;
  uint8_t read2_data;
  uint8_t read4_data;

  // 1. Charger Status & Full
  if (this->read_register(IP5306_REG_READ_0, &read0_data, 1) == i2c::ERROR_OK) {
    bool charger_conn = (read0_data >> 3) & 0x01;
    bool charge_full_bit = (read0_data >> 4) & 0x01;
    
    // Binary senzory zvycajne nevadí ak sa spamujú, ESPHome ich filtruje interne,
    // ale pre istotu:
    if (this->charger_connected_ != nullptr) {
      this->charger_connected_->publish_state(charger_conn);
    }
    if (this->charge_full_ != nullptr) {
      this->charge_full_->publish_state(charger_conn && charge_full_bit);
    }
  }

  // 2. Output Current (Oprava spamovania)
  if (this->current_sensor_ != nullptr) {
    if (this->read_register(IP5306_REG_READ_1, &read1_data, 1) == i2c::ERROR_OK) {
        float current = (float)read1_data * 0.02f; // Vzorec z datasheetu
        
        // Kontrola zmeny (kvoli plavaniu hodnoty porovnavame s malou odchylkou alebo presne)
        if (std::abs(current - this->last_current_) > 0.001f) {
            this->current_sensor_->publish_state(current);
            this->last_current_ = current;
        }
    }
  }

  // 3. Load Status (Oprava spamovania)
  if (this->load_status_sensor_ != nullptr) {
      if (this->read_register(IP5306_REG_READ_2, &read2_data, 1) == i2c::ERROR_OK) {
          bool light_load_bit = (read2_data >> 2) & 0x01;
          std::string status = light_load_bit ? "Light Load" : "Heavy Load";
          
          if (status != this->last_load_status_) {
              this->load_status_sensor_->publish_state(status);
              this->last_load_status_ = status;
          }
      }
  }

  // 4. Battery Level (Uz malo ochranu, ale pre istotu)
  if (this->battery_level_ != nullptr) {
    if (this->read_register(IP5306_REG_READ_4, &read4_data, 1) == i2c::ERROR_OK) {
      float value = 0;
      uint8_t leds = read4_data & 0xF0;
      
      if (leds == 0x00) value = 100;
      else if (leds == 0x80) value = 75;
      else if (leds == 0xC0) value = 50;
      else if (leds == 0xE0) value = 25;
      else value = 0;

      if (this->last_battery_level_ != value) {
        this->last_battery_level_ = value;
        this->battery_level_->publish_state(value);
      }
    }
  }
}

// ... Zvyšok kódu (write_register_bit, control, atď.) zostáva rovnaký ...
// Len pre uplnost pripajam write_register_bit, aby bol subor validny na kompilaciu

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
    case IP5306_SWITCH_BOOST_ENABLE:     reg = IP5306_REG_SYS_0; mask = 0x01; break;
    case IP5306_SWITCH_LOW_LOAD_SHUTDOWN: reg = IP5306_REG_SYS_0; mask = 0x02; break;
    case IP5306_SWITCH_CHARGER_ENABLE:   reg = IP5306_REG_SYS_0; mask = 0x10; break;
    case IP5306_SWITCH_LOW_BAT_SHUTDOWN: reg = IP5306_REG_SYS_1; mask = 0x01; break;
    case IP5306_SWITCH_BOOST_ON_LOAD:    reg = IP5306_REG_SYS_1; mask = 0x04; break;
    case IP5306_SWITCH_BUTTON_SHUTDOWN:  reg = IP5306_REG_SYS_1; mask = 0x80; break;
    case IP5306_SWITCH_SHORT_PRESS_BOOST: reg = IP5306_REG_SYS_2; mask = 0x10; break;
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
       this->parent_->write_register_bits(IP5306_REG_SYS_2, 0x0C, 2, index);
       break;
    case IP5306_SELECT_CHARGE_CUTOFF_VOLTAGE:
       this->parent_->write_register_bits(IP5306_REG_CHG_1, 0x03, 0, index);
       break;
    case IP5306_SELECT_CHARGE_TERMINATION_CURRENT:
       this->parent_->write_register_bits(IP5306_REG_CHG_2, 0x0C, 2, index);
       break;
  }
  this->publish_state(value);
}

float IP5306::get_setup_priority() const {
  return setup_priority::HARDWARE;
}

}  // namespace ip5306
}  // namespace esphome
