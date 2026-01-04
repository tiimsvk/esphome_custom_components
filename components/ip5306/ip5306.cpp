#include "ip5306.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace ip5306 {

static const char *const TAG = "ip5306";

// Registre
static const uint8_t IP5306_REG_SYS_CTL0 = 0x00;
static const uint8_t IP5306_REG_SYS_CTL2 = 0x02;
static const uint8_t IP5306_REG_CHARGER_CTL0 = 0x20;
static const uint8_t IP5306_REG_CHARGER_CTL1 = 0x21;
static const uint8_t IP5306_REG_CHARGER_CTL2 = 0x22;

static const uint8_t IP5306_REG_READ0 = 0x70;
static const uint8_t IP5306_REG_LEVEL = 0x78;

// Implementacia switch a select wrapperov
void IP5306Switch::write_state(bool state) {
  this->parent_->update_switch(this->type, state);
  this->publish_state(state);
}

void IP5306Select::control(const std::string &value) {
  this->parent_->update_select(this->type, value);
  this->publish_state(value);
}

float IP5306::get_setup_priority() const { return setup_priority::IO; }

void IP5306::setup() {
  ESP_LOGD(TAG, "Setting up ip5306...");
  
  // Zakladna inicializacia - zachovame Power Boost Keep On (bit 5 v SYS_CTL0) ak treba,
  // ale teraz to budeme citat/zapisovat dynamicky.
  // Pre istotu precitame aktualny stav a len logneme.
  this->read_initial_state_();
}

void IP5306::read_initial_state_() {
    uint8_t val;
    // Nacitanie a update stavov pre switche
    if (this->read_register(IP5306_REG_SYS_CTL0, &val, 1) == i2c::ERROR_OK) {
        for (auto *sw : this->switches_) {
            if (sw->type == IP5306_SWITCH_CHARGER_ENABLE) {
                sw->publish_state(val & 0x10); // Bit 4
            } else if (sw->type == IP5306_SWITCH_LOW_LOAD_SHUTDOWN) {
                sw->publish_state(val & 0x02); // Bit 1
            }
        }
    }
    // Tu by sme mohli nacitat aj selecty, ale tie su zlozitejsie na parsovanie spat.
    // Zatial nechame default alebo poslednu znamu hodnotu.
}

void IP5306::update() {
  uint8_t data[2];
  if (this->battery_level_ != nullptr) {
    if (this->read_register(IP5306_REG_LEVEL, data, 1) != i2c::ERROR_OK) {
      ESP_LOGE(TAG, "unable to read level");
      this->mark_failed();
      return;
    }
    float value = 0;
    switch (data[0] & 0xF0) {
      case 0xE0: value = 25; break;
      case 0xC0: value = 50; break;
      case 0x80: value = 75; break;
      case 0x00: value = 100; break;
    }
    this->battery_level_->publish_state(value);
  }
  
  if (this->read_register(IP5306_REG_READ0, data, 2) != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "unable to read status");
    // Tu nemarkujeme failed, lebo obcas to moze zlyhat pri spani
    return;
  }
  if (this->charger_connected_ != nullptr)
    this->charger_connected_->publish_state(data[0] & 0x08);
  if (this->charge_full_ != nullptr)
    this->charge_full_->publish_state(data[1] & 0x08);
}

void IP5306::update_switch(IP5306SwitchType type, bool state) {
  uint8_t reg = 0;
  uint8_t mask = 0;
  
  switch (type) {
    case IP5306_SWITCH_CHARGER_ENABLE:
      reg = IP5306_REG_SYS_CTL0;
      mask = 0x10; // Bit 4
      break;
    case IP5306_SWITCH_LOW_LOAD_SHUTDOWN:
      reg = IP5306_REG_SYS_CTL0;
      mask = 0x02; // Bit 1
      break;
  }

  uint8_t val;
  if (this->read_register(reg, &val, 1) != i2c::ERROR_OK) return;
  
  if (state) val |= mask;
  else val &= ~mask;

  this->write_register(reg, &val, 1);
}

void IP5306::update_select(IP5306SelectType type, const std::string &value) {
  uint8_t reg = 0;
  uint8_t val_to_write = 0;
  uint8_t mask = 0;
  uint8_t current_val = 0;

  // Helper lambda pre ziskanie indexu
  auto get_index = [&](const std::vector<std::string> &opts) -> int {
    for (size_t i = 0; i < opts.size(); i++) {
      if (opts[i] == value) return i;
    }
    return 0;
  };

  if (type == IP5306_SELECT_LIGHT_LOAD_SHUTDOWN_TIME) {
    reg = IP5306_REG_SYS_CTL2;
    mask = 0x0C; // Bits 3:2
    // 00: 8s, 01: 32s, 10: 16s, 11: 64s
    int idx = get_index({"8s", "32s", "16s", "64s"});
    val_to_write = (idx << 2);

  } else if (type == IP5306_SELECT_CHARGE_CUTOFF_VOLTAGE) {
    reg = IP5306_REG_CHARGER_CTL1;
    mask = 0x03; // Bits 1:0
    // 00: 4.2V, 01: 4.3V, 10: 4.35V, 11: 4.4V
    int idx = get_index({"4.2V", "4.3V", "4.35V", "4.4V"});
    val_to_write = idx;

  } else if (type == IP5306_SELECT_CHARGE_TERMINATION_CURRENT) {
    reg = IP5306_REG_CHARGER_CTL2;
    mask = 0x03; // Bits 1:0
    // 00: 200mA, 01: 400mA, 10: 500mA, 11: 600mA
    int idx = get_index({"200mA", "400mA", "500mA", "600mA"});
    val_to_write = idx;
    
  } else if (type == IP5306_SELECT_CHARGE_CURRENT) {
    // Toto je zlozitejsie, zalezi od bitu 5 v REG_CHARGER_CTL0.
    // Pre jednoduchost predpokladame standardne nastavenie kde bity 0-4 urcuju prud.
    // Bezny vzorec je cca 100mA kroky, alebo tabulka.
    // Tu zadefinujeme len bezne hodnoty.
    // Pozor: Toto moze byt specificke pre verziu cipu.
    // Skusime: 0x20 register.
    // Pre IP5306 sa casto uvadza:
    // Bit 5 = 0 (Internal resistors)
    // Bits 4:0 = CC Loop Setting.
    // I = 50mA + (VAL * 100mA) ? Alebo cisto len VAL.
    // Zvolim beznu implementaciu pre IP5306_I2C kniznice:
    // 0-31 pre rozne prudy.
    reg = IP5306_REG_CHARGER_CTL0;
    mask = 0x1F; // Bits 4:0
    // Toto je len priklad zoznamu, treba overit v datasheete konkretneho modulu
    // Zjednodusime na vyber: "Low", "Medium", "High" ak nevieme presne,
    // alebo skusime cisla ak mas presne info.
    // Dam sem genericke kroky po 150mA - 300mA ak by to sedelo na bitovy posun.
    // Ale podla datasheetu bit 4:0 (5 bitov) = n * 100mA + offset.
    // Nechame to na tebe si to doladit, tu je len mechanizmus.
    // Pre demo: "100mA" -> 0, "200mA" -> 1 ...
    // Alebo jednoduchsie, posleme len RAW hodnotu ak by sme mali Number komponent.
    // Ale kedze si chcel Select, dam sem fixne moznosti.
    return; // Zatial neimplementovane presne, lebo chyba tabulka.
  }

  if (this->read_register(reg, &current_val, 1) != i2c::ERROR_OK) return;
  
  current_val &= ~mask; // Vynuluj cielove bity
  current_val |= val_to_write; // Nastav nove

  this->write_register(reg, &current_val, 1);
}

}  // namespace ip5306
}  // namespace esphome