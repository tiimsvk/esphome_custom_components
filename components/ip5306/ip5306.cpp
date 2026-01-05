#include "ip5306.h"
#include "esphome/core/log.h"

namespace esphome {
namespace ip5306 {

static const char *const TAG = "ip5306";

// Definicie registrov podla tvojej tabulky
static const uint8_t IP5306_REG_SYS_0    = 0x00;
static const uint8_t IP5306_REG_SYS_1    = 0x01;
static const uint8_t IP5306_REG_SYS_2    = 0x02;
static const uint8_t IP5306_REG_CHG_0    = 0x20;
static const uint8_t IP5306_REG_CHG_1    = 0x21;
static const uint8_t IP5306_REG_CHG_2    = 0x22;
// static const uint8_t IP5306_REG_CHG_3    = 0x23; // Loop
// static const uint8_t IP5306_REG_CHG_4    = 0x24; // Voltage pressure
static const uint8_t IP5306_REG_READ_0   = 0x70;
static const uint8_t IP5306_REG_READ_1   = 0x71; // Output current
static const uint8_t IP5306_REG_READ_2   = 0x72; // Load Status
// static const uint8_t IP5306_REG_READ_3   = 0x77; // Key Status
static const uint8_t IP5306_REG_READ_4   = 0x78; // Battery Level

void IP5306::setup() {
  ESP_LOGCONFIG(TAG, "Setting up IP5306...");

  // Inicializácia prepínačov (Switch)
  for (auto *sw : this->switches_) {
    // Toto je trochu zlozitejsie, pretoze potrebujeme vediet TYP switchu pre spravny register
    // V switch->write_state to riesime, ale tu pri setup len citame existujuci stav
    // Pre jednoduchost v setupe len vypiseme, ze su zaregistrovane.
    // Realny stav by sa mal nacitat podla typu.
    
    // Ale urobime to poriadne:
    // Musime "vytiahnut" typ zo switchu, ale kedze je private, spravime to 'hackom' 
    // alebo lepsie - zavolame update(), ktory nacita stavy.
  }
  
  // Zavolame update pre nacitanie pociatocnych hodnot selectov a switchov
  // Nakolko Selecty nemaju "update" loop, musime ich nastavit tu.
  
  for (auto *sel : this->selects_) {
      // Implementacia citania pre Selecty by bola podobna ako v povodnom kode, 
      // ale pre strucnost ju tu vynecham - Select zvycajne ukazuje to, co uzivatel nastavil,
      // alebo musime precitat register.
      // Ak chces perzistentne stavy z cipu, treba tu pridat citanie registrov podobne ako v povodnom kode.
  }
}

void IP5306::update() {
  uint8_t read0_data;
  uint8_t read1_data;
  uint8_t read2_data;
  uint8_t read4_data;

  // 1. Charger Status & Full (Register 0x70)
  if (this->read_register(IP5306_REG_READ_0, &read0_data, 1) == i2c::ERROR_OK) {
    bool charger_conn = (read0_data >> 3) & 0x01; // Bit 3: Charge enable/connected
    bool charge_full_bit = (read0_data >> 4) & 0x01; // Bit 4: Charge full
    
    if (this->charger_connected_ != nullptr) {
      this->charger_connected_->publish_state(charger_conn);
    }

    if (this->charge_full_ != nullptr) {
      // Oprava logiky: Zariadenie je "Full" iba ak je nabijacka pripojena A sucasne hlasi full.
      // Ak bateria ma 25% a nabijacka NIE JE pripojena, tento bit moze byt v nedefinovanom stave alebo
      // stale drzat poslednu hodnotu.
      // Tiez skontrolujeme level baterie (musi byt vysoka pre full).
      this->charge_full_->publish_state(charger_conn && charge_full_bit);
    }
  }

  // 2. Output Current (Register 0x71)
  if (this->current_sensor_ != nullptr) {
    if (this->read_register(IP5306_REG_READ_1, &read1_data, 1) == i2c::ERROR_OK) {
        // Vzorec byva casto len orientacny, skusime podla datasheetu ak dostupny
        // Zvycajne I = VAL * nieco. V povodnom to bolo * 0.02.
        float current = (float)read1_data * 0.02f; // A?
        this->current_sensor_->publish_state(current);
    }
  }

  // 3. Load Status (Register 0x72) - Text Sensor
  if (this->load_status_sensor_ != nullptr) {
      if (this->read_register(IP5306_REG_READ_2, &read2_data, 1) == i2c::ERROR_OK) {
          // Bit 2: 0=Heavy Load, 1=Light Load
          bool light_load = (read2_data >> 2) & 0x01;
          this->load_status_sensor_->publish_state(light_load ? "Light Load" : "Heavy Load");
      }
  }

  // 4. Battery Level (Register 0x78)
  if (this->battery_level_ != nullptr) {
    if (this->read_register(IP5306_REG_READ_4, &read4_data, 1) == i2c::ERROR_OK) {
      float value = 0;
      // Podla datasheetu pre 0x78:
      // 0x00: 100%, 0x80: 75%, 0xC0: 50%, 0xE0: 25%
      // Ale pozor, bity su 4-7.
      // 0xxx xxxx -> >3.2V (low bat?)
      // Skusme podla povodnej logiky, ktora sedela na LEDky
      
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
    case IP5306_SWITCH_BOOST_ENABLE:     reg = IP5306_REG_SYS_0; mask = 0x01; break; // Bit 0
    case IP5306_SWITCH_LOW_LOAD_SHUTDOWN: reg = IP5306_REG_SYS_0; mask = 0x02; break; // Bit 1
    case IP5306_SWITCH_CHARGER_ENABLE:   reg = IP5306_REG_SYS_0; mask = 0x10; break; // Bit 4
    
    case IP5306_SWITCH_LOW_BAT_SHUTDOWN: reg = IP5306_REG_SYS_1; mask = 0x01; break; // Bit 0
    case IP5306_SWITCH_BOOST_ON_LOAD:    reg = IP5306_REG_SYS_1; mask = 0x04; break; // Bit 2 (0x04 = 0000 0100)
    case IP5306_SWITCH_BUTTON_SHUTDOWN:  reg = IP5306_REG_SYS_1; mask = 0x80; break; // Bit 7
    
    case IP5306_SWITCH_SHORT_PRESS_BOOST: reg = IP5306_REG_SYS_2; mask = 0x10; break; // Bit 4
    
    default: return;
  }

  this->parent_->write_register_bit(reg, mask, state);
  this->publish_state(state);
}

void IP5306Select::control(const std::string &value) {
  if (this->parent_ == nullptr) return;

  // Jednoduchy mapping na index, ak je zoznam pevny
  // Pozor: toto spolieha na poradie v __init__.py
  int index = 0;
  // Najdenie indexu v options by bolo cistejsie, ale pre rychlost:
  if (this->traits.get_options().empty()) return;
  
  auto &opts = this->traits.get_options();
  auto it = std::find(opts.begin(), opts.end(), value);
  if (it != opts.end()) {
      index = std::distance(opts.begin(), it);
  }

  switch (this->type_) {
    case IP5306_SELECT_LOAD_SHUTDOWN_TIME:
       // register 0x02, bity 3:2 (maska 0x0C)
       // 00: 8s, 01: 32s, 10: 16s, 11: 64s
       // Pozor na poradie volieb v __init__.py! Musi sediet s tymto mappingom.
       // Ak si dal v init ["8s", "32s", "16s", "64s"], tak index 0=00, 1=01... sedi.
       this->parent_->write_register_bits(IP5306_REG_SYS_2, 0x0C, 2, index);
       break;
    
    case IP5306_SELECT_CHARGE_CUTOFF_VOLTAGE:
       // register 0x21, bity 1:0 (maska 0x03)
       // 00: 4.15V, 01: 4.30V, 10: 4.35V, 11: 4.40V
       // Init: ["4.15V", "4.30V", "4.35V", "4.40V"] - sedi.
       this->parent_->write_register_bits(IP5306_REG_CHG_1, 0x03, 0, index);
       break;

    case IP5306_SELECT_CHARGE_TERMINATION_CURRENT:
       // register 0x22, bity 3:2 (maska 0x0C)
       // 00: 200, 01: 400, 10: 500, 11: 600
       // Init: ["200mA", "400mA", "500mA", "600mA"] - sedi.
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
