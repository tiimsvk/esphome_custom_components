import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, i2c, sensor, switch, select
from esphome.const import CONF_ID, CONF_BATTERY_LEVEL, DEVICE_CLASS_BATTERY, UNIT_PERCENT

ip5306_ns = cg.esphome_ns.namespace('ip5306')
IP5306 = ip5306_ns.class_('IP5306', i2c.I2CDevice, cg.PollingComponent)

# Helper classes
IP5306Switch = ip5306_ns.class_('IP5306Switch', switch.Switch)
IP5306Select = ip5306_ns.class_('IP5306Select', select.Select)

CONF_CHARGER_CONNECTED = "charger_connected"
CONF_CHARGE_FULL = "charge_full"

# New config keys
CONF_CHARGER_ENABLE = "charger_enable"
CONF_LOW_LOAD_SHUTDOWN = "low_load_shutdown"
CONF_LIGHT_LOAD_SHUTDOWN_TIME = "light_load_shutdown_time"
CONF_CHARGE_CUTOFF_VOLTAGE = "charge_cutoff_voltage"
CONF_CHARGE_TERMINATION_CURRENT = "charge_termination_current"

CONFIG_SCHEMA = cv.COMPONENT_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(IP5306),
        cv.Optional(CONF_BATTERY_LEVEL): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            device_class=DEVICE_CLASS_BATTERY,
            accuracy_decimals=0,
        ),
        cv.Optional(CONF_CHARGER_CONNECTED): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_CHARGE_FULL): binary_sensor.binary_sensor_schema(),
        
        # Switches
        cv.Optional(CONF_CHARGER_ENABLE): switch.switch_schema(IP5306Switch),
        cv.Optional(CONF_LOW_LOAD_SHUTDOWN): switch.switch_schema(IP5306Switch),
        
        # Selects
        cv.Optional(CONF_LIGHT_LOAD_SHUTDOWN_TIME): select.select_schema(IP5306Select),
        cv.Optional(CONF_CHARGE_CUTOFF_VOLTAGE): select.select_schema(IP5306Select),
        cv.Optional(CONF_CHARGE_TERMINATION_CURRENT): select.select_schema(IP5306Select),
    }
).extend(i2c.i2c_device_schema(0x75))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_BATTERY_LEVEL in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY_LEVEL])
        cg.add(var.set_battery_level(sens))

    if CONF_CHARGER_CONNECTED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CHARGER_CONNECTED])
        cg.add(var.set_charger_connected(sens))

    if CONF_CHARGE_FULL in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CHARGE_FULL])
        cg.add(var.set_charge_full(sens))
        
    # Register Switches
    if CONF_CHARGER_ENABLE in config:
        s = await switch.new_switch(config[CONF_CHARGER_ENABLE])
        cg.add(var.set_charger_enable_switch(s))

    if CONF_LOW_LOAD_SHUTDOWN in config:
        s = await switch.new_switch(config[CONF_LOW_LOAD_SHUTDOWN])
        cg.add(var.set_low_load_shutdown_switch(s))

    # Register Selects
    if CONF_LIGHT_LOAD_SHUTDOWN_TIME in config:
        s = await select.new_select(config[CONF_LIGHT_LOAD_SHUTDOWN_TIME], options=[])
        cg.add(var.set_light_load_shutdown_time_select(s))

    if CONF_CHARGE_CUTOFF_VOLTAGE in config:
        s = await select.new_select(config[CONF_CHARGE_CUTOFF_VOLTAGE], options=[])
        cg.add(var.set_charge_cutoff_voltage_select(s))
        
    if CONF_CHARGE_TERMINATION_CURRENT in config:
        s = await select.new_select(config[CONF_CHARGE_TERMINATION_CURRENT], options=[])
        cg.add(var.set_charge_termination_current_select(s))