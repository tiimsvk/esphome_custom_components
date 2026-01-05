import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, i2c, sensor, switch, select, text_sensor
from esphome.const import CONF_ID, UNIT_AMPERE, DEVICE_CLASS_CURRENT, DEVICE_CLASS_BATTERY, UNIT_PERCENT

ip5306_ns = cg.esphome_ns.namespace("ip5306")
IP5306 = ip5306_ns.class_("IP5306", i2c.I2CDevice, cg.Component)
IP5306Switch = ip5306_ns.class_("IP5306Switch", switch.Switch, cg.Component)
IP5306Select = ip5306_ns.class_("IP5306Select", select.Select, cg.Component)
IP5306SwitchType = ip5306_ns.enum("IP5306SwitchType")
IP5306SelectType = ip5306_ns.enum("IP5306SelectType")

AUTO_LOAD = ["binary_sensor", "sensor", "switch", "select", "text_sensor"]

# Konfiguracne konstanty
CONF_BATTERY_LEVEL = "battery_level"
CONF_CHARGER_CONNECTED = "charger_connected"
CONF_CHARGE_FULL = "charge_full"
CONF_CURRENT = "current"
CONF_LOAD_STATUS = "load_status" # Novy textovy senzor

# Switche
CONF_LOW_LOAD_SHUTDOWN = "low_load_shutdown"
CONF_CHARGER_ENABLE = "charger_enable"
CONF_BOOST_ENABLE = "boost_enable"
CONF_BUTTON_SHUTDOWN = "button_shutdown"
CONF_BOOST_ON_LOAD = "boost_on_load"
CONF_SHORT_PRESS_BOOST = "short_press_boost"

# Selecty
CONF_LOAD_SHUTDOWN_TIME = "load_shutdown_time"
CONF_CHARGE_CUTOFF_VOLTAGE = "charge_cutoff_voltage"
CONF_CHARGE_TERMINATION_CURRENT = "charge_termination_current"

CONFIG_SCHEMA = cv.COMPONENT_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(IP5306),
        # Sensory
        cv.Optional(CONF_BATTERY_LEVEL): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            device_class=DEVICE_CLASS_BATTERY,
            accuracy_decimals=0,
        ),
        cv.Optional(CONF_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            device_class=DEVICE_CLASS_CURRENT,
            accuracy_decimals=2,
        ),
        # Text Sensor
        cv.Optional(CONF_LOAD_STATUS): text_sensor.text_sensor_schema(),

        # BinarySensory
        cv.Optional(CONF_CHARGER_CONNECTED): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_CHARGE_FULL): binary_sensor.binary_sensor_schema(),
        
        # Switch
        cv.Optional(CONF_LOW_LOAD_SHUTDOWN): switch.switch_schema(IP5306Switch),
        cv.Optional(CONF_CHARGER_ENABLE): switch.switch_schema(IP5306Switch),
        cv.Optional(CONF_BOOST_ENABLE): switch.switch_schema(IP5306Switch),
        cv.Optional(CONF_BUTTON_SHUTDOWN): switch.switch_schema(IP5306Switch),
        cv.Optional(CONF_BOOST_ON_LOAD): switch.switch_schema(IP5306Switch),
        cv.Optional(CONF_SHORT_PRESS_BOOST): switch.switch_schema(IP5306Switch),

        # Select
        cv.Optional(CONF_LOAD_SHUTDOWN_TIME): select.select_schema(IP5306Select),
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

    if CONF_CURRENT in config:
        sens = await sensor.new_sensor(config[CONF_CURRENT])
        cg.add(var.set_current_sensor(sens))

    if CONF_LOAD_STATUS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_LOAD_STATUS])
        cg.add(var.set_load_status_sensor(sens))

    if CONF_CHARGER_CONNECTED in config:
        binary_sens = await binary_sensor.new_binary_sensor(config[CONF_CHARGER_CONNECTED])
        cg.add(var.set_charger_connected(binary_sens))

    if CONF_CHARGE_FULL in config:
        binary_sens = await binary_sensor.new_binary_sensor(config[CONF_CHARGE_FULL])
        cg.add(var.set_charge_full(binary_sens))

    # Helper function pre switche
    async def add_switch(conf_name, type_enum):
        if conf_name in config:
            sw = await switch.new_switch(config[conf_name])
            cg.add(sw.set_parent(var))
            cg.add(var.set_switch(sw, type_enum))

    await add_switch(CONF_LOW_LOAD_SHUTDOWN, IP5306SwitchType.IP5306_SWITCH_LOW_LOAD_SHUTDOWN)
    await add_switch(CONF_CHARGER_ENABLE, IP5306SwitchType.IP5306_SWITCH_CHARGER_ENABLE)
    await add_switch(CONF_BOOST_ENABLE, IP5306SwitchType.IP5306_SWITCH_BOOST_ENABLE)
    await add_switch(CONF_BUTTON_SHUTDOWN, IP5306SwitchType.IP5306_SWITCH_BUTTON_SHUTDOWN)
    await add_switch(CONF_BOOST_ON_LOAD, IP5306SwitchType.IP5306_SWITCH_BOOST_ON_LOAD)
    await add_switch(CONF_SHORT_PRESS_BOOST, IP5306SwitchType.IP5306_SWITCH_SHORT_PRESS_BOOST)

    # Helper function pre selecty
    async def add_select(conf_name, type_enum, options):
        if conf_name in config:
            sel = await select.new_select(config[conf_name], options=[])
            cg.add(sel.set_parent(var))
            cg.add(sel.traits.set_options(options))
            cg.add(var.set_select(sel, type_enum))

    await add_select(CONF_LOAD_SHUTDOWN_TIME, IP5306SelectType.IP5306_SELECT_LOAD_SHUTDOWN_TIME, ["8s", "32s", "16s", "64s"])
    await add_select(CONF_CHARGE_CUTOFF_VOLTAGE, IP5306SelectType.IP5306_SELECT_CHARGE_CUTOFF_VOLTAGE, ["4.15V", "4.30V", "4.35V", "4.40V"])
    await add_select(CONF_CHARGE_TERMINATION_CURRENT, IP5306SelectType.IP5306_SELECT_CHARGE_TERMINATION_CURRENT, ["200mA", "400mA", "500mA", "600mA"])
