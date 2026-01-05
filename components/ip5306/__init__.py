import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, i2c, sensor, switch, select, text_sensor
from esphome.const import CONF_ID, UNIT_PERCENT, DEVICE_CLASS_BATTERY, DEVICE_CLASS_CURRENT, UNIT_AMPERE

ip5306_ns = cg.esphome_ns.namespace("ip5306")
IP5306 = ip5306_ns.class_("IP5306", i2c.I2CDevice, cg.Component)
IP5306Switch = ip5306_ns.class_("IP5306Switch", switch.Switch, cg.Component)
IP5306Select = ip5306_ns.class_("IP5306Select", select.Select, cg.Component)

AUTO_LOAD = ["binary_sensor", "sensor", "switch", "select", "text_sensor"]

# Konfiguracne konstanty
CONF_BATTERY_LEVEL = "battery_level"
CONF_CHARGER_CONNECTED = "charger_connected"
CONF_CHARGE_FULL = "charge_full"
CONF_LOW_LOAD_SHUTDOWN = "low_load_shutdown"
CONF_CHARGER_ENABLE = "charger_enable"
CONF_CHARGER_CONTROL = "charger_control"
CONF_LOAD_SHUTDOWN_TIME = "load_shutdown_time"
CONF_CHARGE_CUTOFF_VOLTAGE = "charge_cutoff_voltage"
CONF_CHARGE_TERMINATION_CURRENT = "charge_termination_current"
CONF_CURRENT = "current"
CONF_BOOST_CONTROL = "boost_control"
CONF_SOFTWARE_SHUTDOWN = "software_shutdown"
CONF_LOAD_STATUS = "load_status" # Novy textovy senzor

CONFIG_SCHEMA = cv.COMPONENT_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(IP5306),
        
        # Senzory
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
        
        # Novy textovy senzor
        cv.Optional(CONF_LOAD_STATUS): text_sensor.text_sensor_schema(),

        # Binary Senzory
        cv.Optional(CONF_CHARGER_CONNECTED): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_CHARGE_FULL): binary_sensor.binary_sensor_schema(),
        
        # Switche (musia sediet s YAML nazvami)
        cv.Optional(CONF_LOW_LOAD_SHUTDOWN): switch.switch_schema(IP5306Switch),
        cv.Optional(CONF_CHARGER_ENABLE): switch.switch_schema(IP5306Switch),
        cv.Optional(CONF_CHARGER_CONTROL): switch.switch_schema(IP5306Switch),
        cv.Optional(CONF_BOOST_CONTROL): switch.switch_schema(IP5306Switch),
        cv.Optional(CONF_SOFTWARE_SHUTDOWN): switch.switch_schema(IP5306Switch),
        
        # Selecty
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

    # Registracia switchov
    switches = [
        (CONF_LOW_LOAD_SHUTDOWN, "set_low_load_shutdown_switch"),
        (CONF_CHARGER_ENABLE, "set_charger_enable_switch"),
        (CONF_CHARGER_CONTROL, "set_charge_control_switch"),
        (CONF_BOOST_CONTROL, "set_boost_control_switch"),
        (CONF_SOFTWARE_SHUTDOWN, "set_software_shutdown_switch"),
    ]
    for conf_key, func_name in switches:
        if conf_key in config:
            sw = await switch.new_switch(config[conf_key])
            cg.add(sw.set_parent(var))
            cg.add(getattr(var, func_name)(sw))

    # Registracia selectov
    if CONF_LOAD_SHUTDOWN_TIME in config:
        sel = await select.new_select(config[CONF_LOAD_SHUTDOWN_TIME], options=[])
        cg.add(sel.set_parent(var))
        cg.add(sel.traits.set_options(["8s", "32s", "16s", "64s"]))
        cg.add(var.set_load_shutdown_time_select(sel))

    if CONF_CHARGE_CUTOFF_VOLTAGE in config:
        sel = await select.new_select(config[CONF_CHARGE_CUTOFF_VOLTAGE], options=[])
        cg.add(sel.set_parent(var))
        cg.add(sel.traits.set_options(["4.2V", "4.3V", "4.35V", "4.4V"]))
        cg.add(var.set_charge_cutoff_voltage_select(sel))

    if CONF_CHARGE_TERMINATION_CURRENT in config:
        sel = await select.new_select(config[CONF_CHARGE_TERMINATION_CURRENT], options=[])
        cg.add(sel.set_parent(var))
        cg.add(sel.traits.set_options(["200mA", "400mA", "500mA", "600mA"]))
        cg.add(var.set_charge_termination_current_select(sel))
