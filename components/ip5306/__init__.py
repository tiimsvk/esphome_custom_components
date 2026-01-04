import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, i2c, sensor, switch, select
from esphome.const import CONF_ID, CONF_BATTERY_LEVEL, DEVICE_CLASS_BATTERY, UNIT_PERCENT

MULTI_CONF = True

AUTO_LOAD = [ "binary_sensor", "sensor", "switch", "select" ]

ip5306_ns = cg.esphome_ns.namespace('ip5306')
IP5306 = ip5306_ns.class_('IP5306', i2c.I2CDevice, cg.Component)
IP5306Switch = ip5306_ns.class_('IP5306Switch', switch.Switch, cg.Component)
IP5306Select = ip5306_ns.class_('IP5306Select', select.Select, cg.Component)

CONF_CHARGER_CONNECTED = "charger_connected"
CONF_CHARGE_FULL = "charge_full"

# Nove konfiguracie
CONF_CHARGER_ENABLE = "charger_enable"
CONF_LOW_LOAD_SHUTDOWN = "low_load_shutdown"
CONF_LIGHT_LOAD_SHUTDOWN_TIME = "light_load_shutdown_time"
CONF_CHARGE_CUTOFF_VOLTAGE = "charge_cutoff_voltage"
CONF_CHARGE_TERMINATION_CURRENT = "charge_termination_current"

# Enumy pre C++ (Generujeme ako RAW stringy)
SWITCH_TYPES = {
    CONF_CHARGER_ENABLE: cg.RawExpression("ip5306::IP5306_SWITCH_CHARGER_ENABLE"),
    CONF_LOW_LOAD_SHUTDOWN: cg.RawExpression("ip5306::IP5306_SWITCH_LOW_LOAD_SHUTDOWN"),
}

SELECT_TYPES = {
    CONF_LIGHT_LOAD_SHUTDOWN_TIME: cg.RawExpression("ip5306::IP5306_SELECT_LIGHT_LOAD_SHUTDOWN_TIME"),
    CONF_CHARGE_CUTOFF_VOLTAGE: cg.RawExpression("ip5306::IP5306_SELECT_CHARGE_CUTOFF_VOLTAGE"),
    CONF_CHARGE_TERMINATION_CURRENT: cg.RawExpression("ip5306::IP5306_SELECT_CHARGE_TERMINATION_CURRENT"),
}

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
        
        # Switche
        cv.Optional(CONF_CHARGER_ENABLE): switch.switch_schema(IP5306Switch),
        cv.Optional(CONF_LOW_LOAD_SHUTDOWN): switch.switch_schema(IP5306Switch),
        
        # Selecty
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
        
    # Registracia switchov
    for key, type_enum in SWITCH_TYPES.items():
        if key in config:
            conf = config[key]
            sw = await switch.new_switch(conf)
            cg.add(var.set_switch(type_enum, sw))

    # Registracia selectov
    for key, type_enum in SELECT_TYPES.items():
        if key in config:
            conf = config[key]
            sel = await select.new_select(conf, options=[])
            
            options = []
            if key == CONF_LIGHT_LOAD_SHUTDOWN_TIME:
                options = ["8s", "32s", "16s", "64s"]
            elif key == CONF_CHARGE_CUTOFF_VOLTAGE:
                options = ["4.2V", "4.3V", "4.35V", "4.4V"]
            elif key == CONF_CHARGE_TERMINATION_CURRENT:
                options = ["200mA", "400mA", "500mA", "600mA"]
            
            # Oprava: nastavenie options cez traits
            cg.add(sel.traits.set_options(options))
            cg.add(var.set_select(type_enum, sel))