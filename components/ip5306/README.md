# IP5306 battery charger

This component provides support for the IP5306 battery charger IC.

You need an `i2c:` component configured.  All the sensors are optional.

Example:
```yaml
ip5306:
  battery_level:
    name: "Battery Level"
  charger_connected:
    name: "Charger Connected"
  charge_full:
    name: "Charge Full"
  low_load_shutdown:
    name: "Low Load Shutdown"
  charger_enable:
    name: "Charger Enable"
  charger_control:
    name: "Charger Control"
  load_shutdown_time:
    name: "Load Shutdown Time"
  charge_cutoff_voltage:
    name: "Charge Cutoff Voltage"
  charge_termination_current:
    name: "Charge Termination Current"
```
