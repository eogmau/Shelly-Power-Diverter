import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, i2c
from esphome import pins
from esphome.const import (
    CONF_ID,
    CONF_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    ICON_FLASH,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_WATT,
    UNIT_WATT_HOURS,
    UNIT_EMPTY
)

DEPENDENCIES = ["i2c"]

ade7953_ns = cg.esphome_ns.namespace("ade7953")
ADE7953 = ade7953_ns.class_("ADE7953", cg.PollingComponent, i2c.I2CDevice)

CONF_IRQ_PIN = "irq_pin"
CONF_LOAD_PIN = "load_pin"
CONF_CURRENT_A = "current_a"
CONF_CURRENT_B = "current_b"
CONF_ACTIVE_POWER_A = "active_power_a"
CONF_ACTIVE_POWER_B = "active_power_b"
CONF_ENERGY_BUFFER = "energy_buffer"
CONF_ENERGY_DIVERTED = "energy_diverted"
CONF_DIVERTER_PARAMETERS = "diverter_parameters"
CONF_ENERGY_BUFFER_SIZE = "energy_buffer_size"
CONF_ENERGY_THRESHOLD_HIGH = "energy_threshold_high"
CONF_ENERGY_THRESHOLD_LOW = "energy_threshold_low"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ADE7953),
            cv.Optional(CONF_IRQ_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_LOAD_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CURRENT_A): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CURRENT_B): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ACTIVE_POWER_A): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ACTIVE_POWER_B): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ENERGY_BUFFER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT_HOURS,
                accuracy_decimals=0,
                device_class=UNIT_EMPTY,
                state_class=UNIT_EMPTY,
            ),
            cv.Optional(CONF_ENERGY_DIVERTED): sensor.sensor_schema(
                unit_of_measurement=UNIT_EMPTY,
                accuracy_decimals=0,
                device_class=UNIT_EMPTY,
                state_class=UNIT_EMPTY,
            ),
            cv.Optional(CONF_DIVERTER_PARAMETERS, default={}): cv.Schema({
                cv.Optional(CONF_ENERGY_BUFFER_SIZE, default='3600'): cv.positive_int, #1Wh
                cv.Optional(CONF_ENERGY_THRESHOLD_HIGH, default=0.75): cv.percentage,
                cv.Optional(CONF_ENERGY_THRESHOLD_LOW, default=0.25): cv.percentage
    })
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x38))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if irq_pin_config := config.get(CONF_IRQ_PIN):
        irq_pin = await cg.gpio_pin_expression(irq_pin_config)
        cg.add(var.set_irq_pin(irq_pin))
    if load_pin_config := config.get(CONF_LOAD_PIN):
        load_pin = await cg.gpio_pin_expression(load_pin_config)
        cg.add(var.set_load_pin(load_pin))

    for key in [
        CONF_VOLTAGE,
        CONF_CURRENT_A,
        CONF_CURRENT_B,
        CONF_ACTIVE_POWER_A,
        CONF_ACTIVE_POWER_B,
        CONF_ENERGY_BUFFER,
        CONF_ENERGY_DIVERTED
    ]:
        if key not in config:
            continue
        conf = config[key]
        sens = await sensor.new_sensor(conf)
        cg.add(getattr(var, f"set_{key}_sensor")(sens))
    div_param = config[CONF_DIVERTER_PARAMETERS]
    cg.add(var.set_diverter_parameters(div_param[CONF_ENERGY_BUFFER_SIZE], div_param[CONF_ENERGY_THRESHOLD_HIGH],
                                        div_param[CONF_ENERGY_THRESHOLD_LOW] ))
    cg.add(cg.App.set_loop_interval(4)) #change loop interval to get every interrupt and avoid HighSpeedLoop
