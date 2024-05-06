import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_WATT,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_POWER,
)

DEPENDENCIES = ["i2c"]

sic45x_led_ns = cg.esphome_ns.namespace("sic45x_led")
SiC45XComponent = sic45x_led_ns.class_("SiC45XComponent", cg.PollingComponent, i2c.I2CDevice)

CONTROLLER_ID = "controller_id"
#Settings
CONF_LED_NOMINAL_V = "led_nominal_voltage"
CONF_LED_NOMINAL_I = "led_nominal_current"
CONF_LED_NUM_SERIES = "leds_num_series"
CONF_LED_NUM_PARALLEL = "leds_num_parallel"
#optional
CONF_OVER_VOLTAGE_TOL = "overvoltage_tolerance"
CONF_OVER_CURRENT_TOL = "overcurrent_tolerance"
CONF_MIN_INPUT_V = "minimum_input_voltage"
CONF_MIN_EN_V = "minimum_enable_voltage"
CONF_MAX_INPUT_V = "maximum_input_voltage"
CONF_MAX_INPUT_I = "maximum_input_current"
CONF_MAX_OP_TEMP = "maximum_operating_temperature"
CONF_SWITCHING_FREQUENCY = "switching_frequency"
CONF_LIMIT_OUTPUT_POWER_PERCENT = "output_power_limit_percent"
#optional, calibration
CONF_SETPOINT_LOW_VOLTAGE = "calibration_setpoint_low_voltage"
CONF_SETPOINT_HIGH_VOLTAGE = "calibration_setpoint_high_voltage"
CONF_SETPOINT_LOW_CURRENT_PERCENT = "calibration_setpoint_low_percent"
CONF_SETPOINT_HIGH_CURRENT_PERCENT = "calibration_setpoint_high_percent"

#Sensors:
CONF_VOLTAGE_OUT = "voltage"
CONF_CURRENT_OUT = "current"
CONF_TEMPERATURE = "temperature"
CONF_POWER_OUT = "power"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SiC45XComponent),
            cv.Required(CONF_LED_NOMINAL_V): cv.positive_float,
            cv.Required(CONF_LED_NOMINAL_I): cv.positive_float,
            cv.Required(CONF_LED_NUM_SERIES): cv.positive_int,
            cv.Required(CONF_LED_NUM_PARALLEL): cv.positive_int,
            cv.Optional(CONF_OVER_VOLTAGE_TOL): cv.positive_float,
            cv.Optional(CONF_OVER_CURRENT_TOL): cv.positive_float,
            cv.Optional(CONF_MIN_INPUT_V): cv.positive_float,
            cv.Optional(CONF_MIN_EN_V): cv.positive_float,
            cv.Optional(CONF_MAX_INPUT_V): cv.float_range(
                min=0.0, max=20.0
            ),
            cv.Optional(CONF_MAX_INPUT_I): cv.positive_float,
            cv.Optional(CONF_MAX_OP_TEMP): cv.positive_float,
            cv.Optional(CONF_SWITCHING_FREQUENCY): cv.int_range(
                min=300, max=1500
            ),
            cv.Optional(CONF_LIMIT_OUTPUT_POWER_PERCENT): cv.float_range(
                min=0.0, max=1.0
            ),
            cv.Optional(CONF_SETPOINT_LOW_VOLTAGE): cv.positive_float,
            cv.Optional(CONF_SETPOINT_HIGH_VOLTAGE): cv.positive_float,
            cv.Optional(CONF_SETPOINT_LOW_CURRENT_PERCENT): cv.float_range(
                min=0.0, max=1.0
            ),
            cv.Optional(CONF_SETPOINT_HIGH_CURRENT_PERCENT): cv.float_range(
                min=0.0, max=1.0
            ),
            cv.Optional(CONF_VOLTAGE_OUT): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CURRENT_OUT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_POWER_OUT): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(cv.polling_component_schema("10s"))
    .extend(i2c.i2c_device_schema(0x40))
)

async def to_code(config):
    cg.add_library("https://github.com/aaronbeckmann/sic45x-driver.git", None)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    #required params
    cg.add(var.set_led_nominal_voltage(config.get(CONF_LED_NOMINAL_V)))
    cg.add(var.set_led_nominal_current(config.get(CONF_LED_NOMINAL_I)))
    cg.add(var.set_leds_series(config.get(CONF_LED_NUM_SERIES)))
    cg.add(var.set_leds_parallel(config.get(CONF_LED_NUM_PARALLEL)))

    #optional params
    if CONF_OVER_VOLTAGE_TOL in config:
        cg.add(var.set_overvoltage_tol(config.get(CONF_OVER_VOLTAGE_TOL)))
    if CONF_OVER_CURRENT_TOL in config:
        cg.add(var.set_overcurrent_tol(config.get(CONF_OVER_CURRENT_TOL)))
    if CONF_MIN_INPUT_V in config:
        cg.add(var.set_min_input_voltage(config.get(CONF_MIN_INPUT_V)))
    if CONF_MIN_EN_V in config:
        cg.add(var.set_min_enable_voltage(config.get(CONF_MIN_EN_V)))
    if CONF_MAX_INPUT_V in config:
        cg.add(var.set_max_input_voltage(config.get(CONF_MAX_INPUT_V)))
    if CONF_MAX_INPUT_I in config:
        cg.add(var.set_max_input_current(config.get(CONF_MAX_INPUT_I)))
    if CONF_MAX_OP_TEMP in config:
        cg.add(var.set_max_operating_temp(config.get(CONF_MAX_OP_TEMP)))
    if CONF_SWITCHING_FREQUENCY in config:
        cg.add(var.set_switching_frequency(config.get(CONF_SWITCHING_FREQUENCY)))
    if CONF_LIMIT_OUTPUT_POWER_PERCENT in config:
        cg.add(var.set_output_power_limit_percent(config.get(CONF_LIMIT_OUTPUT_POWER_PERCENT)))
    #optional params, calibration
    if CONF_SETPOINT_LOW_VOLTAGE in config:
        cg.add(var.set_setpoint_low_voltage(config.get(CONF_SETPOINT_LOW_VOLTAGE)))
    if CONF_SETPOINT_HIGH_VOLTAGE in config:
        cg.add(var.set_setpoint_high_voltage(config.get(CONF_SETPOINT_HIGH_VOLTAGE)))
    if CONF_SETPOINT_LOW_CURRENT_PERCENT in config:
        cg.add(var.set_setpoint_low_current_percent(config.get(CONF_SETPOINT_LOW_CURRENT_PERCENT)))
    if CONF_SETPOINT_HIGH_CURRENT_PERCENT in config:
        cg.add(var.set_setpoint_high_current_percent(config.get(CONF_SETPOINT_HIGH_CURRENT_PERCENT)))

    #sensors
    if CONF_VOLTAGE_OUT in config:
        sens = await sensor.new_sensor(config[CONF_VOLTAGE_OUT])
        cg.add(var.set_sensor_voltage(sens))

    if CONF_CURRENT_OUT in config:
        sens = await sensor.new_sensor(config[CONF_CURRENT_OUT])
        cg.add(var.set_sensor_current(sens))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_sensor_temperature(sens))

    if CONF_POWER_OUT in config:
        sens = await sensor.new_sensor(config[CONF_POWER_OUT])
        cg.add(var.set_sensor_power(sens))
