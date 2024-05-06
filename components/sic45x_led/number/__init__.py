import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_MODE

from ..sensor import CONTROLLER_ID, SiC45XComponent

fan_pwm_ns = cg.esphome_ns.namespace(
    "sic45x_led::brightness"
)
SiC45X_Brightness = fan_pwm_ns.class_(
    "SiC45X_Brightness", number.Number, cg.Component
)

CONFIG_SCHEMA = (
    number.number_schema(SiC45X_Brightness)
    .extend(
        {
            cv.GenerateID(): cv.declare_id(SiC45X_Brightness),
            cv.Optional(CONF_MODE, default="SLIDER"): cv.enum(
                number.NUMBER_MODES, upper=True
            ),
            cv.Required(CONTROLLER_ID): cv.use_id(SiC45XComponent),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


def to_code(config):
    var = yield number.new_number(config, min_value=0, max_value=100, step=1)
    yield cg.register_component(var, config)

    controller = yield cg.get_variable(config[CONTROLLER_ID])
    cg.add(controller.register_brightness(var))