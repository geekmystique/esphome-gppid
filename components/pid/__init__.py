import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import sensor, output, number, switch
from esphome.const import CONF_HUMIDITY_SENSOR, CONF_ID, CONF_OUTPUT

CODEOWNERS = ["@OttoWinter", "@hsteinhaus"]
MULTI_CONF = True

pid_ns = cg.esphome_ns.namespace("pid")
PIDComponent = pid_ns.class_("PIDComponent", cg.Component)

# PIDResetIntegralTermAction = pid_ns.class_(
#     "PIDResetIntegralTermAction", automation.Action
# )

# PIDSetControlParametersAction = pid_ns.class_(
#     "PIDSetControlParametersAction", automation.Action
# )

CONF_OUTPUT_PARAMETERS = "output"
CONF_KP = "kp"
CONF_KI = "ki"
CONF_STARTING_INTEGRAL_TERM = "starting_integral_term"
CONF_KD = "kd"
CONF_KP_NUM = "kp_num"
CONF_KI_NUM = "ki_num"
CONF_KD_NUM = "kd_num"
CONF_CONTROL_PARAMETERS = "control_parameters"
CONF_NOISEBAND = "noiseband"
CONF_POSITIVE_OUTPUT = "positive_output"
CONF_NEGATIVE_OUTPUT = "negative_output"
CONF_MIN_OUTPUT = "min_output"
CONF_MAX_OUTPUT = "max_output"
CONF_OUTPUT_MIN_NUM = "output_min_num"
CONF_OUTPUT_MAX_NUM = "output_max_num"

# Deadband parameters
CONF_DEADBAND_PARAMETERS = "deadband_parameters"
CONF_THRESHOLD_HIGH = "threshold_high"
CONF_THRESHOLD_LOW = "threshold_low"
CONF_DEADBAND_OUTPUT_AVERAGING_SAMPLES = "deadband_output_averaging_samples"
CONF_KP_MULTIPLIER = "kp_multiplier"
CONF_KI_MULTIPLIER = "ki_multiplier"
CONF_KD_MULTIPLIER = "kd_multiplier"

CONF_TARGET_NUMBER = "target_number"
CONF_TARGET_SENSOR = "target_sensor"
CONF_CURRENT_VALUE = "current_value"
CONF_FF_INPUT = "feedforward_input"
CONF_ENABLE = "enable_switch"

def validate(config):
    if not (CONF_TARGET_NUMBER in config) ^ (CONF_TARGET_SENSOR in config):
        raise cv.Invalid(
            "Either target_sensor or target_number must be provided."
        )

    # if CONF_OUTPUT in config and (
    #     CONF_OUTPUT_MIN not in config or
    #     CONF_OUTPUT_MAX not in config
    # ):
    #     raise cv.Invalid("output_min and ouput_max have to be provided if output is in use.")
    return config


CONFIG_SCHEMA = cv.All(
    cv.COMPONENT_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(PIDComponent),
            cv.Required(CONF_CURRENT_VALUE): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_FF_INPUT): cv.use_id(number.Number),
            cv.Optional(CONF_ENABLE): cv.use_id(switch.Switch),
            cv.Optional(CONF_TARGET_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_TARGET_NUMBER): cv.use_id(number.Number),
            cv.Optional(CONF_HUMIDITY_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_OUTPUT_PARAMETERS): cv.Schema(
                {
                    cv.Required(cv.GenerateID()): cv.use_id(output.FloatOutput),
                }
            ),
            cv.Optional(CONF_DEADBAND_PARAMETERS): cv.Schema(
                {
                    cv.Required(CONF_THRESHOLD_HIGH): cv.float_,
                    cv.Required(CONF_THRESHOLD_LOW): cv.float_,
                    cv.Optional(CONF_KP_MULTIPLIER, default=0.1): cv.float_,
                    cv.Optional(CONF_KI_MULTIPLIER, default=0.0): cv.float_,
                    cv.Optional(CONF_KD_MULTIPLIER, default=0.0): cv.float_,
                    cv.Optional(
                        CONF_DEADBAND_OUTPUT_AVERAGING_SAMPLES, default=1
                    ): cv.int_,
                }
            ),
            cv.Required(CONF_CONTROL_PARAMETERS): cv.Schema(
                {
                    cv.Required(CONF_KP): cv.float_,
                    cv.Optional(CONF_KI, default=0.0): cv.float_,
                    cv.Optional(CONF_KD, default=0.0): cv.float_,
                    cv.Optional(CONF_KP_NUM): cv.use_id(number.Number),
                    cv.Optional(CONF_KI_NUM): cv.use_id(number.Number),
                    cv.Optional(CONF_KD_NUM): cv.use_id(number.Number),
                    cv.Optional(CONF_OUTPUT_MIN_NUM): cv.use_id(number.Number),
                    cv.Optional(CONF_OUTPUT_MAX_NUM): cv.use_id(number.Number),
                    cv.Optional(CONF_STARTING_INTEGRAL_TERM, default=0.0): cv.float_,
                    cv.Optional(CONF_MIN_OUTPUT, default=0): cv.float_,
                    cv.Optional(CONF_MAX_OUTPUT, default=1): cv.float_,
                }
            ),
        }
    )
    , validate
)



async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    sens = await cg.get_variable(config[CONF_CURRENT_VALUE])
    cg.add(var.set_input_sensor(sens))

    if CONF_FF_INPUT in config:
        num = await cg.get_variable(config[CONF_FF_INPUT])
        cg.add(var.set_ff_input_number(num))

    if CONF_ENABLE in config:
        switch = await cg.get_variable(config[CONF_ENABLE])
        cg.add(var.set_enable_switch(switch))

    if CONF_TARGET_SENSOR in config:
        sens = await cg.get_variable(config[CONF_TARGET_SENSOR])
        cg.add(var.set_target_sensor(sens))

    if CONF_TARGET_NUMBER in config:
        num = await cg.get_variable(config[CONF_TARGET_NUMBER])
        cg.add(var.set_target_number(num))

    if CONF_OUTPUT_PARAMETERS in config:
        params = config[CONF_OUTPUT_PARAMETERS]
        out = await cg.get_variable(params[CONF_ID])
        cg.add(var.set_output(out))

    params = config[CONF_CONTROL_PARAMETERS]
    cg.add(var.set_kp(params[CONF_KP]))
    cg.add(var.set_ki(params[CONF_KI]))
    cg.add(var.set_kd(params[CONF_KD]))
    cg.add(var.set_starting_integral_term(params[CONF_STARTING_INTEGRAL_TERM]))
    if CONF_KP_NUM in params:
        num = await cg.get_variable(params[CONF_KP_NUM])
        cg.add(var.set_kp_number(num))
    if CONF_KI_NUM in params:
        num = await cg.get_variable(params[CONF_KI_NUM])
        cg.add(var.set_ki_number(num))
    if CONF_KD_NUM in params:
        num = await cg.get_variable(params[CONF_KD_NUM])
        cg.add(var.set_kd_number(num))
    if CONF_OUTPUT_MIN_NUM in params:
        num = await cg.get_variable(params[CONF_OUTPUT_MIN_NUM])
        cg.add(var.set_min_output_number(num))
    if CONF_OUTPUT_MAX_NUM in params:
        num = await cg.get_variable(params[CONF_OUTPUT_MAX_NUM])
        cg.add(var.set_max_output_number(num))


    if CONF_MIN_OUTPUT in params:
        cg.add(var.set_min_output(params[CONF_MIN_OUTPUT]))
    if CONF_MAX_OUTPUT in params:
        cg.add(var.set_max_output(params[CONF_MAX_OUTPUT]))

    if CONF_DEADBAND_PARAMETERS in config:
        params = config[CONF_DEADBAND_PARAMETERS]
        cg.add(var.set_threshold_low(params[CONF_THRESHOLD_LOW]))
        cg.add(var.set_threshold_high(params[CONF_THRESHOLD_HIGH]))
        cg.add(var.set_kp_multiplier(params[CONF_KP_MULTIPLIER]))
        cg.add(var.set_ki_multiplier(params[CONF_KI_MULTIPLIER]))
        cg.add(var.set_kd_multiplier(params[CONF_KD_MULTIPLIER]))
        cg.add(
            var.set_deadband_output_samples(
                params[CONF_DEADBAND_OUTPUT_AVERAGING_SAMPLES]
            )
        )



# @automation.register_action(
#     "pid.reset_integral_term",
#     PIDResetIntegralTermAction,
#     automation.maybe_simple_id(
#         {
#             cv.Required(CONF_ID): cv.use_id(PIDComponent),
#         }
#     ),
# )
# async def pid_reset_integral_term(config, action_id, template_arg, args):
#     paren = await cg.get_variable(config[CONF_ID])
#     return cg.new_Pvariable(action_id, template_arg, paren)

# @automation.register_action(
#     "pid.set_control_parameters",
#     PIDSetControlParametersAction,
#     automation.maybe_simple_id(
#         {
#             cv.Required(CONF_ID): cv.use_id(PIDComponent),
#             cv.Required(CONF_KP): cv.templatable(cv.float_),
#             cv.Optional(CONF_KI, default=0.0): cv.templatable(cv.float_),
#             cv.Optional(CONF_KD, default=0.0): cv.templatable(cv.float_),
#         }
#     ),
# )
# async def set_control_parameters(config, action_id, template_arg, args):
#     paren = await cg.get_variable(config[CONF_ID])
#     var = cg.new_Pvariable(action_id, template_arg, paren)

#     kp_template_ = await cg.templatable(config[CONF_KP], args, float)
#     cg.add(var.set_kp(kp_template_))

#     ki_template_ = await cg.templatable(config[CONF_KI], args, float)
#     cg.add(var.set_ki(ki_template_))

#     kd_template_ = await cg.templatable(config[CONF_KD], args, float)
#     cg.add(var.set_kd(kd_template_))

#     return var