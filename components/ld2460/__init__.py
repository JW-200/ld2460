import esphome.codegen as cg
from esphome.components import binary_sensor, number, select, sensor, switch, text_sensor, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_OCCUPANCY,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_COUNTER,
    STATE_CLASS_MEASUREMENT,
    UNIT_DEGREES,
    UNIT_METER,
)

AUTO_LOAD = ["binary_sensor", "number", "select", "sensor", "switch", "text_sensor"]
DEPENDENCIES = ["uart"]
MULTI_CONF = True

ld2460_ns = cg.esphome_ns.namespace("ld2460")
LD2460Component = ld2460_ns.class_(
    "LD2460Component", cg.Component, uart.UARTDevice
)
LD2460ReportingSwitch = ld2460_ns.class_("LD2460ReportingSwitch", switch.Switch, cg.Component)
LD2460ConfigNumber = ld2460_ns.class_("LD2460ConfigNumber", number.Number)
LD2460InstallationModeSelect = ld2460_ns.class_("LD2460InstallationModeSelect", select.Select)

CONF_BAUD_SCAN = "baud_scan"
CONF_FLUSH_TIMEOUT = "flush_timeout"
CONF_MAX_BUFFER_SIZE = "max_buffer_size"
CONF_NO_DATA_LOG_INTERVAL = "no_data_log_interval"
CONF_PUBLISH_INTERVAL = "publish_interval"
CONF_REPORTING = "reporting"
CONF_PRESENCE = "presence"
CONF_SUMMARY = "summary"
CONF_TARGET_COUNT = "target_count"
CONF_FIRMWARE = "firmware"
CONF_INSTALLATION_MODE = "installation_mode"
CONF_INSTALLATION_HEIGHT = "installation_height"
CONF_INSTALLATION_ANGLE = "installation_angle"
CONF_DETECTION_DISTANCE = "detection_distance"
CONF_START_ANGLE = "start_angle"
CONF_END_ANGLE = "end_angle"
CONF_X = "x"
CONF_Y = "y"
CONF_DISTANCE = "distance"
CONF_ANGLE = "angle"
CONF_TARGETS = [f"target_{index}" for index in range(1, 6)]

TARGET_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_X): sensor.sensor_schema(
            unit_of_measurement=UNIT_METER,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_DISTANCE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_Y): sensor.sensor_schema(
            unit_of_measurement=UNIT_METER,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_DISTANCE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
            unit_of_measurement=UNIT_METER,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_DISTANCE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ANGLE): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            icon="mdi:angle-acute",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LD2460Component),
            cv.Optional(CONF_BAUD_SCAN, default=True): cv.boolean,
            cv.Optional(CONF_SUMMARY): text_sensor.text_sensor_schema(
                icon="mdi:radar",
            ),
            cv.Optional(CONF_FIRMWARE): text_sensor.text_sensor_schema(
                icon="mdi:chip",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_INSTALLATION_MODE): text_sensor.text_sensor_schema(
                icon="mdi:radar",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_PRESENCE): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_OCCUPANCY,
            ),
            cv.Optional(CONF_TARGET_COUNT): sensor.sensor_schema(
                icon=ICON_COUNTER,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_REPORTING): switch.switch_schema(
                LD2460ReportingSwitch,
                icon="mdi:radar",
            ).extend(cv.COMPONENT_SCHEMA),
            cv.Optional(CONF_INSTALLATION_HEIGHT): number.number_schema(
                LD2460ConfigNumber, unit_of_measurement=UNIT_METER,
            ),
            cv.Optional(CONF_INSTALLATION_ANGLE): number.number_schema(
                LD2460ConfigNumber, unit_of_measurement=UNIT_DEGREES,
            ),
            cv.Optional(CONF_DETECTION_DISTANCE): number.number_schema(
                LD2460ConfigNumber, unit_of_measurement=UNIT_METER,
            ),
            cv.Optional(CONF_START_ANGLE): number.number_schema(
                LD2460ConfigNumber, unit_of_measurement=UNIT_DEGREES,
            ),
            cv.Optional(CONF_END_ANGLE): number.number_schema(
                LD2460ConfigNumber, unit_of_measurement=UNIT_DEGREES,
            ),
            cv.Optional(CONF_INSTALLATION_MODE + "_select"): select.select_schema(
                LD2460InstallationModeSelect,
            ),
            **{cv.Optional(target): TARGET_SCHEMA for target in CONF_TARGETS},
            cv.Optional(CONF_FLUSH_TIMEOUT, default="100ms"): cv.positive_time_period_milliseconds,
            # The shortest valid protocol frame is 11 bytes.
            cv.Optional(CONF_MAX_BUFFER_SIZE, default=48): cv.int_range(min=11, max=512),
            cv.Optional(CONF_NO_DATA_LOG_INTERVAL, default="10s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_PUBLISH_INTERVAL, default="500ms"): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "ld2460",
    baud_rate=115200,
    require_tx=True,
    require_rx=True,
    data_bits=8,
    parity="NONE",
    stop_bits=1,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if summary_config := config.get(CONF_SUMMARY):
        sens = await text_sensor.new_text_sensor(summary_config)
        cg.add(var.set_summary_text_sensor(sens))

    if firmware_config := config.get(CONF_FIRMWARE):
        sens = await text_sensor.new_text_sensor(firmware_config)
        cg.add(var.set_firmware_text_sensor(sens))

    if installation_mode_config := config.get(CONF_INSTALLATION_MODE):
        sens = await text_sensor.new_text_sensor(installation_mode_config)
        cg.add(var.set_installation_mode_text_sensor(sens))

    if presence_config := config.get(CONF_PRESENCE):
        sens = await binary_sensor.new_binary_sensor(presence_config)
        cg.add(var.set_presence_binary_sensor(sens))

    if target_count_config := config.get(CONF_TARGET_COUNT):
        sens = await sensor.new_sensor(target_count_config)
        cg.add(var.set_target_count_sensor(sens))

    if reporting_config := config.get(CONF_REPORTING):
        reporting_switch = cg.new_Pvariable(reporting_config[CONF_ID])
        await cg.register_component(reporting_switch, reporting_config)
        await switch.register_switch(reporting_switch, reporting_config)
        cg.add(reporting_switch.set_parent(var))
        cg.add(var.set_reporting_switch(reporting_switch))

    number_fields = [
        (CONF_INSTALLATION_HEIGHT, 0, 1.6, 2.6, 0.01), (CONF_INSTALLATION_ANGLE, 1, 0, 30, 0.01),
        (CONF_DETECTION_DISTANCE, 2, 0, 6, 0.1), (CONF_START_ANGLE, 3, -60, 360, 0.1),
        (CONF_END_ANGLE, 4, -60, 360, 0.1),
    ]
    for key, field, min_value, max_value, step in number_fields:
        if number_config := config.get(key):
            control = cg.new_Pvariable(number_config[CONF_ID])
            await number.register_number(control, number_config, min_value=min_value,
                                         max_value=max_value, step=step)
            cg.add(control.set_parent(var))
            cg.add(control.set_field(field))
            cg.add(var.set_config_number(field, control))

    if mode_config := config.get(CONF_INSTALLATION_MODE + "_select"):
        control = cg.new_Pvariable(mode_config[CONF_ID])
        await select.register_select(control, mode_config, options=["Side", "Top"])
        cg.add(control.set_parent(var))
        cg.add(var.set_installation_mode_select(control))

    for index, target_key in enumerate(CONF_TARGETS):
        target_config = config.get(target_key)
        if target_config is None:
            continue

        if x_config := target_config.get(CONF_X):
            sens = await sensor.new_sensor(x_config)
            cg.add(var.set_target_x_sensor(index, sens))

        if y_config := target_config.get(CONF_Y):
            sens = await sensor.new_sensor(y_config)
            cg.add(var.set_target_y_sensor(index, sens))

        if distance_config := target_config.get(CONF_DISTANCE):
            sens = await sensor.new_sensor(distance_config)
            cg.add(var.set_target_distance_sensor(index, sens))

        if angle_config := target_config.get(CONF_ANGLE):
            sens = await sensor.new_sensor(angle_config)
            cg.add(var.set_target_angle_sensor(index, sens))

    cg.add(var.set_flush_timeout(config[CONF_FLUSH_TIMEOUT].total_milliseconds))
    cg.add(var.set_max_buffer_size(config[CONF_MAX_BUFFER_SIZE]))
    cg.add(var.set_baud_scan(config[CONF_BAUD_SCAN]))
    cg.add(var.set_no_data_log_interval(config[CONF_NO_DATA_LOG_INTERVAL].total_milliseconds))
    cg.add(var.set_publish_interval(config[CONF_PUBLISH_INTERVAL].total_milliseconds))
