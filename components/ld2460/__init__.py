import esphome.codegen as cg
from esphome.components import binary_sensor, number, select, sensor, switch, text_sensor, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_OCCUPANCY,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ENTITY_CATEGORY_CONFIG,
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
LD2460SensitivitySelect = ld2460_ns.class_("LD2460SensitivitySelect", select.Select)
LD2460TrackingNumber = ld2460_ns.class_("LD2460TrackingNumber", number.Number)
LD2460ExitEdgeSelect = ld2460_ns.class_("LD2460ExitEdgeSelect", select.Select)
LD2460ZoneSwitch = ld2460_ns.class_("LD2460ZoneSwitch", switch.Switch)

CONF_BAUD_SCAN = "baud_scan"
CONF_FLUSH_TIMEOUT = "flush_timeout"
CONF_MAX_BUFFER_SIZE = "max_buffer_size"
CONF_NO_DATA_LOG_INTERVAL = "no_data_log_interval"
CONF_PUBLISH_INTERVAL = "publish_interval"
CONF_PRESENCE_TIMEOUT = "presence_timeout"
CONF_STATIONARY_PRESENCE_TIMEOUT = "stationary_presence_timeout"
CONF_STATIONARY_DWELL = "stationary_dwell"
CONF_EXIT_TIMEOUT = "exit_timeout"
CONF_EDGE_MARGIN = "edge_margin"
CONF_TRACKING = "tracking"
CONF_RADAR_AVAILABLE = "radar_available"
CONF_RADAR_HEALTH_TIMEOUT = "radar_health_timeout"
CONF_INITIAL_VALUE = "initial_value"
CONF_POSITION_UPDATE_THRESHOLD = "position_update_threshold"
CONF_REPORTING = "reporting"
CONF_PRESENCE = "presence"
CONF_TARGET_COUNT = "target_count"
CONF_FIRMWARE = "firmware"
CONF_INSTALLATION_MODE = "installation_mode"
CONF_INSTALLATION_HEIGHT = "installation_height"
CONF_INSTALLATION_ANGLE = "installation_angle"
CONF_DETECTION_DISTANCE = "detection_distance"
CONF_START_ANGLE = "start_angle"
CONF_END_ANGLE = "end_angle"
CONF_SENSITIVITY = "sensitivity"
CONF_X = "x"
CONF_Y = "y"
CONF_DISTANCE = "distance"
CONF_ANGLE = "angle"
CONF_TARGETS = [f"target_{index}" for index in range(1, 6)]

# Fields 0..7: corners; 8..9: selected exit edges; 10..14: timers/margin.
TRACKING_NUMBERS = {
    **{f"point_{point}_{axis}": ((point - 1) * 2 + offset, -12, 12, 0.1, UNIT_METER)
       for point in range(1, 5) for offset, axis in enumerate(("x", "y"))},
    CONF_PRESENCE_TIMEOUT: (10, 1, 3600, 1, "s"),
    CONF_STATIONARY_PRESENCE_TIMEOUT: (11, 1, 86400, 1, "s"),
    CONF_STATIONARY_DWELL: (12, 1, 300, 1, "s"),
    CONF_EXIT_TIMEOUT: (13, 1, 120, 1, "s"),
    CONF_EDGE_MARGIN: (14, 0.1, 2, 0.1, UNIT_METER),
}

TRACKING_SCHEMA = cv.Schema({
    **{cv.Optional(key): number.number_schema(
        LD2460TrackingNumber, unit_of_measurement=unit, entity_category=ENTITY_CATEGORY_CONFIG,
    ).extend({cv.Optional(CONF_INITIAL_VALUE): cv.float_range(min=minimum, max=maximum)})
       for key, (_, minimum, maximum, _, unit) in TRACKING_NUMBERS.items()},
    **{cv.Optional(f"exit_edge_{slot}"): select.select_schema(
        LD2460ExitEdgeSelect, entity_category=ENTITY_CATEGORY_CONFIG,
    ).extend({cv.Optional("initial_edge", default=0): cv.int_range(min=0, max=4)})
       for slot in range(1, 3)},
    cv.Optional("zone_enabled"): switch.switch_schema(
        LD2460ZoneSwitch, entity_category=ENTITY_CATEGORY_CONFIG,
    ).extend({cv.Optional("initial_state", default=False): cv.boolean}),
})

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

def validate_presence_timeouts(config):
    tracking = config.get(CONF_TRACKING, {})
    ordinary = tracking.get(CONF_PRESENCE_TIMEOUT, {}).get(
        CONF_INITIAL_VALUE, config[CONF_PRESENCE_TIMEOUT].total_milliseconds / 1000)
    stationary = tracking.get(CONF_STATIONARY_PRESENCE_TIMEOUT, {}).get(
        CONF_INITIAL_VALUE, config[CONF_STATIONARY_PRESENCE_TIMEOUT].total_milliseconds / 1000)
    if stationary < ordinary:
        raise cv.Invalid("stationary_presence_timeout must be at least presence_timeout")
    for key, maximum in ((CONF_PRESENCE_TIMEOUT, 3600), (CONF_STATIONARY_PRESENCE_TIMEOUT, 86400),
                         (CONF_STATIONARY_DWELL, 300), (CONF_EXIT_TIMEOUT, 120)):
        if not 1000 <= config[key].total_milliseconds <= maximum * 1000:
            raise cv.Invalid(f"{key} must be between 1s and {maximum}s")
    if config[CONF_RADAR_HEALTH_TIMEOUT].total_milliseconds < 75000:
        raise cv.Invalid("radar_health_timeout must be at least 75s to allow the sparse health probe")
    if tracking.get("zone_enabled", {}).get("initial_state", False):
        defaults = [-3, 0, 3, 0, 3, 6, -3, 6]
        points = [tuple(round(tracking.get(f"point_{i}_{axis}", {}).get(CONF_INITIAL_VALUE,
                            defaults[(i - 1) * 2 + j]) * 10)
                        for j, axis in enumerate(("x", "y"))) for i in range(1, 5)]
        def cross(a, b, c):
            return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        def on_segment(a, b, p):
            return (cross(a, b, p) == 0 and min(a[0], b[0]) <= p[0] <= max(a[0], b[0])
                    and min(a[1], b[1]) <= p[1] <= max(a[1], b[1]))
        def intersects(a, b, c, d):
            return (on_segment(a, b, c) or on_segment(a, b, d) or on_segment(c, d, a) or on_segment(c, d, b)
                    or ((cross(a, b, c) > 0) != (cross(a, b, d) > 0)
                        and (cross(c, d, a) > 0) != (cross(c, d, b) > 0)))
        area = sum(a[0] * b[1] - a[1] * b[0] for a, b in zip(points, points[1:] + points[:1]))
        if (len(set(points)) != 4 or area == 0 or intersects(*points)
                or intersects(points[1], points[2], points[3], points[0])):
            raise cv.Invalid("tracking zone needs four distinct corners in perimeter order without crossed edges")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LD2460Component),
            cv.Optional(CONF_BAUD_SCAN, default=True): cv.boolean,
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
            cv.Optional(CONF_RADAR_AVAILABLE): binary_sensor.binary_sensor_schema(
                device_class="connectivity", entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_TRACKING): TRACKING_SCHEMA,
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
            cv.Optional(CONF_SENSITIVITY): select.select_schema(LD2460SensitivitySelect),
            **{cv.Optional(target): TARGET_SCHEMA for target in CONF_TARGETS},
            cv.Optional(CONF_FLUSH_TIMEOUT, default="100ms"): cv.positive_time_period_milliseconds,
            # A five-target report is 11 + 4 * 5 = 31 bytes.
            cv.Optional(CONF_MAX_BUFFER_SIZE, default=48): cv.int_range(min=31, max=512),
            cv.Optional(CONF_NO_DATA_LOG_INTERVAL, default="10s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_PUBLISH_INTERVAL, default="5s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_PRESENCE_TIMEOUT, default="20s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_STATIONARY_PRESENCE_TIMEOUT, default="60min"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_STATIONARY_DWELL, default="10s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_EXIT_TIMEOUT, default="5s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_EDGE_MARGIN, default=0.5): cv.float_range(min=0.1, max=2),
            cv.Optional(CONF_RADAR_HEALTH_TIMEOUT, default="90s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_POSITION_UPDATE_THRESHOLD, default=0.5): cv.float_range(min=0.1, max=5.0),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA),
    validate_presence_timeouts,
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
    cg.add(var.set_tracking_preference_key(str(config[CONF_ID])))
    cg.add(var.set_presence_timeout(config[CONF_PRESENCE_TIMEOUT].total_milliseconds))
    cg.add(var.set_stationary_presence_timeout(config[CONF_STATIONARY_PRESENCE_TIMEOUT].total_milliseconds))
    cg.add(var.set_stationary_dwell(config[CONF_STATIONARY_DWELL].total_milliseconds))
    cg.add(var.set_exit_timeout(config[CONF_EXIT_TIMEOUT].total_milliseconds))
    cg.add(var.set_edge_margin(config[CONF_EDGE_MARGIN]))
    cg.add(var.set_radar_health_timeout(config[CONF_RADAR_HEALTH_TIMEOUT].total_milliseconds))

    tracking = config.get(CONF_TRACKING, {})
    for key, (field, minimum, maximum, step, _) in TRACKING_NUMBERS.items():
        if control_config := tracking.get(key):
            control = cg.new_Pvariable(control_config[CONF_ID])
            await number.register_number(control, control_config, min_value=minimum, max_value=maximum, step=step)
            cg.add(control.set_parent(var))
            cg.add(control.set_field(field))
            cg.add(var.set_tracking_number(field, control))
            if CONF_INITIAL_VALUE in control_config:
                cg.add(var.set_tracking_initial_value(field, control_config[CONF_INITIAL_VALUE]))
    for slot in range(2):
        if control_config := tracking.get(f"exit_edge_{slot + 1}"):
            control = cg.new_Pvariable(control_config[CONF_ID])
            await select.register_select(control, control_config, options=["Disabled", "P1-P2", "P2-P3", "P3-P4", "P4-P1"])
            cg.add(control.set_parent(var))
            cg.add(control.set_slot(slot))
            cg.add(var.set_exit_edge_select(slot, control))
            cg.add(var.set_tracking_initial_value(8 + slot, control_config["initial_edge"]))
    if control_config := tracking.get("zone_enabled"):
        control = cg.new_Pvariable(control_config[CONF_ID])
        await switch.register_switch(control, control_config)
        cg.add(control.set_parent(var))
        cg.add(var.set_zone_switch(control))
        cg.add(var.set_zone_initial_enabled(control_config["initial_state"]))

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

    if sensor_config := config.get(CONF_RADAR_AVAILABLE):
        sens = await binary_sensor.new_binary_sensor(sensor_config)
        cg.add(var.set_radar_available_sensor(sens))

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

    if sensitivity_config := config.get(CONF_SENSITIVITY):
        control = cg.new_Pvariable(sensitivity_config[CONF_ID])
        await select.register_select(control, sensitivity_config, options=["High", "Medium", "Low"])
        cg.add(control.set_parent(var))
        cg.add(var.set_sensitivity_select(control))

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
    cg.add(var.set_position_update_threshold(config[CONF_POSITION_UPDATE_THRESHOLD]))
