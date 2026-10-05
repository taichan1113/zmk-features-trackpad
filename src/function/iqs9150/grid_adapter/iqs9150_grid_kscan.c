/*
 * Travelers Pad IQS9150 grid and edge gesture function.
 *
 * Short, stationary one-finger contacts are exposed as virtual KSCAN keys,
 * so their actions stay in the normal ZMK keymap instead of in this driver.
 */

#define DT_DRV_COMPAT zmk_travelers_pad_iqs9150_grid_kscan

#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(travelers_pad_grid, LOG_LEVEL_INF);

struct grid_config {
	uint16_t rows;
	uint16_t columns;
	uint16_t surface_x;
	uint16_t surface_y;
	uint16_t edge_size;
	uint16_t tap_max_ms;
	uint16_t tap_max_move;
	const uint16_t *edge_keys;
	bool has_edge_keys;
};

struct grid_data {
	kscan_callback_t callback;
	uint16_t x;
	uint16_t y;
	uint16_t start_x;
	uint16_t start_y;
	uint8_t fingers;
	uint8_t max_fingers;
	int64_t started_ms;
	bool moved;
	bool reset_frame;
};

static int absolute_int(int value) {
	return value < 0 ? -value : value;
}

static int grid_configure(const struct device *dev, kscan_callback_t callback) {
	struct grid_data *data = dev->data;

	if (callback == NULL) {
		return -EINVAL;
	}
	data->callback = callback;
	return 0;
}

static const struct kscan_driver_api grid_api = {
	.config = grid_configure,
};

static bool grid_cell_is_valid(const struct grid_config *cfg, uint16_t row, uint16_t column) {
	return row < cfg->rows && column < cfg->columns;
}

static bool grid_edge_cell(const struct grid_config *cfg, uint16_t x, uint16_t y, uint16_t *row,
			   uint16_t *column) {
	uint8_t edge;

	if (!cfg->has_edge_keys || cfg->edge_size == 0) {
		return false;
	}
	if (x <= cfg->edge_size) {
		edge = 0;
	} else if (x + cfg->edge_size >= cfg->surface_x) {
		edge = 1;
	} else if (y <= cfg->edge_size) {
		edge = 2;
	} else if (y + cfg->edge_size >= cfg->surface_y) {
		edge = 3;
	} else {
		return false;
	}
	*row = cfg->edge_keys[edge * 2];
	*column = cfg->edge_keys[(edge * 2) + 1];
	return grid_cell_is_valid(cfg, *row, *column);
}

static void grid_emit_tap(const struct device *dev) {
	const struct grid_config *cfg = dev->config;
	struct grid_data *data = dev->data;
	uint16_t row;
	uint16_t column;

	if (data->callback == NULL) {
		return;
	}
	if (!grid_edge_cell(cfg, data->start_x, data->start_y, &row, &column)) {
		row = MIN((uint32_t)data->start_y * cfg->rows / MAX(cfg->surface_y, 1), cfg->rows - 1);
		column = MIN((uint32_t)data->start_x * cfg->columns / MAX(cfg->surface_x, 1),
			     cfg->columns - 1);
	}
	LOG_INF("grid tap row=%u column=%u", row, column);
	data->callback(dev, row, column, true);
	data->callback(dev, row, column, false);
}

static void grid_process_frame(const struct device *dev) {
	const struct grid_config *cfg = dev->config;
	struct grid_data *data = dev->data;
	int64_t now = k_uptime_get();

	if (data->reset_frame) {
		data->fingers = 0;
		data->max_fingers = 0;
		data->moved = false;
		data->reset_frame = false;
		return;
	}

	if (data->fingers != 0 && data->max_fingers == 0) {
		data->started_ms = now;
		data->start_x = data->x;
		data->start_y = data->y;
		data->moved = false;
	}
	if (data->fingers != 0) {
		data->max_fingers = MAX(data->max_fingers, data->fingers);
		if (absolute_int((int)data->x - (int)data->start_x) > cfg->tap_max_move ||
		    absolute_int((int)data->y - (int)data->start_y) > cfg->tap_max_move) {
			data->moved = true;
		}
		return;
	}
	if (data->max_fingers == 1 && !data->moved && now - data->started_ms <= cfg->tap_max_ms) {
		grid_emit_tap(dev);
	}
	data->max_fingers = 0;
}

static void grid_handle_input(const struct device *dev, struct input_event *evt) {
	struct grid_data *data = dev->data;

	if (evt->type == INPUT_EV_ABS) {
		if (evt->code == INPUT_ABS_X) {
			data->x = evt->value;
		} else if (evt->code == INPUT_ABS_Y) {
			data->y = evt->value;
		} else if (evt->code == INPUT_ABS_Z) {
			data->fingers = evt->value;
		} else if (evt->code == INPUT_ABS_RZ && evt->value) {
			data->reset_frame = true;
		}
	}
	if (evt->sync) {
		grid_process_frame(dev);
	}
}

#define GRID_INST(n)                                                                             \
	static struct grid_data grid_data_##n;                                                        \
	static const uint16_t grid_edge_keys_##n[] = DT_INST_PROP_OR(n, edge_keys, {});              \
	static const struct grid_config grid_config_##n = {                                           \
		.rows = DT_INST_PROP(n, rows), .columns = DT_INST_PROP(n, columns),                    \
		.surface_x = DT_INST_PROP_OR(n, surface_x, 2000), .surface_y = DT_INST_PROP_OR(n, surface_y, 1213), \
		.edge_size = DT_INST_PROP_OR(n, edge_size, 0),                                            \
		.tap_max_ms = DT_INST_PROP_OR(n, tap_max_ms, 350),                                        \
		.tap_max_move = DT_INST_PROP_OR(n, tap_max_move, 75),                                    \
		.edge_keys = grid_edge_keys_##n,                                                          \
		.has_edge_keys = DT_INST_NODE_HAS_PROP(n, edge_keys),                                    \
	};                                                                                            \
	static void grid_input_handler_##n(struct input_event *evt) {                                 \
		grid_handle_input(DEVICE_DT_INST_GET(n), evt);                                          \
	}                                                                                             \
	INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n, input_device)),                        \
			      grid_input_handler_##n);                                                         \
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, &grid_data_##n, &grid_config_##n, POST_KERNEL,          \
			      CONFIG_TRAVELERS_PAD_IQS9150_FUNCTION_INIT_PRIORITY, &grid_api);

DT_INST_FOREACH_STATUS_OKAY(GRID_INST)
