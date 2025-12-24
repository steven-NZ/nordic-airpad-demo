/*
 * Central Thread Implementation
 *
 * Orchestrates all sensor reading, communication, and output control
 */

#include "central_thread.h"
#include "../drivers/radio/esb_handler.h"
#include "../drivers/button/btn_handler.h"
#include "../drivers/gesture/mgc_handler.h"
#include "../drivers/imu/imu_handler.h"
#include "../drivers/vibration/vib_handler.h"
#include "../drivers/led/led_handler.h"
#include "../middleware/sensor_fusion/imu_fusion.h"
#include "../middleware/sensor_fusion/imu_math.h"
#include "../drivers/driver_framework.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(central_thread, LOG_LEVEL_INF);

#define CENTRAL_THREAD_STACK_SIZE 4096
#define CENTRAL_THREAD_PRIORITY 5
#define CENTRAL_THREAD_PERIOD_MS 10

/* Driver file descriptors */
static driver_fd_t imu_fd = DRIVER_FD_INVALID;
static driver_fd_t btn_fd = DRIVER_FD_INVALID;
static driver_fd_t mgc_fd = DRIVER_FD_INVALID;
static driver_fd_t esb_fd = DRIVER_FD_INVALID;
static driver_fd_t vib_fd = DRIVER_FD_INVALID;
static driver_fd_t led_fd = DRIVER_FD_INVALID;

/* IMU fusion state */
static imu_fusion_state_t *fusion_state = NULL;

/* Response data for ESB ACK payload */
static response_data_t response_data;

/* LED control state tracking */
typedef struct {
	uint8_t color_index[4];      /* Color index for each cardinal direction (0-5) */
	uint8_t previous_touch;      /* Previous touch electrode state (bits 0-3) */
	uint16_t hue_angle;          /* HSV hue for airwheel LEDs (0-359 degrees) */
	uint8_t cycle_counter;       /* Counter for 20Hz update rate (0-255) */
	bool airwheel_was_active;    /* Previous airwheel active state */
} led_control_state_t;

static led_control_state_t led_state = {
	.color_index = {0, 0, 0, 0},  /* Start all at red */
	.previous_touch = 0,
	.hue_angle = 0,
	.cycle_counter = 0,
	.airwheel_was_active = false
};

/* Rainbow color palette (6 colors: Red, Orange, Yellow, Green, Blue, Purple) */
static const led_color_t rainbow_colors[6] = {
	{.red = 255, .green = 0,   .blue = 0},    /* Red */
	{.red = 255, .green = 127, .blue = 0},    /* Orange */
	{.red = 255, .green = 255, .blue = 0},    /* Yellow */
	{.red = 0,   .green = 255, .blue = 0},    /* Green */
	{.red = 0,   .green = 0,   .blue = 255},  /* Blue */
	{.red = 127, .green = 0,   .blue = 255}   /* Purple */
};

/* Touch electrode to LED index mapping */
#define LED_INDEX_NORTH 0  /* LED 1 (index 0) */
#define LED_INDEX_EAST  1  /* LED 2 (index 1) */
#define LED_INDEX_SOUTH 2  /* LED 3 (index 2) */
#define LED_INDEX_WEST  3  /* LED 4 (index 3) */
#define LED_INDEX_CW    4  /* LED 5 (index 4) - Clockwise airwheel */
#define LED_INDEX_CCW   5  /* LED 6 (index 5) - Counter-clockwise airwheel */

/* Cardinal direction index for color_index array */
#define CARDINAL_NORTH  0
#define CARDINAL_EAST   1
#define CARDINAL_SOUTH  2
#define CARDINAL_WEST   3

/* Color cycling parameters */
#define HUE_INCREMENT_PER_UPDATE  10  /* Degrees per 20Hz update (36 steps = 360°) */
#define AIRWHEEL_UPDATE_DIVIDER   5   /* Update every 5 cycles (50ms @ 100Hz = 20Hz) */
#define LED_BRIGHTNESS_DEFAULT    15  /* Mid-range brightness (0-31) */

/*
 * Convert HSV color space to RGB
 * H: Hue (0-359 degrees)
 * S: Saturation (0-255, use 255 for full saturation)
 * V: Value/Brightness (0-255, use 255 for full brightness)
 * Output: led_color_t with RGB values
 */
static led_color_t hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v)
{
	led_color_t rgb;
	uint8_t region, remainder, p, q, t;

	/* Ensure hue wraps at 360 */
	h = h % 360;

	if (s == 0) {
		/* Achromatic (grey) */
		rgb.red = v;
		rgb.green = v;
		rgb.blue = v;
		return rgb;
	}

	/* Calculate region (0-5) and position within region */
	region = h / 60;
	remainder = (h % 60) * 255 / 60;

	p = (v * (255 - s)) / 255;
	q = (v * (255 - ((s * remainder) / 255))) / 255;
	t = (v * (255 - ((s * (255 - remainder)) / 255))) / 255;

	switch (region) {
	case 0:
		rgb.red = v; rgb.green = t; rgb.blue = p;
		break;
	case 1:
		rgb.red = q; rgb.green = v; rgb.blue = p;
		break;
	case 2:
		rgb.red = p; rgb.green = v; rgb.blue = t;
		break;
	case 3:
		rgb.red = p; rgb.green = q; rgb.blue = v;
		break;
	case 4:
		rgb.red = t; rgb.green = p; rgb.blue = v;
		break;
	default: /* case 5: */
		rgb.red = v; rgb.green = p; rgb.blue = q;
		break;
	}

	return rgb;
}

/*
 * Process touch events and update cardinal direction LEDs (LEDs 1-4)
 * Detects rising edge (press) on each electrode and cycles to next color
 */
static void process_touch_led_control(uint8_t touch_electrodes)
{
	uint8_t changed = touch_electrodes ^ led_state.previous_touch;
	uint8_t pressed = changed & touch_electrodes;  /* Rising edges only */

	/* Update previous state */
	led_state.previous_touch = touch_electrodes;

	/* Check each cardinal direction for press event */
	if (pressed != 0) {
		led_control_t led_ctrl;
		led_ctrl.brightness = LED_BRIGHTNESS_DEFAULT;

		/* North electrode (bit 2) → LED 1 */
		if (pressed & MGC3130_TOUCH_NORTH) {
			led_state.color_index[CARDINAL_NORTH] =
				(led_state.color_index[CARDINAL_NORTH] + 1) % 6;
			led_ctrl.led_index = LED_INDEX_NORTH;
			led_ctrl.color = rainbow_colors[led_state.color_index[CARDINAL_NORTH]];

			ssize_t result = led_write(led_fd, &led_ctrl, sizeof(led_ctrl));
			if (result < 0) {
				LOG_ERR("Failed to update LED 1 (North): %d", (int)result);
			} else {
				LOG_INF("LED 1 (North) → Color %d",
				        led_state.color_index[CARDINAL_NORTH]);
			}
		}

		/* East electrode (bit 3) → LED 2 */
		if (pressed & MGC3130_TOUCH_EAST) {
			led_state.color_index[CARDINAL_EAST] =
				(led_state.color_index[CARDINAL_EAST] + 1) % 6;
			led_ctrl.led_index = LED_INDEX_EAST;
			led_ctrl.color = rainbow_colors[led_state.color_index[CARDINAL_EAST]];

			ssize_t result = led_write(led_fd, &led_ctrl, sizeof(led_ctrl));
			if (result < 0) {
				LOG_ERR("Failed to update LED 2 (East): %d", (int)result);
			} else {
				LOG_INF("LED 2 (East) → Color %d",
				        led_state.color_index[CARDINAL_EAST]);
			}
		}

		/* South electrode (bit 0) → LED 3 */
		if (pressed & MGC3130_TOUCH_SOUTH) {
			led_state.color_index[CARDINAL_SOUTH] =
				(led_state.color_index[CARDINAL_SOUTH] + 1) % 6;
			led_ctrl.led_index = LED_INDEX_SOUTH;
			led_ctrl.color = rainbow_colors[led_state.color_index[CARDINAL_SOUTH]];

			ssize_t result = led_write(led_fd, &led_ctrl, sizeof(led_ctrl));
			if (result < 0) {
				LOG_ERR("Failed to update LED 3 (South): %d", (int)result);
			} else {
				LOG_INF("LED 3 (South) → Color %d",
				        led_state.color_index[CARDINAL_SOUTH]);
			}
		}

		/* West electrode (bit 1) → LED 4 */
		if (pressed & MGC3130_TOUCH_WEST) {
			led_state.color_index[CARDINAL_WEST] =
				(led_state.color_index[CARDINAL_WEST] + 1) % 6;
			led_ctrl.led_index = LED_INDEX_WEST;
			led_ctrl.color = rainbow_colors[led_state.color_index[CARDINAL_WEST]];

			ssize_t result = led_write(led_fd, &led_ctrl, sizeof(led_ctrl));
			if (result < 0) {
				LOG_ERR("Failed to update LED 4 (West): %d", (int)result);
			} else {
				LOG_INF("LED 4 (West) → Color %d",
				        led_state.color_index[CARDINAL_WEST]);
			}
		}
	}
}

/*
 * Process airwheel rotation and update LEDs 5-6
 * CW rotation → LED 5 with RGB cycling
 * CCW rotation → LED 6 with RGB cycling
 * Updates at 20Hz (every 5 central thread cycles)
 */
static void process_airwheel_led_control(const mgc3130_esb_state_t *mgc_esb_state)
{
	led_control_t led_ctrl;
	led_ctrl.brightness = LED_BRIGHTNESS_DEFAULT;

	/* Detect airwheel activation/deactivation */
	bool airwheel_active = mgc_esb_state->airwheel_active;
	bool airwheel_became_active = airwheel_active && !led_state.airwheel_was_active;
	bool airwheel_became_inactive = !airwheel_active && led_state.airwheel_was_active;

	/* Update previous state */
	led_state.airwheel_was_active = airwheel_active;

	/* If airwheel just became inactive, turn off LEDs 5 & 6 */
	if (airwheel_became_inactive) {
		led_color_t off_color = {.red = 0, .green = 0, .blue = 0};

		/* Turn off LED 5 */
		led_ctrl.led_index = LED_INDEX_CW;
		led_ctrl.brightness = 0;
		led_ctrl.color = off_color;
		ssize_t result = led_write(led_fd, &led_ctrl, sizeof(led_ctrl));
		if (result < 0) {
			LOG_ERR("Failed to turn off LED 5: %d", (int)result);
		}

		/* Turn off LED 6 */
		led_ctrl.led_index = LED_INDEX_CCW;
		led_ctrl.brightness = 0;
		led_ctrl.color = off_color;
		result = led_write(led_fd, &led_ctrl, sizeof(led_ctrl));
		if (result < 0) {
			LOG_ERR("Failed to turn off LED 6: %d", (int)result);
		}

		LOG_INF("Airwheel inactive - LEDs 5 & 6 OFF");
		return;
	}

	/* If airwheel just became active, reset hue */
	if (airwheel_became_active) {
		led_state.hue_angle = 0;
		LOG_INF("Airwheel active - Starting RGB cycle");
	}

	/* Only update if airwheel is active */
	if (!airwheel_active) {
		return;
	}

	/* Increment cycle counter and check if update is needed (20Hz) */
	led_state.cycle_counter++;
	if ((led_state.cycle_counter % AIRWHEEL_UPDATE_DIVIDER) != 0) {
		return;  /* Not time to update yet */
	}

	/* Update hue angle (0-359 degrees, wraps automatically) */
	led_state.hue_angle = (led_state.hue_angle + HUE_INCREMENT_PER_UPDATE) % 360;

	/* Convert HSV to RGB (full saturation, full value) */
	led_color_t rgb_color = hsv_to_rgb(led_state.hue_angle, 255, 255);

	/* Update appropriate LED based on rotation direction */
	if (mgc_esb_state->airwheel_direction_cw) {
		/* Clockwise → LED 5 */
		led_ctrl.led_index = LED_INDEX_CW;
		led_ctrl.color = rgb_color;
		led_ctrl.brightness = LED_BRIGHTNESS_DEFAULT;

		ssize_t result = led_write(led_fd, &led_ctrl, sizeof(led_ctrl));
		if (result < 0) {
			LOG_ERR("Failed to update LED 5 (CW): %d", (int)result);
		}

		/* Turn off LED 6 (CCW) */
		led_ctrl.led_index = LED_INDEX_CCW;
		led_ctrl.brightness = 0;
		led_ctrl.color = (led_color_t){.red = 0, .green = 0, .blue = 0};
		led_write(led_fd, &led_ctrl, sizeof(led_ctrl));

	} else {
		/* Counter-clockwise → LED 6 */
		led_ctrl.led_index = LED_INDEX_CCW;
		led_ctrl.color = rgb_color;
		led_ctrl.brightness = LED_BRIGHTNESS_DEFAULT;

		ssize_t result = led_write(led_fd, &led_ctrl, sizeof(led_ctrl));
		if (result < 0) {
			LOG_ERR("Failed to update LED 6 (CCW): %d", (int)result);
		}

		/* Turn off LED 5 (CW) */
		led_ctrl.led_index = LED_INDEX_CW;
		led_ctrl.brightness = 0;
		led_ctrl.color = (led_color_t){.red = 0, .green = 0, .blue = 0};
		led_write(led_fd, &led_ctrl, sizeof(led_ctrl));
	}
}

/* Central thread entry point */
static void central_thread_entry(void *p1, void *p2, void *p3)
{
	ssize_t result;
	sensor_data_t sensor_packet;
	uint8_t rx_buf[32];
	uint8_t btn_state_raw;
	mgc3130_esb_state_t mgc_esb_state;

	LOG_INF("Central thread started");

	/* Open all drivers */
	imu_fd = imu_open(0);
	if (imu_fd < 0) {
		LOG_ERR("Failed to open IMU driver: %d", imu_fd);
		return;
	}
	LOG_INF("IMU driver opened with fd=%d", imu_fd);

	btn_fd = btn_open(0);
	if (btn_fd < 0) {
		LOG_ERR("Failed to open button driver: %d", btn_fd);
		imu_close(imu_fd);
		return;
	}
	LOG_INF("Button driver opened with fd=%d", btn_fd);

	mgc_fd = mgc_open();
	if (mgc_fd < 0) {
		LOG_ERR("Failed to open MGC handler: %d", mgc_fd);
		btn_close(btn_fd);
		imu_close(imu_fd);
		return;
	}
	LOG_INF("MGC handler opened with fd=%d", mgc_fd);

	esb_fd = esb_open(0);
	if (esb_fd < 0) {
		LOG_ERR("Failed to open ESB driver: %d", esb_fd);
		mgc_close(mgc_fd);
		btn_close(btn_fd);
		imu_close(imu_fd);
		return;
	}
	LOG_INF("ESB driver opened with fd=%d", esb_fd);

	vib_fd = vib_open(0);
	if (vib_fd < 0) {
		LOG_ERR("Failed to open vibration driver: %d", vib_fd);
		esb_close(esb_fd);
		mgc_close(mgc_fd);
		btn_close(btn_fd);
		imu_close(imu_fd);
		return;
	}
	LOG_INF("Vibration driver opened with fd=%d", vib_fd);

	led_fd = led_open(0);
	if (led_fd < 0) {
		LOG_ERR("Failed to open LED driver: %d", led_fd);
		vib_close(vib_fd);
		esb_close(esb_fd);
		mgc_close(mgc_fd);
		btn_close(btn_fd);
		imu_close(imu_fd);
		return;
	}
	LOG_INF("LED driver opened with fd=%d", led_fd);

	/* Initialize IMU fusion */
	imu_fusion_config_t fusion_config = {
		.slerp_power = 0.02f,      /* 2% correction per sample */
		.enable_gyro = true,
		.enable_accel = true,
		.sample_rate_hz = 100
	};

	fusion_state = imu_fusion_init(&fusion_config);
	if (!fusion_state) {
		LOG_ERR("Failed to initialize IMU fusion");
		led_close(led_fd);
		vib_close(vib_fd);
		esb_close(esb_fd);
		mgc_close(mgc_fd);
		btn_close(btn_fd);
		imu_close(imu_fd);
		return;
	}
	LOG_INF("IMU fusion initialized (SLERP power: %.2f)",
	        (double)fusion_config.slerp_power);

	/* Initialize MGC ESB state */
	memset(&mgc_esb_state, 0, sizeof(mgc3130_esb_state_t));

	/* Initialize response data */
	memset(&response_data, 0, sizeof(response_data_t));

	/* Enable ACK payload transmission */
	bool enable_ack = true;
	result = esb_ioctl(esb_fd, ESB_IOCTL_ENABLE_ACK_PL, &enable_ack);
	if (result < 0) {
		LOG_ERR("Failed to enable ACK payload: %d", (int)result);
	}

	LOG_INF("All drivers opened successfully");

	while (1) {
		/* Read IMU sensor data */
		imu_data_t imu_data;
		imu_fusion_output_t fusion_output = {0};
		result = imu_read(imu_fd, &imu_data, sizeof(imu_data));
		if (result < 0) {
			LOG_WRN("IMU read failed: %d", (int)result);
		} else {
			/*  Accel: m/s², Gyro: rad/s */
			vector3_t accel, gyro;

			accel.x = imu_data.accel_x;
			accel.y = imu_data.accel_y;
			accel.z = imu_data.accel_z;

			gyro.x = imu_data.gyro_x;
			gyro.y = imu_data.gyro_y;
			gyro.z = imu_data.gyro_z;

			/* Get timestamp (convert ms to μs) */
			uint64_t timestamp_us = k_uptime_get() * 1000ULL;

			/* Update fusion */
			int ret = imu_fusion_update(fusion_state, &accel, &gyro,
			                            timestamp_us, &fusion_output);

			if (ret == 0 && fusion_output.valid) {
				/* Log every 10th sample (10Hz) to reduce verbosity */
				static uint32_t log_counter = 0;
				if (++log_counter >= 1000) {
					log_counter = 0;
					LOG_INF("Quat[w:%.3f x:%.3f y:%.3f z:%.3f] "
					        "Euler[R:%.1f° P:%.1f° Y:%.1f°]",
					        (double)fusion_output.orientation.w,
					        (double)fusion_output.orientation.x,
					        (double)fusion_output.orientation.y,
					        (double)fusion_output.orientation.z,
					        (double)(fusion_output.euler.roll * IMU_RAD_TO_DEG),
					        (double)(fusion_output.euler.pitch * IMU_RAD_TO_DEG),
					        (double)(fusion_output.euler.yaw * IMU_RAD_TO_DEG));
				}
			} else {
				LOG_WRN("Fusion update failed: %d", ret);
			}
		}

		/* Read button state */
		result = btn_read(btn_fd, &btn_state_raw, sizeof(btn_state_raw));
		if (result < 0) {
			LOG_WRN("Button read failed: %d", (int)result);
			btn_state_raw = 0;
		}

		/* Poll MGC3130 for sensor data (processes internally) */
		mgc3130_sensor_output_t mgc_sensor_output;
		result = mgc_read(mgc_fd, &mgc_sensor_output, sizeof(mgc_sensor_output));
		if (result < 0 && result != DRIVER_ERR_AGAIN) {
			LOG_WRN("MGC read failed: %d", (int)result);
		}

		/* Get MGC state for ESB transmission */
		result = mgc_ioctl(mgc_fd, MGC_IOCTL_GET_ESB_STATE, &mgc_esb_state);
		if (result < 0) {
			LOG_WRN("MGC ioctl GET_ESB_STATE failed: %d", (int)result);
			/* Keep using previous state */
		}

		/* Process touch events for LED control (LEDs 1-4) */
		if (led_fd != DRIVER_FD_INVALID) {
			process_touch_led_control(mgc_esb_state.touch_electrodes);
		}

		/* Process airwheel for LED control (LEDs 5-6) */
		if (led_fd != DRIVER_FD_INVALID) {
			process_airwheel_led_control(&mgc_esb_state);
		}

		/* Build sensor data packet with quaternion */
		sensor_packet.btn_state = btn_state_raw;

		/* Pack MGC state (touch + airwheel) */
		sensor_packet.mgc_state = MGC_PACK_STATE(
			mgc_esb_state.touch_electrodes,
			mgc_esb_state.airwheel_active,
			mgc_esb_state.airwheel_direction_cw,
			mgc_esb_state.airwheel_velocity
		);

		/* Get latest quaternion from fusion output */
		if (fusion_output.valid) {
			/* Convert quaternion float to int16_t with scaling */
			sensor_packet.quat_w = QUAT_FLOAT_TO_INT16(fusion_output.orientation.w);
			sensor_packet.quat_x = QUAT_FLOAT_TO_INT16(fusion_output.orientation.x);
			sensor_packet.quat_y = QUAT_FLOAT_TO_INT16(fusion_output.orientation.y);
			sensor_packet.quat_z = QUAT_FLOAT_TO_INT16(fusion_output.orientation.z);
		} else {
			/* Fusion not ready - send identity quaternion */
			sensor_packet.quat_w = QUAT_FLOAT_TO_INT16(1.0f);
			sensor_packet.quat_x = 0;
			sensor_packet.quat_y = 0;
			sensor_packet.quat_z = 0;
		}

		/* Send sensor data via ESB */
		result = esb_write(esb_fd, &sensor_packet, sizeof(sensor_packet));
		if (result < 0) {
			LOG_ERR("ESB write failed: %d", (int)result);
		}

		/* Check for received ESB data (commands from receiver) */
		result = esb_read(esb_fd, rx_buf, sizeof(rx_buf));
		if (result > 0) {
			/* Process received command data */
			LOG_INF("Received ESB command, len: %d", (int)result);

			/* Expect response_data_t structure */
			if (result >= sizeof(response_data_t)) {
				response_data_t *cmd = (response_data_t *)rx_buf;

				/* Update vibration state based on command */
				vib_control_t vib_ctrl;
				vib_ctrl.intensity = cmd->vibration_intensity;  /* 0-255 intensity value */
				vib_ctrl.enable = (cmd->vibration_intensity > 0) ? 1 : 0;  /* On if intensity > 0 */

				ssize_t vib_result = vib_write(vib_fd, &vib_ctrl, sizeof(vib_ctrl));
				if (vib_result < 0) {
					LOG_ERR("Failed to update vibration: %d", (int)vib_result);
				} else {
					LOG_INF("Vibration intensity=%u", vib_ctrl.intensity);
				}

				/* Update response data to echo back current state */
				response_data.vibration_intensity = vib_ctrl.intensity;

				/* Update ESB ACK payload with current state */
				result = esb_ioctl(esb_fd, ESB_IOCTL_SET_ACK_PAYLOAD, &response_data);
				if (result < 0) {
					LOG_ERR("Failed to set ACK payload: %d", (int)result);
				}
			} else {
				LOG_WRN("Received incomplete command (len=%d)", (int)result);
			}
		} else if (result < 0 && result != DRIVER_ERR_AGAIN) {
			/* Log errors other than "no data available" */
			LOG_WRN("ESB read error: %d", (int)result);
		}

		/* Sleep for 10ms (100Hz) */
		k_sleep(K_MSEC(CENTRAL_THREAD_PERIOD_MS));
	}

	/* Cleanup (never reached for now) */
	if (fusion_state) {
		imu_fusion_destroy(fusion_state);
	}
	if (led_fd != DRIVER_FD_INVALID) {
		led_close(led_fd);
	}
	vib_close(vib_fd);
	esb_close(esb_fd);
	mgc_close(mgc_fd);
	btn_close(btn_fd);
	imu_close(imu_fd);
}

/* Define central thread */
K_THREAD_DEFINE(central_thread, CENTRAL_THREAD_STACK_SIZE,
		central_thread_entry, NULL, NULL, NULL,
		CENTRAL_THREAD_PRIORITY, 0, 0);

int central_thread_init(void)
{
	LOG_INF("Central thread initialized");
	return 0;
}
