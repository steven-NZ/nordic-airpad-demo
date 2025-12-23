/*
 * LED Handler Implementation
 * APA102C RGB LED Driver
 */

#include "led_handler.h"
#include "../driver_framework.h"
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <string.h>

LOG_MODULE_REGISTER(led_handler, LOG_LEVEL_INF);

/* APA102 Protocol Constants */
#define LED_COUNT 6
#define LED_BUFFER_SIZE 32  /* 4 start + 24 LED (6*4) + 4 end */

/*
 * Private Driver State
 */
typedef struct {
	driver_instance_t base;      /* Base driver instance (must be first) */
	const struct device *spi_dev; /* SPI device pointer */
	struct spi_config spi_cfg;   /* SPI configuration */

	uint8_t brightness[LED_COUNT];  /* Brightness per LED (0-31) */
	led_color_t colors[LED_COUNT];  /* RGB values per LED */

	uint8_t spi_buffer[LED_BUFFER_SIZE];  /* SPI transfer buffer */
	led_stats_t stats;                    /* Usage statistics */
} led_instance_t;

/* Single LED handler instance (static allocation) */
static led_instance_t led_inst;
static driver_instance_t *led_instances[1] = { &led_inst.base };

/*
 * Build APA102 SPI buffer from current LED state
 * Must be called with driver lock held
 */
static void led_build_spi_buffer(void)
{
	uint8_t *buf = led_inst.spi_buffer;
	int offset;

	/* Start frame: 4 bytes of 0x00 */
	buf[0] = buf[1] = buf[2] = buf[3] = 0x00;

	/* LED frames: 4 bytes per LED */
	for (int i = 0; i < LED_COUNT; i++) {
		offset = 4 + (i * 4);

		/* Byte 0: 111 (header) + 5-bit brightness */
		buf[offset + 0] = 0xE0 | (led_inst.brightness[i] & 0x1F);

		/* Byte 1: Blue */
		buf[offset + 1] = led_inst.colors[i].blue;

		/* Byte 2: Green */
		buf[offset + 2] = led_inst.colors[i].green;

		/* Byte 3: Red */
		buf[offset + 3] = led_inst.colors[i].red;
	}

	/* End frame: 4 bytes of 0xFF */
	buf[28] = buf[29] = buf[30] = buf[31] = 0x00;
}

/*
 * Update LEDs via SPI
 * Must be called with driver lock held
 */
static int led_update_spi(void)
{
	int err;
	struct spi_buf tx_buf;
	struct spi_buf_set tx_bufs;

	if (!led_inst.spi_dev) {
		LOG_ERR("SPI device not initialized");
		return DRIVER_ERR_NODEV;
	}

	/* Build SPI buffer */
	led_build_spi_buffer();

	/* Configure transfer */
	tx_buf.buf = led_inst.spi_buffer;
	tx_buf.len = LED_BUFFER_SIZE;

	tx_bufs.buffers = &tx_buf;
	tx_bufs.count = 1;

	/* Transmit (blocking) */
	err = spi_transceive(led_inst.spi_dev, &led_inst.spi_cfg, &tx_bufs, NULL);

	if (err) {
		LOG_ERR("SPI transfer failed: %d", err);
		led_inst.stats.spi_error_count++;
		return DRIVER_ERR_IO;
	}

	led_inst.stats.update_count++;
	return DRIVER_OK;
}

/*
 * Device initialization - called automatically at boot via SYS_INIT
 * Initializes the SPI peripheral
 */
static int led_device_init(void)
{
	int err;

	/* Initialize driver instance */
	DRIVER_INSTANCE_INIT(&led_inst.base);

	/* Initialize LED state to OFF */
	for (int i = 0; i < LED_COUNT; i++) {
		led_inst.brightness[i] = 0;
		led_inst.colors[i].red = 0;
		led_inst.colors[i].green = 0;
		led_inst.colors[i].blue = 0;
	}

	/* Clear statistics */
	memset(&led_inst.stats, 0, sizeof(led_stats_t));

	/* Get SPI device */
	led_inst.spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi3));
	if (!led_inst.spi_dev) {
		LOG_ERR("Failed to get SPI3 device");
		return -ENODEV;
	}

	/* Check if SPI device is ready */
	if (!device_is_ready(led_inst.spi_dev)) {
		LOG_ERR("SPI3 device not ready");
		led_inst.spi_dev = NULL;
		return -ENODEV;
	}

	/* Configure SPI */
	led_inst.spi_cfg.frequency = 4000000;  /* 4 MHz */
	led_inst.spi_cfg.operation = SPI_OP_MODE_MASTER |
	                              SPI_WORD_SET(8) |
	                              SPI_TRANSFER_MSB;
	led_inst.spi_cfg.slave = 0;
	/* No CS configuration needed for APA102 */
	led_inst.spi_cfg.cs.gpio.port = NULL;
	led_inst.spi_cfg.cs.gpio.pin = 0;
	led_inst.spi_cfg.cs.gpio.dt_flags = 0;
	led_inst.spi_cfg.cs.delay = 0;

	/* Send initial update (all LEDs off) */
	led_build_spi_buffer();

	struct spi_buf tx_buf = {
		.buf = led_inst.spi_buffer,
		.len = LED_BUFFER_SIZE
	};
	struct spi_buf_set tx_bufs = {
		.buffers = &tx_buf,
		.count = 1
	};

	err = spi_transceive(led_inst.spi_dev, &led_inst.spi_cfg, &tx_bufs, NULL);
	if (err) {
		LOG_WRN("Initial SPI transfer failed: %d", err);
		/* Continue anyway - device is initialized */
	}

	LOG_INF("LED handler initialized (6× APA102, SPI3 @ 4MHz, P1.12/P1.13)");
	return 0;
}

/* Register device initialization at boot (POST_KERNEL level, priority 91) */
SYS_INIT(led_device_init, POST_KERNEL, 91);

/*
 * Open LED driver - Returns file descriptor
 */
driver_fd_t led_open(uint32_t flags)
{
	driver_fd_t fd;

	/* Only one LED instance supported */
	if (led_inst.base.state == DRIVER_STATE_OPEN) {
		LOG_WRN("LED driver already open");
		return DRIVER_ERR_BUSY;
	}

	/* Check if SPI device was initialized successfully */
	if (!led_inst.spi_dev) {
		LOG_ERR("SPI device not available");
		return DRIVER_ERR_NODEV;
	}

	/* Allocate file descriptor (always returns 0 for single instance) */
	fd = driver_fd_alloc(led_instances, 1);
	if (fd == DRIVER_FD_INVALID) {
		LOG_ERR("Failed to allocate LED fd");
		return DRIVER_ERR_NOMEM;
	}

	/* Open driver instance */
	DRIVER_LOCK(&led_inst.base);
	led_inst.base.state = DRIVER_STATE_OPEN;
	led_inst.base.flags = flags;
	DRIVER_UNLOCK(&led_inst.base);

	LOG_INF("LED driver opened with fd=%d", fd);
	return fd;
}

/*
 * Close LED driver
 */
int led_close(driver_fd_t fd)
{
	driver_instance_t *inst;

	/* Validate file descriptor */
	inst = driver_fd_to_instance(fd, led_instances, 1);
	if (!inst) {
		LOG_ERR("Invalid LED fd=%d", fd);
		return DRIVER_ERR_BADF;
	}

	/* Turn off all LEDs before closing */
	DRIVER_LOCK(inst);
	for (int i = 0; i < LED_COUNT; i++) {
		led_inst.brightness[i] = 0;
		led_inst.colors[i].red = 0;
		led_inst.colors[i].green = 0;
		led_inst.colors[i].blue = 0;
	}
	led_update_spi();
	inst->state = DRIVER_STATE_CLOSED;
	inst->flags = 0;
	DRIVER_UNLOCK(inst);

	LOG_INF("LED driver closed (fd=%d, writes=%u, updates=%u, errors=%u)",
	        fd, led_inst.stats.write_count, led_inst.stats.update_count,
	        led_inst.stats.spi_error_count);

	return DRIVER_OK;
}

/*
 * Write LED control data
 */
ssize_t led_write(driver_fd_t fd, const void *buf, size_t count)
{
	driver_instance_t *inst;
	int err;

	/* Validate file descriptor */
	inst = driver_fd_to_instance(fd, led_instances, 1);
	if (!inst) {
		LOG_ERR("Invalid LED fd=%d", fd);
		return DRIVER_ERR_BADF;
	}

	/* Validate buffer */
	if (!buf) {
		LOG_ERR("Invalid buffer (buf=%p)", buf);
		return DRIVER_ERR_INVAL;
	}

	DRIVER_LOCK(inst);

	/* Check buffer size and handle accordingly */
	if (count == sizeof(led_control_t)) {
		/* Single LED control */
		const led_control_t *ctrl = (const led_control_t *)buf;

		/* Validate LED index */
		if (ctrl->led_index >= LED_COUNT) {
			LOG_ERR("Invalid LED index: %u (must be 0-%d)",
			        ctrl->led_index, LED_COUNT - 1);
			DRIVER_UNLOCK(inst);
			return DRIVER_ERR_INVAL;
		}

		/* Validate brightness */
		if (ctrl->brightness > 31) {
			LOG_WRN("Brightness %u clamped to 31", ctrl->brightness);
		}

		/* Update single LED state */
		led_inst.brightness[ctrl->led_index] = ctrl->brightness & 0x1F;
		led_inst.colors[ctrl->led_index] = ctrl->color;

	} else if (count == sizeof(led_array_control_t)) {
		/* All LEDs control */
		const led_array_control_t *ctrl = (const led_array_control_t *)buf;

		/* Validate brightness */
		if (ctrl->brightness > 31) {
			LOG_WRN("Brightness %u clamped to 31", ctrl->brightness);
		}

		/* Update all LED states */
		uint8_t brightness = ctrl->brightness & 0x1F;
		for (int i = 0; i < LED_COUNT; i++) {
			led_inst.brightness[i] = brightness;
			led_inst.colors[i] = ctrl->leds[i];
		}

	} else {
		LOG_ERR("Invalid buffer size: %zu (expected %zu or %zu)",
		        count, sizeof(led_control_t), sizeof(led_array_control_t));
		DRIVER_UNLOCK(inst);
		return DRIVER_ERR_INVAL;
	}

	/* Update LEDs via SPI */
	err = led_update_spi();
	if (err < 0) {
		DRIVER_UNLOCK(inst);
		return err;
	}

	led_inst.stats.write_count++;
	DRIVER_UNLOCK(inst);

	return count;
}

/*
 * Read LED state
 */
ssize_t led_read(driver_fd_t fd, void *buf, size_t count)
{
	driver_instance_t *inst;
	led_array_control_t *ctrl;

	/* Validate file descriptor */
	inst = driver_fd_to_instance(fd, led_instances, 1);
	if (!inst) {
		LOG_ERR("Invalid LED fd=%d", fd);
		return DRIVER_ERR_BADF;
	}

	/* Validate buffer and size */
	if (!buf || count < sizeof(led_array_control_t)) {
		LOG_ERR("Invalid buffer (buf=%p, count=%zu)", buf, count);
		return DRIVER_ERR_INVAL;
	}

	DRIVER_LOCK(inst);

	/* Return current state */
	ctrl = (led_array_control_t *)buf;

	/* Use brightness from first LED (they're all set to same value in array mode) */
	ctrl->brightness = led_inst.brightness[0];

	/* Copy all LED colors */
	for (int i = 0; i < LED_COUNT; i++) {
		ctrl->leds[i] = led_inst.colors[i];
	}

	DRIVER_UNLOCK(inst);

	return sizeof(led_array_control_t);
}

/*
 * LED driver control (ioctl)
 */
int led_ioctl(driver_fd_t fd, unsigned int cmd, void *arg)
{
	driver_instance_t *inst;
	int err = DRIVER_OK;

	/* Validate file descriptor */
	inst = driver_fd_to_instance(fd, led_instances, 1);
	if (!inst) {
		LOG_ERR("Invalid LED fd=%d", fd);
		return DRIVER_ERR_BADF;
	}

	DRIVER_LOCK(inst);

	switch (cmd) {
	case DRIVER_IOCTL_GET_STATUS:
		/* Return driver state */
		if (arg) {
			*(driver_state_t *)arg = inst->state;
		}
		break;

	case DRIVER_IOCTL_GET_INFO:
		/* Return driver info string */
		if (arg) {
			*(const char **)arg = "APA102 RGB LED (SPI)";
		}
		break;

	case LED_IOCTL_SET_BRIGHTNESS:
		/* Set global brightness for all LEDs */
		if (arg) {
			uint8_t brightness = *(uint8_t *)arg;
			if (brightness > 31) {
				LOG_WRN("Brightness %u clamped to 31", brightness);
				brightness = 31;
			}
			for (int i = 0; i < LED_COUNT; i++) {
				led_inst.brightness[i] = brightness;
			}
			err = led_update_spi();
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	case LED_IOCTL_SET_COLOR:
		/* Set color of specific LED */
		if (arg) {
			const led_control_t *ctrl = (const led_control_t *)arg;
			if (ctrl->led_index >= LED_COUNT) {
				LOG_ERR("Invalid LED index: %u", ctrl->led_index);
				err = DRIVER_ERR_INVAL;
			} else {
				led_inst.brightness[ctrl->led_index] = ctrl->brightness & 0x1F;
				led_inst.colors[ctrl->led_index] = ctrl->color;
				err = led_update_spi();
			}
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	case LED_IOCTL_SET_ALL_OFF:
		/* Turn all LEDs off */
		for (int i = 0; i < LED_COUNT; i++) {
			led_inst.brightness[i] = 0;
			led_inst.colors[i].red = 0;
			led_inst.colors[i].green = 0;
			led_inst.colors[i].blue = 0;
		}
		err = led_update_spi();
		break;

	case LED_IOCTL_GET_STATS:
		/* Get usage statistics */
		if (arg) {
			memcpy(arg, &led_inst.stats, sizeof(led_stats_t));
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	case LED_IOCTL_RESET_STATS:
		/* Reset statistics counters */
		memset(&led_inst.stats, 0, sizeof(led_stats_t));
		break;

	default:
		LOG_WRN("Unknown LED ioctl command: 0x%x", cmd);
		err = DRIVER_ERR_INVAL;
		break;
	}

	DRIVER_UNLOCK(inst);

	return err;
}
