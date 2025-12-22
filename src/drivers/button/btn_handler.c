/*
 * Button Handler Implementation
 */

#include "btn_handler.h"
#include "../driver_framework.h"
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <dk_buttons_and_leds.h>
#include <string.h>

LOG_MODULE_REGISTER(btn_handler, LOG_LEVEL_INF);

/*
 * Private Driver State
 */
typedef struct {
	driver_instance_t base;         /* Base driver instance (must be first) */
	uint8_t button_state_bits;      /* Current button state (3 bits) */
	btn_press_counts_t press_counts; /* Press event counters */
	uint32_t read_count;            /* Statistics: read operations */
} btn_instance_t;

/* Single button handler instance (static allocation) */
static btn_instance_t btn_inst;
static driver_instance_t *btn_instances[1] = { &btn_inst.base };

/*
 * Button state change handler callback (ISR context)
 *
 * Called by dk_buttons library when button state changes.
 * Updates internal state atomically.
 */
static void button_interrupt_handler(uint32_t button_state, uint32_t has_changed)
{
	/* Button 1 - track press and release */
	if (has_changed & DK_BTN1_MSK) {
		if (button_state & DK_BTN1_MSK) {
			btn_inst.button_state_bits |= (1 << 0);  /* Set bit 0 */
			btn_inst.press_counts.button1_count++;
			LOG_INF("Button 1 pressed");
		} else {
			btn_inst.button_state_bits &= ~(1 << 0); /* Clear bit 0 */
			LOG_INF("Button 1 released");
		}
	}

	/* Button 2 - track press and release */
	if (has_changed & DK_BTN2_MSK) {
		if (button_state & DK_BTN2_MSK) {
			btn_inst.button_state_bits |= (1 << 1);  /* Set bit 1 */
			btn_inst.press_counts.button2_count++;
			LOG_INF("Button 2 pressed");
		} else {
			btn_inst.button_state_bits &= ~(1 << 1); /* Clear bit 1 */
			LOG_INF("Button 2 released");
		}
	}

	/* Button 3 - track press and release */
	if (has_changed & DK_BTN3_MSK) {
		if (button_state & DK_BTN3_MSK) {
			btn_inst.button_state_bits |= (1 << 2);  /* Set bit 2 */
			btn_inst.press_counts.button3_count++;
			LOG_INF("Button 3 pressed");
		} else {
			btn_inst.button_state_bits &= ~(1 << 2); /* Clear bit 2 */
			LOG_INF("Button 3 released");
		}
	}

	/* Button 4 is ignored */
}

/*
 * Device initialization - called automatically at boot via SYS_INIT
 * Initializes the DK buttons library
 */
static int btn_device_init(void)
{
	int err;

	/* Initialize driver instance */
	DRIVER_INSTANCE_INIT(&btn_inst.base);
	btn_inst.button_state_bits = 0;
	btn_inst.read_count = 0;
	memset(&btn_inst.press_counts, 0, sizeof(btn_press_counts_t));

	/* Initialize DK buttons library with our ISR callback */
	err = dk_buttons_init(button_interrupt_handler);
	if (err) {
		LOG_ERR("Failed to initialize buttons, err %d", err);
		return err;
	}

	LOG_INF("Button device initialized");
	return 0;
}

/* Register device initialization at boot (POST_KERNEL level, priority 91) */
SYS_INIT(btn_device_init, POST_KERNEL, 91);

/*
 * Open button driver - Returns file descriptor
 */
driver_fd_t btn_open(uint32_t flags)
{
	driver_fd_t fd;

	/* Only one button instance supported */
	if (btn_inst.base.state == DRIVER_STATE_OPEN) {
		LOG_WRN("Button driver already open");
		return DRIVER_ERR_BUSY;
	}

	/* Allocate file descriptor (always returns 0 for single instance) */
	fd = driver_fd_alloc(btn_instances, 1);
	if (fd == DRIVER_FD_INVALID) {
		LOG_ERR("Failed to allocate button fd");
		return DRIVER_ERR_NOMEM;
	}

	/* Open driver instance */
	DRIVER_LOCK(&btn_inst.base);
	btn_inst.base.state = DRIVER_STATE_OPEN;
	btn_inst.base.flags = flags;
	DRIVER_UNLOCK(&btn_inst.base);

	LOG_INF("Button driver opened with fd=%d", fd);
	return fd;
}

/*
 * Close button driver
 */
int btn_close(driver_fd_t fd)
{
	driver_instance_t *inst;

	/* Validate file descriptor */
	inst = driver_fd_to_instance(fd, btn_instances, 1);
	if (!inst) {
		LOG_ERR("Invalid button fd=%d", fd);
		return DRIVER_ERR_BADF;
	}

	/* Close driver instance */
	DRIVER_LOCK(inst);
	inst->state = DRIVER_STATE_CLOSED;
	inst->flags = 0;
	DRIVER_UNLOCK(inst);

	LOG_INF("Button driver closed (fd=%d, reads=%u)", fd, btn_inst.read_count);

	return DRIVER_OK;
}

/*
 * Read button state
 */
ssize_t btn_read(driver_fd_t fd, void *buf, size_t count)
{
	driver_instance_t *inst;
	uint8_t state_bits;
	btn_state_t *state;

	/* Validate file descriptor */
	inst = driver_fd_to_instance(fd, btn_instances, 1);
	if (!inst) {
		LOG_ERR("Invalid button fd=%d", fd);
		return DRIVER_ERR_BADF;
	}

	/* Validate buffer */
	if (!buf || count < 1) {
		LOG_ERR("Invalid buffer (buf=%p, count=%zu)", buf, count);
		return DRIVER_ERR_INVAL;
	}

	DRIVER_LOCK(inst);

	/* Read button state atomically (ISR updates this) */
	state_bits = btn_inst.button_state_bits & 0x07;

	/* Return structured state if buffer is large enough */
	if (count >= sizeof(btn_state_t)) {
		state = (btn_state_t *)buf;
		state->button1 = (state_bits >> 0) & 0x01;
		state->button2 = (state_bits >> 1) & 0x01;
		state->button3 = (state_bits >> 2) & 0x01;
		state->reserved = 0;
		btn_inst.read_count++;
		DRIVER_UNLOCK(inst);
		return sizeof(btn_state_t);
	} else {
		/* Return raw bits if buffer is only 1 byte */
		*(uint8_t *)buf = state_bits;
		btn_inst.read_count++;
		DRIVER_UNLOCK(inst);
		return 1;
	}
}

/*
 * Button driver control (ioctl)
 */
int btn_ioctl(driver_fd_t fd, unsigned int cmd, void *arg)
{
	driver_instance_t *inst;
	int err = DRIVER_OK;

	/* Validate file descriptor */
	inst = driver_fd_to_instance(fd, btn_instances, 1);
	if (!inst) {
		LOG_ERR("Invalid button fd=%d", fd);
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
			*(const char **)arg = "DK Buttons (1-3)";
		}
		break;

	case BTN_IOCTL_GET_RAW_STATE:
		/* Return raw button state bits */
		if (arg) {
			*(uint8_t *)arg = btn_inst.button_state_bits & 0x07;
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	case BTN_IOCTL_GET_PRESS_COUNT:
		/* Return press counters for all buttons */
		if (arg) {
			memcpy(arg, &btn_inst.press_counts, sizeof(btn_press_counts_t));
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	case BTN_IOCTL_RESET_COUNTERS:
		/* Reset all press counters */
		memset(&btn_inst.press_counts, 0, sizeof(btn_press_counts_t));
		LOG_INF("Button press counters reset");
		break;

	default:
		LOG_WRN("Unknown button ioctl command: 0x%x", cmd);
		err = DRIVER_ERR_INVAL;
		break;
	}

	DRIVER_UNLOCK(inst);

	return err;
}
