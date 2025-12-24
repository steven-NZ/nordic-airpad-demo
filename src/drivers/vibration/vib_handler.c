/*
 * Vibration Handler Implementation
 */

#include "vib_handler.h"
#include "../driver_framework.h"
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <string.h>

LOG_MODULE_REGISTER(vib_handler, LOG_LEVEL_INF);

/* PWM Configuration */
#define VIB_PWM_PERIOD_US 1000  /* 1 kHz (1000 microseconds period) */
#define VIB_PWM_CHANNEL 0        /* PWM channel 0 */

/*
 * Private Driver State
 */
typedef struct {
	driver_instance_t base;      /* Base driver instance (must be first) */
	const struct device *pwm_dev; /* PWM device pointer */
	uint8_t enable;              /* Current enable state (0 or 1) */
	uint8_t intensity;           /* Current intensity (0-255) */
	vib_stats_t stats;           /* Usage statistics */
	int64_t last_enable_time;    /* Timestamp of last enable (for stats) */
} vib_instance_t;

/* Single vibration handler instance (static allocation) */
static vib_instance_t vib_inst;
static driver_instance_t *vib_instances[1] = { &vib_inst.base };

/*
 * Update PWM output based on current enable and intensity state
 * Must be called with driver lock held
 */
static int vib_update_pwm(void)
{
	uint32_t pulse_width_us;
	int err;

	if (!vib_inst.pwm_dev) {
		LOG_ERR("PWM device not initialized");
		return DRIVER_ERR_NODEV;
	}

	/* Calculate pulse width based on enable and intensity */
	if (vib_inst.enable && vib_inst.intensity > 0) {
		/* Map intensity (0-255) to pulse width (0-VIB_PWM_PERIOD_US) */
		pulse_width_us = ((uint32_t)vib_inst.intensity * VIB_PWM_PERIOD_US) / 255;
	} else {
		/* Motor off */
		pulse_width_us = 0;
	}

	/* Set PWM duty cycle */
	err = pwm_set(vib_inst.pwm_dev, VIB_PWM_CHANNEL,
	              PWM_USEC(VIB_PWM_PERIOD_US),
	              PWM_USEC(pulse_width_us), 0);

	if (err) {
		LOG_ERR("Failed to set PWM: %d", err);
		return DRIVER_ERR_IO;
	}

	return DRIVER_OK;
}

/*
 * Device initialization - called automatically at boot via SYS_INIT
 * Initializes the PWM peripheral
 */
static int vib_device_init(void)
{
	int err;

	/* Initialize driver instance */
	DRIVER_INSTANCE_INIT(&vib_inst.base);
	vib_inst.enable = 0;
	vib_inst.intensity = 0;
	vib_inst.last_enable_time = 0;
	memset(&vib_inst.stats, 0, sizeof(vib_stats_t));

	/* Get PWM device */
	vib_inst.pwm_dev = DEVICE_DT_GET(DT_NODELABEL(pwm0));
	if (!vib_inst.pwm_dev) {
		LOG_ERR("Failed to get PWM device");
		return -ENODEV;
	}

	/* Check if PWM device is ready */
	if (!device_is_ready(vib_inst.pwm_dev)) {
		LOG_ERR("PWM device not ready");
		vib_inst.pwm_dev = NULL;
		return -ENODEV;
	}

	/* Initialize PWM to OFF state (0% duty cycle) */
	err = pwm_set(vib_inst.pwm_dev, VIB_PWM_CHANNEL,
	              PWM_USEC(VIB_PWM_PERIOD_US),
	              PWM_USEC(0), 0);
	if (err) {
		LOG_ERR("Failed to initialize PWM: %d", err);
		return err;
	}

	LOG_INF("Vibration device initialized (PWM 1kHz, P1.10)");
	return 0;
}

/* Register device initialization at boot (POST_KERNEL level, priority 91) */
SYS_INIT(vib_device_init, POST_KERNEL, 91);

/*
 * Open vibration driver - Returns file descriptor
 */
driver_fd_t vib_open(uint32_t flags)
{
	driver_fd_t fd;

	/* Only one vibration instance supported */
	if (vib_inst.base.state == DRIVER_STATE_OPEN) {
		LOG_WRN("Vibration driver already open");
		return DRIVER_ERR_BUSY;
	}

	/* Check if PWM device was initialized successfully */
	if (!vib_inst.pwm_dev) {
		LOG_ERR("PWM device not available");
		return DRIVER_ERR_NODEV;
	}

	/* Allocate file descriptor (always returns 0 for single instance) */
	fd = driver_fd_alloc(vib_instances, 1);
	if (fd == DRIVER_FD_INVALID) {
		LOG_ERR("Failed to allocate vibration fd");
		return DRIVER_ERR_NOMEM;
	}

	/* Open driver instance */
	DRIVER_LOCK(&vib_inst.base);
	vib_inst.base.state = DRIVER_STATE_OPEN;
	vib_inst.base.flags = flags;
	DRIVER_UNLOCK(&vib_inst.base);

	LOG_INF("Vibration driver opened with fd=%d", fd);
	return fd;
}

/*
 * Close vibration driver
 */
int vib_close(driver_fd_t fd)
{
	driver_instance_t *inst;

	/* Validate file descriptor */
	inst = driver_fd_to_instance(fd, vib_instances, 1);
	if (!inst) {
		LOG_ERR("Invalid vibration fd=%d", fd);
		return DRIVER_ERR_BADF;
	}

	/* Disable motor before closing */
	DRIVER_LOCK(inst);
	vib_inst.enable = 0;
	vib_update_pwm();
	inst->state = DRIVER_STATE_CLOSED;
	inst->flags = 0;
	DRIVER_UNLOCK(inst);

	LOG_INF("Vibration driver closed (fd=%d, writes=%u, enables=%u)",
	        fd, vib_inst.stats.write_count, vib_inst.stats.enable_count);

	return DRIVER_OK;
}

/*
 * Write vibration control data
 */
ssize_t vib_write(driver_fd_t fd, const void *buf, size_t count)
{
	driver_instance_t *inst;
	const vib_control_t *ctrl;
	int err;
	int64_t current_time;
	uint8_t prev_enable;

	/* Validate file descriptor */
	inst = driver_fd_to_instance(fd, vib_instances, 1);
	if (!inst) {
		LOG_ERR("Invalid vibration fd=%d", fd);
		return DRIVER_ERR_BADF;
	}

	/* Validate buffer and size */
	if (!buf || count < sizeof(vib_control_t)) {
		LOG_ERR("Invalid buffer (buf=%p, count=%zu)", buf, count);
		return DRIVER_ERR_INVAL;
	}

	ctrl = (const vib_control_t *)buf;

	/* Validate enable field (must be 0 or 1) */
	if (ctrl->enable > 1) {
		LOG_ERR("Invalid enable value: %u (must be 0 or 1)", ctrl->enable);
		return DRIVER_ERR_INVAL;
	}

	DRIVER_LOCK(inst);

	prev_enable = vib_inst.enable;
	current_time = k_uptime_get();

	/* Update statistics for on-time tracking */
	if (prev_enable && vib_inst.last_enable_time > 0) {
		int64_t elapsed_ms = current_time - vib_inst.last_enable_time;
		if (elapsed_ms > 0) {
			vib_inst.stats.total_on_time_ms += (uint32_t)elapsed_ms;
		}
	}

	/* Update state */
	vib_inst.enable = ctrl->enable;
	vib_inst.intensity = ctrl->intensity;

	/* Track enable count (when transitioning from off to on) */
	if (ctrl->enable && !prev_enable) {
		vib_inst.stats.enable_count++;
		vib_inst.last_enable_time = current_time;
		LOG_INF("Vibration ON (intensity=%u)", ctrl->intensity);
	} else if (!ctrl->enable && prev_enable) {
		LOG_INF("Vibration OFF");
	} else if (ctrl->enable) {
		vib_inst.last_enable_time = current_time;
	}

	/* Update PWM output */
	err = vib_update_pwm();
	if (err < 0) {
		DRIVER_UNLOCK(inst);
		return err;
	}

	vib_inst.stats.write_count++;
	DRIVER_UNLOCK(inst);

	return sizeof(vib_control_t);
}

/*
 * Read vibration state
 */
ssize_t vib_read(driver_fd_t fd, void *buf, size_t count)
{
	driver_instance_t *inst;
	vib_control_t *ctrl;

	/* Validate file descriptor */
	inst = driver_fd_to_instance(fd, vib_instances, 1);
	if (!inst) {
		LOG_ERR("Invalid vibration fd=%d", fd);
		return DRIVER_ERR_BADF;
	}

	/* Validate buffer and size */
	if (!buf || count < sizeof(vib_control_t)) {
		LOG_ERR("Invalid buffer (buf=%p, count=%zu)", buf, count);
		return DRIVER_ERR_INVAL;
	}

	DRIVER_LOCK(inst);

	/* Return current state */
	ctrl = (vib_control_t *)buf;
	ctrl->enable = vib_inst.enable;
	ctrl->intensity = vib_inst.intensity;

	DRIVER_UNLOCK(inst);

	return sizeof(vib_control_t);
}

/*
 * Vibration driver control (ioctl)
 */
int vib_ioctl(driver_fd_t fd, unsigned int cmd, void *arg)
{
	driver_instance_t *inst;
	int err = DRIVER_OK;
	int64_t current_time;

	/* Validate file descriptor */
	inst = driver_fd_to_instance(fd, vib_instances, 1);
	if (!inst) {
		LOG_ERR("Invalid vibration fd=%d", fd);
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
			*(const char **)arg = "Vibration Motor (PWM)";
		}
		break;

	case VIB_IOCTL_SET_ENABLE:
		/* Enable/disable motor */
		if (arg) {
			uint8_t enable = *(uint8_t *)arg;
			if (enable > 1) {
				err = DRIVER_ERR_INVAL;
			} else {
				uint8_t prev_enable = vib_inst.enable;
				vib_inst.enable = enable;

				/* Update statistics */
				if (enable && !prev_enable) {
					vib_inst.stats.enable_count++;
					vib_inst.last_enable_time = k_uptime_get();
				}

				err = vib_update_pwm();
			}
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	case VIB_IOCTL_GET_ENABLE:
		/* Get enable state */
		if (arg) {
			*(uint8_t *)arg = vib_inst.enable;
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	case VIB_IOCTL_SET_INTENSITY:
		/* Set intensity (0-255) */
		if (arg) {
			vib_inst.intensity = *(uint8_t *)arg;
			err = vib_update_pwm();
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	case VIB_IOCTL_GET_INTENSITY:
		/* Get intensity */
		if (arg) {
			*(uint8_t *)arg = vib_inst.intensity;
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	case VIB_IOCTL_GET_STATS:
		/* Get usage statistics */
		if (arg) {
			/* Update on-time if currently enabled */
			if (vib_inst.enable && vib_inst.last_enable_time > 0) {
				current_time = k_uptime_get();
				int64_t elapsed_ms = current_time - vib_inst.last_enable_time;
				if (elapsed_ms > 0) {
					vib_stats_t *stats = (vib_stats_t *)arg;
					stats->enable_count = vib_inst.stats.enable_count;
					stats->write_count = vib_inst.stats.write_count;
					stats->total_on_time_ms = vib_inst.stats.total_on_time_ms +
					                          (uint32_t)elapsed_ms;
					DRIVER_UNLOCK(inst);
					return DRIVER_OK;
				}
			}
			memcpy(arg, &vib_inst.stats, sizeof(vib_stats_t));
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	default:
		LOG_WRN("Unknown vibration ioctl command: 0x%x", cmd);
		err = DRIVER_ERR_INVAL;
		break;
	}

	DRIVER_UNLOCK(inst);

	return err;
}
