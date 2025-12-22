/*
 * ESB Hardware Handler Implementation
 */

#include "esb_handler.h"
#include "../driver_framework.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/init.h>
#include <dk_buttons_and_leds.h>
#include <esb.h>
#include <string.h>

LOG_MODULE_REGISTER(esb_handler, LOG_LEVEL_INF);

/*
 * Private Driver State
 */
typedef struct {
	driver_instance_t base;         /* Base driver instance (must be first) */
	struct esb_payload rx_payload;  /* RX buffer */
	struct esb_payload tx_payload;  /* TX buffer */
	struct esb_payload ack_payload; /* ACK payload buffer */
	response_data_t response_data;  /* Response data structure */
	bool ack_payload_enabled;       /* Enable ACK payload transmission */
	esb_stats_t stats;              /* Statistics counters */
	int8_t tx_power;                /* Current TX power (dBm) */
} esb_instance_t;

/* Single ESB instance (static allocation) */
static esb_instance_t esb_inst;
static driver_instance_t *esb_instances[1] = { &esb_inst.base };

/*
 * ESB event handler (ISR context)
 *
 * Called by ESB library when events occur (TX complete, RX received, etc.)
 */
static void esb_driver_event_handler(struct esb_evt const *event)
{
	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS EVENT");
		esb_inst.stats.tx_success_count++;
		dk_set_led_off(DK_LED1);
		break;

	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED EVENT");
		esb_inst.stats.tx_failed_count++;
		dk_set_led_on(DK_LED1);
		break;

	case ESB_EVENT_RX_RECEIVED:
		LOG_DBG("RX RECEIVED EVENT");
		esb_inst.stats.rx_count++;
		break;

	default:
		LOG_WRN("Unknown ESB event: %d", event->evt_id);
		break;
	}
}

/*
 * Device initialization - called automatically at boot via SYS_INIT
 * Initializes the ESB radio subsystem
 */
static int esb_device_init(void)
{
	int err;
	struct esb_config config = ESB_DEFAULT_CONFIG;
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

	/* Initialize driver instance */
	DRIVER_INSTANCE_INIT(&esb_inst.base);
	memset(&esb_inst.stats, 0, sizeof(esb_stats_t));
	esb_inst.tx_power = 8; /* 8 dBm */

	/* Initialize ACK payload */
	memset(&esb_inst.response_data, 0, sizeof(response_data_t));
	esb_inst.ack_payload_enabled = false;
	esb_inst.ack_payload.length = sizeof(response_data_t);
	esb_inst.ack_payload.pipe = 0;  /* Pipe 0 for ACK */
	memset(esb_inst.ack_payload.data, 0, sizeof(response_data_t));

	/* Initialize TX payload with default values */
	esb_inst.tx_payload = (struct esb_payload)ESB_CREATE_PAYLOAD(0,
		0x01, 0x00, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08);

	/* Configure ESB for maximum range */
	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.mode = ESB_MODE_PTX;
	config.event_handler = esb_driver_event_handler;
	config.bitrate = ESB_BITRATE_1MBPS;
	config.tx_output_power = ESB_TX_POWER_8DBM;
	config.retransmit_delay = 600;
	config.retransmit_count = 3;
	config.tx_mode = ESB_TXMODE_AUTO;
	config.payload_length = 32;
	config.selective_auto_ack = true;

#if defined(CONFIG_SOC_SERIES_NRF52X) || defined(CONFIG_SOC_SERIES_NRF53X)
	config.use_fast_ramp_up = true;
#endif

	err = esb_init(&config);
	if (err) {
		LOG_ERR("ESB initialization failed, err %d", err);
		return err;
	}

	LOG_INF("ESB initialized");

	/* Set base addresses */
	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		LOG_ERR("Failed to set base address 0, err %d", err);
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err) {
		LOG_ERR("Failed to set base address 1, err %d", err);
		return err;
	}

	/* Set address prefixes */
	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err) {
		LOG_ERR("Failed to set address prefixes, err %d", err);
		return err;
	}

	LOG_INF("ESB device initialized");

	return 0;
}

/* Register device initialization at boot (POST_KERNEL level, priority 92) */
SYS_INIT(esb_device_init, POST_KERNEL, 92);

/*
 * Open ESB driver - Returns file descriptor
 */
driver_fd_t esb_open(uint32_t flags)
{
	driver_fd_t fd;

	/* Only one ESB instance supported */
	if (esb_inst.base.state == DRIVER_STATE_OPEN) {
		LOG_WRN("ESB driver already open");
		return DRIVER_ERR_BUSY;
	}

	/* Allocate file descriptor (always returns 0 for single instance) */
	fd = driver_fd_alloc(esb_instances, 1);
	if (fd == DRIVER_FD_INVALID) {
		LOG_ERR("Failed to allocate ESB fd");
		return DRIVER_ERR_NOMEM;
	}

	/* Open driver instance */
	DRIVER_LOCK(&esb_inst.base);
	esb_inst.base.state = DRIVER_STATE_OPEN;
	esb_inst.base.flags = flags;
	DRIVER_UNLOCK(&esb_inst.base);

	LOG_INF("ESB driver opened with fd=%d", fd);
	return fd;
}

/*
 * Close ESB driver
 */
int esb_close(driver_fd_t fd)
{
	driver_instance_t *inst;

	/* Validate file descriptor */
	inst = driver_fd_to_instance(fd, esb_instances, 1);
	if (!inst) {
		LOG_ERR("Invalid ESB fd=%d", fd);
		return DRIVER_ERR_BADF;
	}

	/* Close driver instance */
	DRIVER_LOCK(inst);
	inst->state = DRIVER_STATE_CLOSED;
	inst->flags = 0;
	DRIVER_UNLOCK(inst);

	LOG_INF("ESB driver closed (fd=%d, TX success=%u, TX failed=%u, RX=%u)",
		fd, esb_inst.stats.tx_success_count,
		esb_inst.stats.tx_failed_count,
		esb_inst.stats.rx_count);

	return DRIVER_OK;
}

/*
 * Read received data from ESB
 */
ssize_t esb_read(driver_fd_t fd, void *buf, size_t count)
{
	driver_instance_t *inst;
	int err;
	size_t copy_len;

	/* Validate file descriptor */
	inst = driver_fd_to_instance(fd, esb_instances, 1);
	if (!inst) {
		LOG_ERR("Invalid ESB fd=%d", fd);
		return DRIVER_ERR_BADF;
	}

	/* Validate buffer */
	if (!buf || count == 0) {
		LOG_ERR("Invalid buffer (buf=%p, count=%zu)", buf, count);
		return DRIVER_ERR_INVAL;
	}

	DRIVER_LOCK(inst);

	/* Check if RX data is available */
	err = esb_read_rx_payload(&esb_inst.rx_payload);
	if (err) {
		/* No data available or error */
		DRIVER_UNLOCK(inst);
		return (err == -ENODATA) ? DRIVER_ERR_AGAIN : DRIVER_ERR_IO;
	}

	/* Copy received data to buffer */
	if (esb_inst.rx_payload.length > 0) {
		copy_len = (esb_inst.rx_payload.length < count) ?
			   esb_inst.rx_payload.length : count;
		memcpy(buf, esb_inst.rx_payload.data, copy_len);
		LOG_DBG("Received packet, len: %zu", copy_len);
		DRIVER_UNLOCK(inst);
		return (ssize_t)copy_len;
	}

	DRIVER_UNLOCK(inst);
	return DRIVER_ERR_AGAIN;
}

/*
 * Update ACK payload with current response data
 * Called before each transmission to ensure latest data
 *
 * NOTE: ACK payloads are not supported in PTX mode in Nordic ESB.
 * This is a placeholder for future implementation if mode changes to PRX.
 */
static void esb_update_ack_payload(void)
{
	if (esb_inst.ack_payload_enabled) {
		/* Copy response data to ACK payload buffer */
		memcpy(esb_inst.ack_payload.data, &esb_inst.response_data,
		       sizeof(response_data_t));

		/* NOTE: esb_set_tx_ack_payload() is not available in PTX mode
		 * ACK payloads are a PRX (receiver) feature. In PTX mode,
		 * we can only receive ACK payloads, not send them.
		 * If bidirectional communication is needed, the receiver (PRX)
		 * should send regular packets back.
		 */

		LOG_DBG("Response data updated (PTX mode - ACK payload not sent): vib_enable=%d",
		        esb_inst.response_data.vibration_enable);
	}
}

/*
 * Write data to ESB for transmission
 */
ssize_t esb_write(driver_fd_t fd, const void *buf, size_t count)
{
	driver_instance_t *inst;
	int err;

	/* Validate file descriptor */
	inst = driver_fd_to_instance(fd, esb_instances, 1);
	if (!inst) {
		LOG_ERR("Invalid ESB fd=%d", fd);
		return DRIVER_ERR_BADF;
	}

	/* Validate buffer */
	if (!buf || count == 0 || count > CONFIG_ESB_MAX_PAYLOAD_LENGTH) {
		LOG_ERR("Invalid buffer (buf=%p, count=%zu)", buf, count);
		return DRIVER_ERR_INVAL;
	}

	DRIVER_LOCK(inst);

	/* Update TX payload */
	esb_inst.tx_payload.length = (uint8_t)count;
	memcpy(esb_inst.tx_payload.data, buf, count);

	/* For sensor_data_t, add debug logging */
	if (count == sizeof(sensor_data_t)) {
		const sensor_data_t *data = (const sensor_data_t *)buf;
		LOG_DBG("Sending sensor data: btn=0x%02X", data->btn_state);
	}

	/* Update ACK payload before transmitting */
	esb_update_ack_payload();

	/* Transmit packet */
	err = esb_write_payload(&esb_inst.tx_payload);
	if (err) {
		LOG_ERR("Failed to write payload, err %d", err);
		DRIVER_UNLOCK(inst);
		return DRIVER_ERR_IO;
	}

	DRIVER_UNLOCK(inst);
	return (ssize_t)count;
}

/*
 * ESB driver control (ioctl)
 */
int esb_ioctl(driver_fd_t fd, unsigned int cmd, void *arg)
{
	driver_instance_t *inst;
	int err = DRIVER_OK;

	/* Validate file descriptor */
	inst = driver_fd_to_instance(fd, esb_instances, 1);
	if (!inst) {
		LOG_ERR("Invalid ESB fd=%d", fd);
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
			*(const char **)arg = "ESB Radio";
		}
		break;

	case ESB_IOCTL_GET_TX_POWER:
		/* Return current TX power */
		if (arg) {
			*(int8_t *)arg = esb_inst.tx_power;
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	case ESB_IOCTL_GET_TX_SUCCESS:
		/* Return TX success count */
		if (arg) {
			*(uint32_t *)arg = esb_inst.stats.tx_success_count;
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	case ESB_IOCTL_GET_TX_FAILED:
		/* Return TX failed count */
		if (arg) {
			*(uint32_t *)arg = esb_inst.stats.tx_failed_count;
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	case ESB_IOCTL_GET_RX_COUNT:
		/* Return RX packet count */
		if (arg) {
			*(uint32_t *)arg = esb_inst.stats.rx_count;
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	case ESB_IOCTL_RESET_STATS:
		/* Reset all statistics counters */
		memset(&esb_inst.stats, 0, sizeof(esb_stats_t));
		LOG_INF("ESB statistics reset");
		break;

	case ESB_IOCTL_SET_ACK_PAYLOAD:
		/* Set ACK payload data */
		if (arg) {
			memcpy(&esb_inst.response_data, arg, sizeof(response_data_t));
			LOG_DBG("ACK payload updated: vib_enable=%d",
			        esb_inst.response_data.vibration_enable);
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	case ESB_IOCTL_ENABLE_ACK_PL:
		/* Enable/disable ACK payload */
		if (arg) {
			esb_inst.ack_payload_enabled = *(bool *)arg;
			LOG_INF("ACK payload %s",
			        esb_inst.ack_payload_enabled ? "enabled" : "disabled");
		} else {
			err = DRIVER_ERR_INVAL;
		}
		break;

	case ESB_IOCTL_SET_TX_POWER:
	case ESB_IOCTL_SET_BASE_ADDR_0:
	case ESB_IOCTL_SET_BASE_ADDR_1:
	case ESB_IOCTL_SET_ADDR_PREFIX:
		/* Runtime configuration changes not implemented */
		LOG_WRN("ESB runtime config not supported (cmd=0x%x)", cmd);
		err = DRIVER_ERR_NOTSUP;
		break;

	default:
		LOG_WRN("Unknown ESB ioctl command: 0x%x", cmd);
		err = DRIVER_ERR_INVAL;
		break;
	}

	DRIVER_UNLOCK(inst);

	return err;
}
