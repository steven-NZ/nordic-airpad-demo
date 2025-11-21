/*
 * ESB Hardware Handler
 *
 * Provides hardware interface for Enhanced ShockBurst protocol
 */

#include "esb_handler.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include <esb.h>

LOG_MODULE_REGISTER(esb_handler, LOG_LEVEL_INF);

/* ESB payload buffers */
static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
	0x01, 0x00, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08);

/* ESB event handler */
static void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS EVENT");
		dk_set_led_off(DK_LED1);
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED EVENT");
		dk_set_led_on(DK_LED1);
		break;
	case ESB_EVENT_RX_RECEIVED:
		LOG_DBG("RX RECEIVED EVENT");
		break;
	default:
		LOG_WRN("Unknown ESB event: %d", event->evt_id);
		break;
	}
}

int esb_handler_init(void)
{
	int err;
	struct esb_config config = ESB_DEFAULT_CONFIG;
	uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

	/* Configure ESB for maximum range */
	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.mode = ESB_MODE_PTX;
	config.event_handler = event_handler;
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

	LOG_INF("ESB configuration complete");

	return 0;
}

int esb_handler_send(const sensor_data_t *data)
{
	int err;

	if (!data) {
		return -EINVAL;
	}

	/* Update TX payload */
	tx_payload.length = sizeof(sensor_data_t);
	memcpy(tx_payload.data, data, sizeof(sensor_data_t));

	LOG_DBG("Sending sensor data: btn=0x%02X, seq=%u",
		data->btn_state, data->sequence_num);

	/* Transmit packet */
	err = esb_write_payload(&tx_payload);
	if (err) {
		LOG_ERR("Failed to write payload, err %d", err);
		return err;
	}

	return 0;
}

int esb_handler_receive(uint8_t *buf, size_t *len)
{
	int err;

	if (!buf || !len) {
		return -EINVAL;
	}

	/* Check if RX data is available */
	err = esb_read_rx_payload(&rx_payload);
	if (err) {
		/* No data available or error */
		*len = 0;
		return err;
	}

	/* Copy received data to buffer */
	if (rx_payload.length > 0) {
		size_t copy_len = rx_payload.length < *len ? rx_payload.length : *len;
		memcpy(buf, rx_payload.data, copy_len);
		*len = copy_len;
		LOG_DBG("Received packet, len: %zu", copy_len);
		return 0;
	}

	*len = 0;
	return -ENODATA;
}
