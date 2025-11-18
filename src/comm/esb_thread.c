/*
 * ESB Thread Implementation
 *
 * Handles Enhanced ShockBurst protocol communication
 */

#include "esb_thread.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include <esb.h>
#include <string.h>

LOG_MODULE_REGISTER(esb_thread, LOG_LEVEL_INF);

/* ESB payload buffers */
static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
	0x01, 0x00, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08);

/* Packet sequence number */
static uint32_t packet_sequence = 0;

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
		while (esb_read_rx_payload(&rx_payload) == 0) {
			if (rx_payload.length > 0) {
				LOG_INF("Packet received, len: %d", rx_payload.length);
				LOG_HEXDUMP_INF(rx_payload.data, rx_payload.length, "RX:");
			}
		}
		break;
	default:
		LOG_WRN("Unknown ESB event: %d", event->evt_id);
		break;
	}
}

int esb_thread_init(void)
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

void esb_thread_send_button_event(uint8_t button_num, bool pressed)
{
	int err;
	button_packet_t packet;

	/* Build button event packet */
	packet.sequence_num = packet_sequence++;
	packet.button_num = button_num;
	packet.pressed = pressed ? 1 : 0;
	packet.timestamp_ms = k_uptime_get_32();

	/* Update TX payload */
	tx_payload.length = sizeof(button_packet_t);
	memcpy(tx_payload.data, &packet, sizeof(button_packet_t));

	LOG_INF("Sending button event: btn=%d, pressed=%d, seq=%u",
		button_num, pressed, packet.sequence_num);

	/* Transmit packet */
	err = esb_write_payload(&tx_payload);
	if (err) {
		LOG_ERR("Failed to write payload, err %d", err);
	}
}

void esb_thread_send_char(char character)
{
	int err;

	/* Update TX payload with single character */
	tx_payload.length = 1;
	tx_payload.data[0] = character;

	LOG_INF("Sending character: '%c'", character);

	/* Transmit packet */
	err = esb_write_payload(&tx_payload);
	if (err) {
		LOG_ERR("Failed to write payload, err %d", err);
	}
}

void esb_thread_process(void)
{
	/* In single-threaded mode, this is mostly a placeholder */
	/* ESB events are handled via interrupt callback (event_handler) */
	/* Future: When using actual threads, this would poll message queues */
	k_sleep(K_MSEC(1));
}
