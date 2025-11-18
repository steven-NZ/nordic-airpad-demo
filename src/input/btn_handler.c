/*
 * Button Handler Implementation
 *
 * Handles button press detection and event generation
 */

#include "btn_handler.h"
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include "../comm/esb_thread.h"

LOG_MODULE_REGISTER(btn_handler, LOG_LEVEL_INF);

/*
 * Button press handler callback
 *
 * Called by dk_buttons library when button state changes
 * Only triggers on button press events (not release)
 */
static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	/* Handle button presses - only send on press, not release */
	if (has_changed & button_state & DK_BTN1_MSK) {
		LOG_INF("Button 1 pressed - sending 'A'");
		esb_thread_send_char('A');
	}
	if (has_changed & button_state & DK_BTN2_MSK) {
		LOG_INF("Button 2 pressed - sending 'B'");
		esb_thread_send_char('B');
	}
	if (has_changed & button_state & DK_BTN3_MSK) {
		LOG_INF("Button 3 pressed - sending 'C'");
		esb_thread_send_char('C');
	}
	if (has_changed & button_state & DK_BTN4_MSK) {
		LOG_INF("Button 4 pressed - sending 'D'");
		esb_thread_send_char('D');
	}
}

int btn_handler_init(void)
{
	int err;

	/* Initialize DK buttons library with our callback */
	err = dk_buttons_init(button_handler);
	if (err) {
		LOG_ERR("Failed to initialize buttons, err %d", err);
		return err;
	}

	LOG_INF("Button handler initialized");
	return 0;
}
