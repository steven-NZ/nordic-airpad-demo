/*
 * Button Handler Implementation
 *
 * Handles button press detection and event generation
 */

#include "btn_handler.h"
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>

LOG_MODULE_REGISTER(btn_handler, LOG_LEVEL_INF);

/* Button state tracking - 3 bits for buttons 1-3 (1=pressed, 0=released) */
static uint8_t button_state_bits = 0;

/*
 * Button state change handler callback
 *
 * Called by dk_buttons library when button state changes
 * Tracks both press and release events for buttons 1-3
 * Button state is sent periodically via ESB (not on interrupt)
 */
static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	/* Button 1 - track press and release */
	if (has_changed & DK_BTN1_MSK) {
		if (button_state & DK_BTN1_MSK) {
			button_state_bits |= (1 << 0);  /* Set bit 0 */
			LOG_INF("Button 1 pressed");
		} else {
			button_state_bits &= ~(1 << 0); /* Clear bit 0 */
			LOG_INF("Button 1 released");
		}
	}

	/* Button 2 - track press and release */
	if (has_changed & DK_BTN2_MSK) {
		if (button_state & DK_BTN2_MSK) {
			button_state_bits |= (1 << 1);  /* Set bit 1 */
			LOG_INF("Button 2 pressed");
		} else {
			button_state_bits &= ~(1 << 1); /* Clear bit 1 */
			LOG_INF("Button 2 released");
		}
	}

	/* Button 3 - track press and release */
	if (has_changed & DK_BTN3_MSK) {
		if (button_state & DK_BTN3_MSK) {
			button_state_bits |= (1 << 2);  /* Set bit 2 */
			LOG_INF("Button 3 pressed");
		} else {
			button_state_bits &= ~(1 << 2); /* Clear bit 2 */
			LOG_INF("Button 3 released");
		}
	}

	/* Button 4 is ignored */
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

uint8_t btn_handler_get_state(void)
{
	/* Return only the 3 button bits (mask off any higher bits) */
	return button_state_bits & 0x07;
}
