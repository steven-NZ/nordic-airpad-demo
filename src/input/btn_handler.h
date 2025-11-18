/*
 * Button Handler Interface
 *
 * Handles button press detection and event generation
 */

#ifndef BTN_HANDLER_H_
#define BTN_HANDLER_H_

#include <stdint.h>

/**
 * Initialize button handler
 *
 * Sets up button interrupt handling using DK buttons library
 * Configures buttons 1-3 with press/release detection
 *
 * @return 0 on success, negative error code on failure
 */
int btn_handler_init(void);

/**
 * Get current button state
 *
 * Returns the current state of buttons 1-3 as a 3-bit value
 * Bit 0: Button 1 (1=pressed, 0=released)
 * Bit 1: Button 2 (1=pressed, 0=released)
 * Bit 2: Button 3 (1=pressed, 0=released)
 *
 * @return 3-bit button state value (0x00-0x07)
 */
uint8_t btn_handler_get_state(void);

#endif /* BTN_HANDLER_H_ */
