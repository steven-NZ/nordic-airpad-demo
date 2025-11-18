/*
 * Button Handler Interface
 *
 * Handles button press detection and event generation
 */

#ifndef BTN_HANDLER_H_
#define BTN_HANDLER_H_

/**
 * Initialize button handler
 *
 * Sets up button interrupt handling using DK buttons library
 * Configures all 4 buttons with press detection
 *
 * @return 0 on success, negative error code on failure
 */
int btn_handler_init(void);

#endif /* BTN_HANDLER_H_ */
