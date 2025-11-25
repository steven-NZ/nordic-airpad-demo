/*
 * Button Handler Interface
 */

#ifndef BTN_HANDLER_H_
#define BTN_HANDLER_H_

#include <stdint.h>
#include "../drivers/driver_framework.h"

/* Button state structure */
typedef struct {
	uint8_t button1 : 1;  /* Button 1 state (1=pressed, 0=released) */
	uint8_t button2 : 1;  /* Button 2 state */
	uint8_t button3 : 1;  /* Button 3 state */
	uint8_t reserved : 5; /* Reserved for future buttons */
} __attribute__((packed)) btn_state_t;

/* Button-specific ioctl commands */
#define BTN_IOCTL_GET_RAW_STATE     0x2001  /* Get raw button bits (uint8_t) */
#define BTN_IOCTL_GET_PRESS_COUNT   0x2002  /* Get press count for button */
#define BTN_IOCTL_RESET_COUNTERS    0x2003  /* Reset all button counters */

/* Button press counter structure (for ioctl) */
typedef struct {
	uint32_t button1_count;
	uint32_t button2_count;
	uint32_t button3_count;
} btn_press_counts_t;

/**
 * @brief Open the button driver
 *
 * Initializes button GPIO interrupts and returns a file descriptor
 * for accessing the driver. Only one instance is supported.
 *
 * @param flags Open flags (reserved for future use, pass 0)
 * @return File descriptor on success, DRIVER_FD_INVALID on failure
 */
driver_fd_t btn_open(uint32_t flags);

/**
 * @brief Close the button driver
 *
 * Releases resources associated with the button driver instance.
 *
 * @param fd File descriptor returned by btn_open()
 * @return 0 on success, negative errno code on failure
 */
int btn_close(driver_fd_t fd);

/**
 * @brief Read button state
 *
 * Reads the current button state. The buffer must be at least
 * sizeof(btn_state_t) bytes or 1 byte for raw button bits.
 *
 * @param fd File descriptor returned by btn_open()
 * @param buf Pointer to buffer to store button state
 * @param count Size of buffer
 * @return Number of bytes read on success, negative errno code on failure
 */
ssize_t btn_read(driver_fd_t fd, void *buf, size_t count);

/**
 * @brief Control the button driver
 *
 * Performs control operations on the button driver such as reading
 * press counters, resetting counters, etc.
 *
 * @param fd File descriptor returned by btn_open()
 * @param cmd ioctl command (BTN_IOCTL_*)
 * @param arg Command-specific argument (depends on cmd)
 * @return 0 on success, negative errno code on failure
 */
int btn_ioctl(driver_fd_t fd, unsigned int cmd, void *arg);

#endif /* BTN_HANDLER_H_ */
