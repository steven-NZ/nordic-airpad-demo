/*
 * Vibration Handler Interface
 */

#ifndef VIB_HANDLER_H_
#define VIB_HANDLER_H_

#include <stdint.h>
#include <stdbool.h>
#include "../driver_framework.h"

/* Vibration control structure */
typedef struct {
	uint8_t enable;        /* 0=off, 1=on */
	uint8_t intensity;     /* 0-255 (PWM duty cycle) */
} __attribute__((packed)) vib_control_t;

/* Vibration-specific ioctl commands (range: 0x5000-0x5FFF) */
#define VIB_IOCTL_SET_ENABLE        0x5001  /* Enable/disable motor (uint8_t) */
#define VIB_IOCTL_GET_ENABLE        0x5002  /* Get enable state (uint8_t*) */
#define VIB_IOCTL_SET_INTENSITY     0x5003  /* Set intensity 0-255 (uint8_t) */
#define VIB_IOCTL_GET_INTENSITY     0x5004  /* Get intensity (uint8_t*) */
#define VIB_IOCTL_GET_STATS         0x5005  /* Get usage statistics (vib_stats_t*) */

/* Vibration statistics */
typedef struct {
	uint32_t enable_count;     /* Number of times motor was enabled */
	uint32_t total_on_time_ms; /* Total time motor was on (milliseconds) */
	uint32_t write_count;      /* Number of write operations */
} vib_stats_t;

/**
 * @brief Open the vibration driver
 *
 * Initializes PWM peripheral and returns a file descriptor.
 * Only one instance is supported.
 *
 * @param flags Open flags (reserved for future use, pass 0)
 * @return File descriptor on success, DRIVER_FD_INVALID on failure
 */
driver_fd_t vib_open(uint32_t flags);

/**
 * @brief Close the vibration driver
 *
 * Disables motor and releases resources.
 *
 * @param fd File descriptor returned by vib_open()
 * @return 0 on success, negative errno code on failure
 */
int vib_close(driver_fd_t fd);

/**
 * @brief Write vibration control data
 *
 * Writes a vib_control_t structure to control motor state.
 * Buffer must be at least sizeof(vib_control_t) bytes.
 *
 * @param fd File descriptor returned by vib_open()
 * @param buf Pointer to vib_control_t structure
 * @param count Size of buffer (must be >= sizeof(vib_control_t))
 * @return Number of bytes written on success, negative errno code on failure
 */
ssize_t vib_write(driver_fd_t fd, const void *buf, size_t count);

/**
 * @brief Read vibration state
 *
 * Reads current vibration state into vib_control_t structure.
 *
 * @param fd File descriptor returned by vib_open()
 * @param buf Pointer to buffer to store vib_control_t
 * @param count Size of buffer (must be >= sizeof(vib_control_t))
 * @return Number of bytes read on success, negative errno code on failure
 */
ssize_t vib_read(driver_fd_t fd, void *buf, size_t count);

/**
 * @brief Control the vibration driver
 *
 * Performs control operations (set intensity, enable, get stats, etc.)
 *
 * @param fd File descriptor returned by vib_open()
 * @param cmd ioctl command (VIB_IOCTL_*)
 * @param arg Command-specific argument (depends on cmd)
 * @return 0 on success, negative errno code on failure
 */
int vib_ioctl(driver_fd_t fd, unsigned int cmd, void *arg);

#endif /* VIB_HANDLER_H_ */
