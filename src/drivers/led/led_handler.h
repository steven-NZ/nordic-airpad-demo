/*
 * LED Handler Interface
 * APA102C RGB LED Driver
 */

#ifndef LED_HANDLER_H_
#define LED_HANDLER_H_

#include <stdint.h>
#include <stdbool.h>
#include "../driver_framework.h"

/* RGB color structure */
typedef struct {
	uint8_t red;        /* Red channel (0-255) */
	uint8_t green;      /* Green channel (0-255) */
	uint8_t blue;       /* Blue channel (0-255) */
} __attribute__((packed)) led_color_t;

/* Single LED control structure */
typedef struct {
	uint8_t led_index;      /* LED index (0-5) */
	uint8_t brightness;     /* Global brightness (0-31, 5-bit) */
	led_color_t color;      /* RGB color */
} __attribute__((packed)) led_control_t;

/* LED array control structure for all LEDs */
typedef struct {
	uint8_t brightness;     /* Global brightness for all LEDs (0-31) */
	led_color_t leds[6];    /* Color for each of 6 LEDs */
} __attribute__((packed)) led_array_control_t;

/* LED-specific ioctl commands (range: 0x6000-0x6FFF) */
#define LED_IOCTL_SET_BRIGHTNESS     0x6001  /* Set global brightness (uint8_t) */
#define LED_IOCTL_SET_COLOR          0x6002  /* Set LED color (led_control_t*) */
#define LED_IOCTL_SET_ALL_OFF        0x6003  /* Turn all LEDs off */
#define LED_IOCTL_GET_STATS          0x6004  /* Get statistics (led_stats_t*) */
#define LED_IOCTL_RESET_STATS        0x6005  /* Reset statistics counters */

/* LED statistics */
typedef struct {
	uint32_t write_count;       /* Number of write operations */
	uint32_t update_count;      /* Number of SPI updates sent */
	uint32_t spi_error_count;   /* Number of SPI errors */
} led_stats_t;

/**
 * @brief Open the LED driver
 *
 * Initializes SPI peripheral and returns a file descriptor.
 * Only one instance is supported.
 *
 * @param flags Open flags (reserved for future use, pass 0)
 * @return File descriptor on success, DRIVER_FD_INVALID on failure
 */
driver_fd_t led_open(uint32_t flags);

/**
 * @brief Close the LED driver
 *
 * Turns off all LEDs and releases resources.
 *
 * @param fd File descriptor returned by led_open()
 * @return 0 on success, negative errno code on failure
 */
int led_close(driver_fd_t fd);

/**
 * @brief Write LED control data
 *
 * Writes LED control data to update LED colors and brightness.
 * Accepts either led_control_t (single LED) or led_array_control_t (all LEDs).
 *
 * @param fd File descriptor returned by led_open()
 * @param buf Pointer to led_control_t or led_array_control_t structure
 * @param count Size of buffer
 * @return Number of bytes written on success, negative errno code on failure
 */
ssize_t led_write(driver_fd_t fd, const void *buf, size_t count);

/**
 * @brief Read LED state
 *
 * Reads current LED state into led_array_control_t structure.
 *
 * @param fd File descriptor returned by led_open()
 * @param buf Pointer to buffer to store led_array_control_t
 * @param count Size of buffer (must be >= sizeof(led_array_control_t))
 * @return Number of bytes read on success, negative errno code on failure
 */
ssize_t led_read(driver_fd_t fd, void *buf, size_t count);

/**
 * @brief Control the LED driver
 *
 * Performs control operations (set brightness, color, get stats, etc.)
 *
 * @param fd File descriptor returned by led_open()
 * @param cmd ioctl command (LED_IOCTL_*)
 * @param arg Command-specific argument (depends on cmd)
 * @return 0 on success, negative errno code on failure
 */
int led_ioctl(driver_fd_t fd, unsigned int cmd, void *arg);

#endif /* LED_HANDLER_H_ */
