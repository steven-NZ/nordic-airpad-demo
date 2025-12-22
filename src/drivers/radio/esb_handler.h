/*
 * ESB Hardware Handler Interface
 */

#ifndef ESB_HANDLER_H_
#define ESB_HANDLER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "../driver_framework.h"

/* Unified sensor data packet structure */
typedef struct {
	uint8_t btn_state;      /* 3 bits: btn1, btn2, btn3 (1=pressed, 0=released) */
	uint16_t mgc_state;     /* MGC data: touch(byte 1) + airwheel(byte 2) */
	int16_t quat_w;         /* Quaternion w component (scaled by 32767) */
	int16_t quat_x;         /* Quaternion x component (scaled by 32767) */
	int16_t quat_y;         /* Quaternion y component (scaled by 32767) */
	int16_t quat_z;         /* Quaternion z component (scaled by 32767) */
} __attribute__((packed)) sensor_data_t;

/* Quaternion conversion constants
 * Quaternion components are normalized floats in range [-1.0, 1.0]
 * Scale factor: INT16_MAX (32767) for maximum resolution
 */
#define QUAT_SCALE_FACTOR 32767

/* Helper macro for quaternion float to int16_t conversion */
#define QUAT_FLOAT_TO_INT16(f) ((int16_t)((f) * QUAT_SCALE_FACTOR))

/* MGC state bit packing helpers (2-byte layout) */
/* Byte 1 (Touch) - Lower 8 bits */
#define MGC_TOUCH_MASK              0x000F  /* Bits 0-3: Touch electrodes */
#define MGC_TOUCH_RESERVED_MASK     0x00F0  /* Bits 4-7: Reserved */

/* Byte 2 (Airwheel) - Upper 8 bits */
#define MGC_AIRWHEEL_ACTIVE         0x0100  /* Bit 8: Airwheel active */
#define MGC_AIRWHEEL_DIRECTION_CW   0x0200  /* Bit 9: Direction (1=CW, 0=CCW) */
#define MGC_AIRWHEEL_VELOCITY_SHIFT 10      /* Bits 10-15: Velocity (0-63) */
#define MGC_AIRWHEEL_VELOCITY_MASK  0xFC00  /* Bits 10-15 mask */

/* Helper macro to pack MGC state into uint16_t */
#define MGC_PACK_STATE(touch, active, dir_cw, vel) \
	((uint16_t)( \
		((touch) & MGC_TOUCH_MASK) | \
		((active) ? MGC_AIRWHEEL_ACTIVE : 0) | \
		((dir_cw) ? MGC_AIRWHEEL_DIRECTION_CW : 0) | \
		(((vel) << MGC_AIRWHEEL_VELOCITY_SHIFT) & MGC_AIRWHEEL_VELOCITY_MASK) \
	))

/* Helper macros to unpack MGC state from uint16_t */
#define MGC_UNPACK_TOUCH(state)            ((uint8_t)((state) & MGC_TOUCH_MASK))
#define MGC_UNPACK_AIRWHEEL_ACTIVE(state)  (((state) & MGC_AIRWHEEL_ACTIVE) != 0)
#define MGC_UNPACK_DIRECTION_CW(state)     (((state) & MGC_AIRWHEEL_DIRECTION_CW) != 0)
#define MGC_UNPACK_VELOCITY(state)         ((uint8_t)(((state) & MGC_AIRWHEEL_VELOCITY_MASK) >> MGC_AIRWHEEL_VELOCITY_SHIFT))

/* ESB-specific ioctl commands */
#define ESB_IOCTL_SET_TX_POWER      0x3001  /* Set TX power (int8_t) */
#define ESB_IOCTL_GET_TX_POWER      0x3002  /* Get TX power (int8_t*) */
#define ESB_IOCTL_SET_BASE_ADDR_0   0x3003  /* Set base address 0 (uint8_t[4]) */
#define ESB_IOCTL_SET_BASE_ADDR_1   0x3004  /* Set base address 1 (uint8_t[4]) */
#define ESB_IOCTL_SET_ADDR_PREFIX   0x3005  /* Set address prefixes (uint8_t[8]) */
#define ESB_IOCTL_GET_TX_SUCCESS    0x3006  /* Get TX success count (uint32_t*) */
#define ESB_IOCTL_GET_TX_FAILED     0x3007  /* Get TX failed count (uint32_t*) */
#define ESB_IOCTL_GET_RX_COUNT      0x3008  /* Get RX packet count (uint32_t*) */
#define ESB_IOCTL_RESET_STATS       0x3009  /* Reset statistics counters */

/* ESB statistics structure */
typedef struct {
	uint32_t tx_success_count;
	uint32_t tx_failed_count;
	uint32_t rx_count;
} esb_stats_t;

/**
 * @brief Open the ESB driver
 *
 * Initializes the Enhanced ShockBurst radio and returns a file descriptor
 * for accessing the driver. Only one instance is supported.
 *
 * @param flags Open flags (reserved for future use, pass 0)
 * @return File descriptor on success, DRIVER_FD_INVALID on failure
 */
driver_fd_t esb_open(uint32_t flags);

/**
 * @brief Close the ESB driver
 *
 * Releases resources associated with the ESB driver instance and
 * disables the radio.
 *
 * @param fd File descriptor returned by esb_open()
 * @return 0 on success, negative errno code on failure
 */
int esb_close(driver_fd_t fd);

/**
 * @brief Read received data from ESB
 *
 * Reads received packets from the ESB RX FIFO. Non-blocking operation
 * returns DRIVER_ERR_AGAIN if no data is available.
 *
 * @param fd File descriptor returned by esb_open()
 * @param buf Pointer to buffer to store received data
 * @param count Size of buffer
 * @return Number of bytes read on success, negative errno code on failure
 */
ssize_t esb_read(driver_fd_t fd, void *buf, size_t count);

/**
 * @brief Write data to ESB for transmission
 *
 * Writes a packet to the ESB TX FIFO for transmission. The packet
 * should typically be a sensor_data_t structure.
 *
 * @param fd File descriptor returned by esb_open()
 * @param buf Pointer to data to transmit
 * @param count Size of data to transmit
 * @return Number of bytes written on success, negative errno code on failure
 */
ssize_t esb_write(driver_fd_t fd, const void *buf, size_t count);

/**
 * @brief Control the ESB driver
 *
 * Performs control operations on the ESB driver such as configuring
 * TX power, addresses, reading statistics, etc.
 *
 * @param fd File descriptor returned by esb_open()
 * @param cmd ioctl command (ESB_IOCTL_*)
 * @param arg Command-specific argument (depends on cmd)
 * @return 0 on success, negative errno code on failure
 */
int esb_ioctl(driver_fd_t fd, unsigned int cmd, void *arg);

#endif /* ESB_HANDLER_H_ */
