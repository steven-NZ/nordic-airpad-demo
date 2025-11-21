/*
 * ESB Hardware Handler Interface
 *
 * Provides hardware interface for Enhanced ShockBurst protocol
 */

#ifndef ESB_HANDLER_H_
#define ESB_HANDLER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Unified sensor data packet structure */
typedef struct {
	uint32_t sequence_num;
	uint8_t btn_state;      /* 3 bits: btn1, btn2, btn3 (1=pressed, 0=released) */
	uint32_t timestamp_ms;
} __attribute__((packed)) sensor_data_t;

/**
 * Initialize ESB hardware
 *
 * Sets up ESB protocol with configuration parameters
 *
 * @return 0 on success, negative error code on failure
 */
int esb_handler_init(void);

/**
 * Send sensor data packet via ESB
 *
 * @param data Pointer to sensor data to transmit
 * @return 0 on success, negative error code on failure
 */
int esb_handler_send(const sensor_data_t *data);

/**
 * Receive data from ESB
 *
 * @param buf Buffer to store received data
 * @param len Pointer to buffer size (input) and received length (output)
 * @return 0 on success, -ENODATA if no data available, negative error code on failure
 */
int esb_handler_receive(uint8_t *buf, size_t *len);

#endif /* ESB_HANDLER_H_ */
