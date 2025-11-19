/*
 * ESB Thread Interface
 *
 * Handles Enhanced ShockBurst protocol communication
 */

#ifndef ESB_THREAD_H_
#define ESB_THREAD_H_

#include <stdint.h>
#include <stdbool.h>

/* Unified sensor data packet structure */
typedef struct {
    uint32_t sequence_num;
    uint8_t btn_state;      /* 3 bits: btn1, btn2, btn3 (1=pressed, 0=released) */
    uint32_t timestamp_ms;
    /* Future expansion:
     * uint8_t cap_state;
     * int16_t imu_accel_x, y, z;
     * int16_t imu_gyro_x, y, z;
     */
} __attribute__((packed)) sensor_data_t;

/**
 * Initialize ESB thread
 *
 * Sets up ESB protocol with configuration parameters
 *
 * @return 0 on success, negative error code on failure
 */
int esb_thread_init(void);

/**
 * Process ESB events
 *
 * Should be called periodically from main loop to handle
 * ESB TX/RX events and maintain protocol state
 */
void esb_thread_process(void);

#endif /* ESB_THREAD_H_ */
