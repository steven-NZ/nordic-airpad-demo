/*
 * ESB Thread Interface
 *
 * Handles Enhanced ShockBurst protocol communication
 */

#ifndef ESB_THREAD_H_
#define ESB_THREAD_H_

#include <stdint.h>
#include <stdbool.h>

/* Button event packet structure */
typedef struct {
    uint32_t sequence_num;
    uint8_t button_num;
    uint8_t pressed;
    uint32_t timestamp_ms;
} __attribute__((packed)) button_packet_t;

/**
 * Initialize ESB thread
 *
 * Sets up ESB protocol with configuration parameters
 *
 * @return 0 on success, negative error code on failure
 */
int esb_thread_init(void);

/**
 * Send button event packet
 *
 * @param button_num Button number (1-4)
 * @param pressed true if pressed, false if released
 */
void esb_thread_send_button_event(uint8_t button_num, bool pressed);

/**
 * Send simple character payload
 *
 * @param character Character to send
 */
void esb_thread_send_char(char character);

/**
 * Process ESB events
 *
 * Should be called periodically from main loop to handle
 * ESB TX/RX events and maintain protocol state
 */
void esb_thread_process(void);

#endif /* ESB_THREAD_H_ */
