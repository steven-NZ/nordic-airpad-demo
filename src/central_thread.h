/*
 * Central Thread Interface
 *
 * Orchestrates all sensor reading, communication, and output control
 */

#ifndef CENTRAL_THREAD_H_
#define CENTRAL_THREAD_H_

/**
 * Initialize central thread
 *
 * Sets up the main orchestration thread that coordinates:
 * - IMU sensor reading
 # - MGC sensor reading
 * - Button state reading
 * - ESB communication (TX/RX)
 * - Output control (LED, vibration)
 *
 * @return 0 on success, negative error code on failure
 */
int central_thread_init(void);

#endif /* CENTRAL_THREAD_H_ */
