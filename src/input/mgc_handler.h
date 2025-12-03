/*
 * MGC3130 Gesture Sensor Driver
 */

#ifndef MGC_HANDLER_H
#define MGC_HANDLER_H

#include <stdint.h>
#include <stddef.h>
#include "../drivers/driver_framework.h"

/* MGC3130 Message IDs */
#define MGC3130_MSG_FW_VERSION_INFO    0x83
#define MGC3130_MSG_SYSTEM_STATUS      0x15
#define MGC3130_MSG_REQUEST_MESSAGE    0x06

/* MGC3130 Message Header */
typedef struct {
	uint16_t size;      /* Total message size including header */
	uint8_t  flags;     /* Message flags */
	uint8_t  seq;       /* Sequence counter */
	uint8_t  id;        /* Message ID */
} __packed mgc3130_msg_header_t;

/* Firmware Version Info - ASCII string buffer */
#define MGC3130_FW_VERSION_MAX_LEN  256

typedef struct {
	char version_string[MGC3130_FW_VERSION_MAX_LEN];  /* ASCII: "1.0.0;p:HillstarV01;..." */
	size_t length;
} mgc3130_fw_version_t;

/* Driver API */
driver_fd_t mgc_open(void);
int mgc_close(driver_fd_t fd);
ssize_t mgc_read(driver_fd_t fd, void *buf, size_t count);
int mgc_ioctl(driver_fd_t fd, unsigned int cmd, void *arg);

/* IOCTL Commands */
#define MGC_IOCTL_GET_FW_VERSION  0x01

#endif /* MGC_HANDLER_H */
