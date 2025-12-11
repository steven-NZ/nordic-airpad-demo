/*
 * MGC3130 Gesture Sensor Driver
 */

#ifndef MGC_HANDLER_H
#define MGC_HANDLER_H

#include <stdint.h>
#include <stddef.h>
#include "../drivers/driver_framework.h"

/* MGC3130 Message IDs */
#define MGC3130_MSG_FW_VERSION_INFO       0x83
#define MGC3130_MSG_SYSTEM_STATUS         0x15
#define MGC3130_MSG_REQUEST_MESSAGE       0x06
#define MGC3130_MSG_SET_RUNTIME_PARAMETER 0xA2
#define MGC3130_MSG_SENSOR_DATA_OUTPUT    0x91

/* MGC3130 Message Header */
typedef struct {
	uint8_t size;      /* Message Size */
	uint8_t  flags;     /* Message flags */
	uint8_t  seq;       /* Sequence counter */
	uint8_t  id;        /* Message ID */
} __packed mgc3130_msg_header_t;

/* Runtime Parameter IDs */
#define MGC3130_PARAM_DSP_TOUCH_CONFIG      0x0097  /* Touch detection configuration */
#define MGC3130_PARAM_DSP_AIR_WHEEL_CONFIG  0x0090  /* AirWheel detection configuration */
#define MGC3130_PARAM_DATA_OUTPUT_ENABLE    0x00A0  /* Data output enable mask */

/* Touch Configuration Values */
#define MGC3130_TOUCH_CONFIG_ENABLE_ARG0   0x08  /* Enable touch detection (arg0) */
#define MGC3130_TOUCH_CONFIG_ENABLE_ARG1   0x08  /* Enable touch detection (arg1) */

/* AirWheel Configuration Values */
#define MGC3130_AIRWHEEL_CONFIG_ENABLE_ARG0  0x20  /* Enable airwheel (arg0) */
#define MGC3130_AIRWHEEL_CONFIG_ENABLE_ARG1  0x20  /* Enable airwheel (arg1) */

/* Data Output Configuration Masks */
#define MGC3130_OUTPUT_MASK_OVERWRITE      0xFFFFFFFF  /* Overwrite all output config bits */

/* Touch Electrode Mask */
#define MGC3130_TOUCH_ELECTRODE_MASK       0x0F  /* Bits 0-3: South, West, North, East */

/* DataOutputConfigMask bits */
#define MGC3130_OUTPUT_DSP_STATUS      (1 << 0)
#define MGC3130_OUTPUT_GESTURE_INFO    (1 << 1)
#define MGC3130_OUTPUT_TOUCH_INFO      (1 << 2)
#define MGC3130_OUTPUT_AIR_WHEEL_INFO  (1 << 3)
#define MGC3130_OUTPUT_XYZ_POSITION    (1 << 4)

/* Touch electrode bit positions */
#define MGC3130_TOUCH_SOUTH   (1 << 0)
#define MGC3130_TOUCH_WEST    (1 << 1)
#define MGC3130_TOUCH_NORTH   (1 << 2)
#define MGC3130_TOUCH_EAST    (1 << 3)
#define MGC3130_TOUCH_CENTER  (1 << 4)

/* SystemInfo field bit definitions (from Sensor_Data_Output) */
#define MGC3130_SYSINFO_POSITION_VALID   (1 << 0)  /* Position data valid */
#define MGC3130_SYSINFO_AIRWHEEL_VALID   (1 << 1)  /* AirWheel gesture active */
#define MGC3130_SYSINFO_RAW_DATA_VALID   (1 << 2)  /* Raw data available */

/* Request_Message Structure (ID: 0x06) - Used to request data from device */
typedef struct {
	mgc3130_msg_header_t header;  /* 4 bytes: size, flags, seq, id=0x06 */
	uint8_t requested_msg_id;     /* 1 byte: Message ID to request */
	uint8_t reserved[3];          /* 3 bytes: Reserved */
	uint32_t param;               /* 4 bytes: Parameter */
} __packed mgc3130_request_msg_t;

/* SET_RUNTIME_PARAMETER Message Structure */
typedef struct {
	mgc3130_msg_header_t header;  /* size=16, flags=0, seq=0, id=0xA2 */
	uint16_t param_id;            /* Runtime parameter ID */
	uint16_t reserved;            /* Reserved - must be 0 */
	uint32_t arg0;                /* Argument 0 */
	uint32_t arg1;                /* Argument 1 */
} __packed mgc3130_set_runtime_param_msg_t;

/* System_Status Message Structure */
typedef struct {
	mgc3130_msg_header_t header;  /* 4 bytes: size, flags, seq, id=0x15 */
	uint8_t msg_id;               /* Message ID which System_Status corresponds to */
	uint8_t max_cmd_size;         /* Maximum I2C packet size device accepts */
	uint16_t error_code;          /* Error code for previous message (16-bit) */
	uint64_t reserved;            /* Reserved (4 bytes) */
} __packed mgc3130_system_status_msg_t;

/* TouchInfo Field Structure (4 bytes) */
typedef struct {
	uint16_t touch_flags;      /* Bits 0-4: electrode touch */
	uint8_t touch_counter;     /* Touch duration counter (value × 5ms) */
	uint8_t reserved;          /* Reserved */
} __packed mgc3130_touch_info_t;

/* AirWheelInfo Field Structure (2 bytes) */
typedef struct {
	uint8_t counter;       /* Counter value (0-255): increments CW, decrements CCW */
	uint8_t reserved;      /* Reserved */
} __packed mgc3130_airwheel_info_t;

/* Sensor_Data_Output Message Structure (variable size) */
typedef struct {
	mgc3130_msg_header_t header;   /* size varies, id=0x91 */
	uint16_t config_mask;           /* DataOutputConfigMask */
	uint8_t timestamp;              /* Timestamp (200µs units) */
	uint8_t system_info;            /* System status */
	/* Variable fields follow based on config_mask */
	/* When bit 2 set: mgc3130_touch_info_t */
} __packed mgc3130_sensor_data_msg_t;

/* Touch State Tracking Structure */
typedef struct {
	uint8_t current_touch;     /* Current touch electrode bits */
	uint8_t previous_touch;    /* Previous touch electrode bits */
	uint32_t touch_start_time; /* Timestamp when touch started (ms) */
	bool touch_active;         /* True if any electrode is touched */
} mgc3130_touch_state_t;

/* AirWheel State Tracking Structure */
typedef struct {
	uint8_t current_counter;      /* Current counter value */
	uint8_t previous_counter;     /* Previous counter value */
	bool airwheel_valid;          /* True if airwheel is active (from SystemInfo bit 1) */
} mgc3130_airwheel_state_t;

/* Combined sensor data return structure */
typedef struct {
	mgc3130_touch_info_t touch;
	mgc3130_airwheel_info_t airwheel;
	uint8_t system_info;       /* From sensor data output (bit 1 = AirWheelValid) */
} mgc3130_sensor_output_t;

/* MGC3130 Configuration */
typedef struct {
	bool touch_enabled;        /* Touch detection configured */
	bool airwheel_enabled;     /* Airwheel detection configured */
	uint16_t output_mask;      /* Current output configuration mask */
} mgc3130_config_t;

/* MGC State for ESB Transmission */
typedef struct {
	uint8_t touch_electrodes;   /* Bits 0-3: N, S, E, W touch state */
	bool airwheel_active;       /* Airwheel detection active */
	bool airwheel_direction_cw; /* True=CW, False=CCW */
	uint8_t airwheel_velocity;  /* 0-3: velocity level */
} mgc3130_esb_state_t;

/* Firmware Version Info - ASCII string buffer */
#define MGC3130_FW_VERSION_MAX_LEN  256

/* Firmware Version String Field Prefixes */
#define MGC3130_FW_FIELD_PLATFORM     "p:"    /* Platform identifier */
#define MGC3130_FW_FIELD_DSP          "DSP:"  /* Colibri Suite/DSP version */
#define MGC3130_FW_FIELD_BUILD_TIME   "t:"    /* Build timestamp */

typedef struct {
	char version_string[MGC3130_FW_VERSION_MAX_LEN];  /* ASCII: "1.0.0;p:HillstarV01;..." */
	size_t length;
	/* Parsed fields */
	char library_version[32];     /* GestIC Library Version */
	char platform[32];            /* Platform */
	char colibri_version[32];     /* Colibri Suite Version (DSP) */
	char build_time[32];          /* Build Time */
} mgc3130_fw_version_t;

/* Driver API */
driver_fd_t mgc_open(void);
int mgc_close(driver_fd_t fd);
ssize_t mgc_read(driver_fd_t fd, void *buf, size_t count);
int mgc_ioctl(driver_fd_t fd, unsigned int cmd, void *arg);

/* IOCTL Commands (MGC range: 0x4000+) */
#define MGC_IOCTL_GET_FW_VERSION   0x4001  /* Get firmware version info */
#define MGC_IOCTL_CONFIGURE_TOUCH  0x4002  /* Configure touch detection */
#define MGC_IOCTL_READ_SENSOR_DATA 0x4003  /* Read sensor data */
#define MGC_IOCTL_GET_ESB_STATE    0x4004  /* Get MGC state for ESB (mgc3130_esb_state_t*) */

#endif /* MGC_HANDLER_H */
