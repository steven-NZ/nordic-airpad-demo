# Nordic Airpad Demo - System Design

## Overview
Wireless sensor node transmitting IMU orientation (quaternion), MGC gesture data, and button state via Nordic ESB at 100Hz.

**Hardware:**
- nRF52840 DK
- ICM-42670-P IMU (I2C0) - quaternion fusion via RTQF algorithm
- MGC3130 gesture sensor (I2C1) - 4-electrode touch + AirWheel
- 3 buttons (GPIO interrupts)

**Architecture:**
- Single central thread (100Hz, 4KB stack)
- POSIX-style driver API (open/read/write/ioctl)
- 14-byte ESB packet: buttons + MGC state + quaternion

---

## Hardware Configuration

| Component | Bus/Pin | Config | Notes |
|-----------|---------|--------|-------|
| ICM-42670-P IMU | I2C0 (P0.26/27) | 100kHz, ±16g, ±2000dps, 100Hz | 6-DOF sensor |
| MGC3130 Gesture | I2C1 (P0.30/31) | 100kHz, addr 0x42 | TS handshake (P0.29), MCLR (P0.28) |
| Button 0/1/2 | P0.11/12/13 | Active-low, pull-up, interrupt | Simple state tracking |
| ESB Radio | nRF radio | 2.4GHz, 1Mbps, 8dBm TX | 14-byte packet @ 100Hz |

**Not Implemented:** LED/vibration output (TODO: central_thread.c:209)

---

## Architecture

**Central Thread (Priority 5, 4KB stack, 10ms period):**
```
Loop @ 100Hz:
  1. imu_read() → raw accel/gyro
  2. imu_fusion_update() → quaternion (RTQF algorithm)
  3. btn_read() → 3-bit button state
  4. mgc_read() → touch electrodes + airwheel
  5. mgc_ioctl(GET_ESB_STATE) → pack to 2 bytes
  6. Build sensor_data_t (14 bytes)
  7. esb_write() → transmit packet
  8. esb_read() → poll for RX (currently just logged)
```

**Button ISRs:** Update shared state variable (interrupt-driven)
**Drivers:** POSIX-style API (open/close/read/write/ioctl), mutex-protected

---

## Sensor Protocols

### Buttons
- GPIO interrupts (rising/falling edges)
- Returns 3-bit state (1=pressed, 0=released)
- No debouncing or complex state machine

### MGC3130 Gesture Sensor (I2C, addr 0x42)
**Protocol:** TS handshake required, 200μs delay after I2C transfer
**Touch:** 4 electrodes (N/S/E/W), read from msg 0x91
**AirWheel:** Counter 0-255 (CW increments, CCW decrements), velocity from delta

**2-Byte ESB Encoding:**
- Byte 1: Touch bits [0-3]
- Byte 2: Active flag, Direction (CW=1), Velocity [0-63]

### IMU Fusion (RTQF Algorithm)
**Input:** Accel + Gyro @ 100Hz
**Process:** Predict (integrate gyro) → Measure (accel gravity) → Correct (SLERP blend, 2% power)
**Output:** Quaternion (w,x,y,z) avoiding gimbal lock
**Encoding:** int16_t = float × 32767

---

## ESB Packet Format

**sensor_data_t (14 bytes packed):**
```c
uint8_t  btn_state;    // 3 bits used
uint16_t mgc_state;    // Byte1: touch[0-3], Byte2: airwheel flags+velocity
int16_t  quat_w/x/y/z; // Quaternion × 32767 (decode: /32767.0)
```

**ESB Config:** 2.4GHz, 1Mbps, 8dBm TX, 3 retries, ACK enabled, 32B max payload

---

## Code Organization

```
src/
├── main.c, central_thread.c/h      # Init, 100Hz main loop
├── drivers/driver_framework.h       # POSIX API (open/read/write/ioctl)
├── input/
│   ├── btn_handler.c/h              # Button GPIO interrupts
│   └── mgc_handler.c/h              # MGC3130 I2C + TS protocol
├── sensors/
│   ├── imu_handler.c/h              # ICM-42670-P I2C
│   ├── imu_fusion.c/h               # RTQF algorithm
│   └── imu_math.c/h                 # Quaternion math
└── comm/esb_handler.c/h             # ESB TX/RX
```

---

## Device Tree (nrf52840dk_nrf52840.overlay)

```dts
&i2c0 {  /* P0.26/27, 100kHz */
    icm42670p@68 {
        accel-hz = <100>; accel-fs = <16>;
        gyro-hz = <100>; gyro-fs = <2000>;
    };
};

&i2c1 {  /* P0.30/31, 100kHz - MGC3130 @ 0x42 (no DT binding, SW driver) */
};

/ {
    buttons {
        button0/1/2 { gpios = <&gpio0 11/12/13 GPIO_ACTIVE_LOW>; };
    };
};
```

---

## Important Notes

**Threading:** Single central thread, no message queues - all drivers called directly

**MGC3130 Quirks:**
- Requires TS handshake (TS LOW = data ready)
- Mandatory 200μs delay after each I2C transfer
- No Zephyr DT binding, uses raw I2C API

**Not Implemented:**
- LED/vibration output (TODO: central_thread.c:209)
- Complex button events (double-click, hold)
- Power optimizations (data-ready interrupts, sleep modes)

---

## Key Terms

- **RTQF**: Real-Time Quaternion Filter (6-DOF fusion, SLERP-based drift correction)
- **ESB**: Enhanced ShockBurst (Nordic 2.4GHz protocol)
- **MGC3130**: Microchip gesture sensor (I2C addr 0x42)
- **TS**: Transfer Status (MGC3130 handshake line)
- **Quaternion**: 4D rotation representation avoiding gimbal lock
- **SLERP**: Spherical Linear Interpolation (quaternion blending)