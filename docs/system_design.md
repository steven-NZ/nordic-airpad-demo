# Zephyr RTOS System Design
**Sensor-to-ESB Controller with Pattern Output**

---

## System Overview
Embedded device that collects sensor inputs (IMU, capacitive touch, buttons), transmits data via Enhanced ShockBurst (ESB), and controls outputs (LEDs, vibration) based on received commands.

---

## Hardware Interface

### Inputs
- **IMU**: I2C interface
- **Capacitive Touch Sensor**: I2C interface  
- **3x Buttons**: GPIO (active low with pull-up)

### Outputs
- **LEDs**: SPI (addressable LEDs)
- **Vibration Motor**: GPIO with PWM

### Communication
- **ESB Radio**: Bidirectional wireless link

---

## Architecture

### Threading Model
```
┌─────────────────────────────────────────────┐
│         Main Thread (Init + Idle)           │
└─────────────────────────────────────────────┘
                      │
              ┌───────▼──────────┐
              │  Central Thread  │  (Single unified thread)
              │   (Priority 5)   │
              └──────────────────┘
                      │
        ┌─────────────┼─────────────┐
        │             │             │
   ┌────▼────┐   ┌───▼────┐   ┌───▼─────┐
   │   IMU   │   │  ESB   │   │ Output  │
   │Handler  │   │Handler │   │(future) │
   └─────────┘   └────────┘   └─────────┘

        + Button ISRs (interrupt-based)
```

**Thread Priorities**: Central Thread (5)

### Communication Flow
```
Central Thread (10ms period):
  ├─ Read IMU (imu_handler_read)
  ├─ Read Buttons (btn_handler_get_state)
  ├─ Package & Send ESB (esb_handler_send)
  └─ Receive & Process ESB (esb_handler_receive)

Button ISRs ────────> button_state (shared variable)
```

---

## Input System

### Input Processing Strategy
- **Buttons**: GPIO interrupts on edge (both rising/falling)
- **Cap Sensor**: I2C with interrupt pin (if available) or 20ms polling
- **IMU**: I2C with data-ready interrupt or 100Hz polling

### Event Detection State Machine
```
States: IDLE → PRESSED → WAIT_DOUBLE → HOLD_DETECT → IDLE
                    ↓          ↓            ↓
Events:      SINGLE_CLICK  DOUBLE_CLICK   HOLD/LONG_HOLD
```

**Timing Parameters**:
- Debounce: 50ms
- Double-click window: 400ms
- Hold threshold: 1000ms
- Long hold: 2000ms

### Input Event Structure
```c
typedef struct {
    uint8_t input_id;           // Button 0-2, Cap sensor
    input_event_type_t type;    // SINGLE/DOUBLE/HOLD/RELEASE
    uint64_t timestamp;
} input_event_t;
```

### Implementation
- GPIO interrupts trigger work submission to system workqueue
- State machines run in Input Thread context
- Debouncing handled in software
- Events pushed to `input_to_esb_queue`

---

## ESB Communication Layer

### Sensor Packet Format
```c
typedef struct {
    uint32_t sequence_num;
    uint8_t btn_state;          // 3 bits: btn1, btn2, btn3
    uint32_t timestamp_ms;
} __packed sensor_data_t;
```

### IMU Data
IMU data is read every 10ms and logged to console (not transmitted via ESB)

### Output Command Format
Received ESB data is currently logged (LED/vibration control not implemented)

### Central Thread Responsibilities
1. Read and log IMU sensor data via `imu_handler_read()`
2. Read button state via `btn_handler_get_state()`
3. Package button data into `sensor_data_t`
4. Transmit via `esb_handler_send()`
5. Receive and log commands via `esb_handler_receive()`
6. Period: 10ms (100Hz)

### Configuration
- Payload size: 32 bytes
- TX power: 8dBm (maximum range)
- Retry count: 3
- ACK enabled
- Bitrate: 1Mbps

---

## Output System

### Pattern Engine Architecture
```c
typedef enum {
    PATTERN_OFF,
    PATTERN_SOLID,
    PATTERN_BLINK,
    PATTERN_PULSE,
    PATTERN_BREATHE,
    PATTERN_CUSTOM
} pattern_type_t;

typedef struct {
    pattern_type_t type;
    uint32_t duration_ms;       // 0 = infinite
    uint8_t intensity;          // 0-255
    uint16_t period_ms;
    uint8_t duty_percent;       // For BLINK
    uint8_t custom_data[16];    // For CUSTOM patterns
} pattern_config_t;
```

### LED Control (SPI)
- Target: WS2812B or similar addressable LEDs
- Update rate: 30-60 Hz (pattern dependent)
- DMA transfer if available
- Color buffer management in Output Thread

### Vibration Control (PWM)
- Frequency: 50Hz (20ms period typical)
- Duty cycle: 0-100% for intensity control
- Pattern timing via k_timer

### Output Thread Operation
1. Wait on `esb_to_output_queue` (timeout: 50ms)
2. On new command: Load pattern configurations
3. Update LED and vibration states based on active patterns
4. Handle pattern completion and looping

---

## Inter-Module Communication

No message queues are used. The central thread directly calls hardware handler functions:
- `imu_handler_read()` - Returns IMU data via output parameter
- `btn_handler_get_state()` - Returns current button state
- `esb_handler_send()` - Sends sensor packet via ESB
- `esb_handler_receive()` - Polls for received ESB data

Button state is shared via a static variable updated by GPIO interrupts.

---

## Device Tree Configuration

```dts
/ {
    buttons {
        compatible = "gpio-keys";
        button0: button_0 { gpios = <&gpio0 11 GPIO_ACTIVE_LOW>; };
        button1: button_1 { gpios = <&gpio0 12 GPIO_ACTIVE_LOW>; };
        button2: button_2 { gpios = <&gpio0 13 GPIO_ACTIVE_LOW>; };
    };

    vibrator {
        compatible = "pwm-vibrator";
        pwms = <&pwm0 0 PWM_MSEC(20) PWM_POLARITY_NORMAL>;
    };
    
    leds_spi {
        compatible = "worldsemi,ws2812-spi";
        spi-max-frequency = <4000000>;
        spi-one-frame = <0x70>;
        spi-zero-frame = <0x40>;
    };
};

&i2c0 {
    imu: imu@68 { compatible = "invensense,mpu6050"; reg = <0x68>; };
    capsense: capsense@5a { compatible = "azoteq,iqs263"; reg = <0x5a>; };
};
```

---

## Source File Organization

```
src/
├── main.c                      # System initialization
├── central_thread.c            # Main orchestration thread (10ms period)
├── input/
│   └── btn_handler.c           # Button interrupt handling
├── sensors/
│   └── imu_handler.c           # IMU I2C interface (hardware layer)
└── comm/
    └── esb_handler.c           # ESB hardware interface
```

**Hardware Layer**: `imu_handler`, `btn_handler`, `esb_handler` provide clean hardware interfaces
**Application Layer**: `central_thread` orchestrates all hardware calls

---

## Key Design Decisions

### Why Single Central Thread?
- Simpler architecture - no message queue overhead
- Direct function calls reduce latency
- ESB communication handled by RTOS (no separate thread needed)
- Easier to reason about timing and execution order
- Clean separation: Hardware handlers vs. Application orchestration

### Why Interrupts for Buttons?
- Lower latency (immediate response)
- Reduced power consumption (no polling)
- Cleaner code with Zephyr's GPIO callback API
- Shared state variable is thread-safe for reads

### Why Hardware Handler Layer?
- Clean abstraction of hardware interfaces
- Easy to test and maintain
- Reusable across different application threads
- Separates concerns: hardware access vs. application logic

---

## Timing Budget

| Task | Period | Max Duration |
|------|--------|--------------|
| Central Thread Loop | 10ms | 8ms |
| - IMU read | 10ms | 3ms |
| - ESB TX | 10ms | 4ms |
| - ESB RX check | 10ms | 1ms |
| Button ISR | Event | <100μs |

---

## Stack Sizes

- Central Thread: 2KB
- System Workqueue: 2KB
- Main: 1KB

---

## Power Considerations

- Use GPIO interrupts (not polling) for buttons
- IMU: Configure to low-power mode when static
- ESB: Consider duty cycling radio if updates < 10Hz needed
- LEDs: Use low brightness or pulse patterns to reduce current
- Sleep mode: `pm_device_runtime_put()` on idle peripherals

---

## Debug Strategy

- Enable Zephyr logging: `LOG_MODULE_REGISTER()` per file
- Input events: Log all state transitions
- ESB: Log TX/RX packet counts and errors
- Output: Log pattern changes
- Use RTT or UART at 115200 baud

---

## Future Enhancements

- [ ] Battery monitoring (ADC)
- [ ] Persistent config storage (NVS)
- [ ] OTA firmware updates via ESB
- [ ] Multi-device ESB network support
- [ ] IMU gesture recognition (shake, tilt)