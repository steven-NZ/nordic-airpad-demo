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
┌─────────────────────────────────────────────────┐
│              Main Thread (Init Only)            │
└─────────────────────────────────────────────────┘
           │            │            │
    ┌──────▼────┐  ┌───▼─────┐  ┌──▼──────┐
    │  Input    │  │   ESB   │  │ Output  │
    │  Thread   │  │  Thread │  │ Thread  │
    │  (Prio 5) │  │ (Prio 7)│  │ (Prio 8)│
    └───────────┘  └─────────┘  └─────────┘
```

**Thread Priorities**: Input (5) > ESB (7) > Output (8) > Main (10)

### Communication Flow
```
[Inputs] → MsgQ → [ESB Thread] ⇄ Radio ⇄ Remote Device
                       ↓
                     MsgQ
                       ↓
                  [Output Thread] → [LEDs/Vibration]
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
    input_event_t input_event;
    imu_data_t imu;             // xyz accel/gyro
    uint8_t cap_state;
    uint32_t timestamp_ms;
} __packed sensor_packet_t;
```

### Output Command Format
```c
typedef struct {
    uint8_t command_id;
    pattern_config_t led_pattern;
    pattern_config_t vib_pattern;
} __packed output_command_t;
```

### ESB Thread Responsibilities
1. Poll `input_to_esb_queue` for input events (timeout: 100ms)
2. Periodically read IMU (every 100ms or on data-ready)
3. Package and transmit sensor packets
4. Receive output commands from remote device
5. Forward commands to `esb_to_output_queue`

### Configuration
- Payload size: 32 bytes (configurable)
- TX power: 0dBm (configurable)
- Retry count: 3
- ACK enabled

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

## Message Queues

```c
// Input → ESB
K_MSGQ_DEFINE(input_to_esb_queue, sizeof(input_event_t), 10, 4);

// ESB → Output  
K_MSGQ_DEFINE(esb_to_output_queue, sizeof(output_command_t), 10, 4);
```

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
├── main.c                      # System init, thread creation
├── input/
│   ├── btn_handler.c             # Button state machine
│   ├── cap_handler.c      # Cap sensor state machine  
│   └── input_thread.c          # Input event aggregation
├── sensors/
│   └── imu_reader.c            # IMU polling/interrupt handling
├── comm/
│   └── esb_thread.c            # ESB TX/RX and packet handling
├── output/
│   ├── led_patterns.c          # LED pattern engine
│   ├── vib_patterns.c          # Vibration pattern engine
│   └── output_thread.c         # Output coordination
└── common/
    ├── events.h                # Event type definitions
    └── patterns.h              # Pattern structures
```

---

## Key Design Decisions

### Why Interrupts for Buttons?
- Lower latency (immediate response)
- Reduced power consumption (no polling)
- Cleaner code with Zephyr's GPIO callback API

### Why State Machines?
- Clear, maintainable input interpretation
- Easy to add new gesture types
- Separates timing logic from application logic

### Why Message Queues?
- Thread-safe communication
- Decouples producers/consumers
- Built-in blocking/timeout support

### Why Separate Threads?
- Input: High priority for responsiveness
- ESB: Independent radio timing requirements
- Output: Non-blocking pattern generation
- Clean separation of concerns

---

## Timing Budget

| Task | Period | Max Duration |
|------|--------|--------------|
| Button debounce | 50ms | 5ms |
| IMU read | 100ms | 10ms |
| ESB TX | 100ms | 15ms |
| LED update | 33ms | 8ms |
| Vibration update | 20ms | 2ms |

---

## Stack Sizes (Recommended)

- Input Thread: 2KB
- ESB Thread: 2KB  
- Output Thread: 2KB
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