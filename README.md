# FPV Drone Firmware Project Structure

```
fpv-drone-firmware/
├── Forge.toml                 # Project configuration
├── README.md                  # Project documentation
├── .gitignore                 # Git ignore file
├── memory.ld                  # Linker script for STM32F405
├── memory_h743.ld             # Linker script for STM32H743
├── memory_rp2040.ld           # Linker script for RP2040
├── build.rs                   # Build script
├── src/
│   ├── main.forge             # Main entry point
│   ├── hardware/              # Hardware abstraction modules
│   │   ├── mod.forge
│   │   ├── stm32f4.forge      # STM32F4 HAL
│   │   ├── stm32h7.forge      # STM32H7 HAL
│   │   ├── rp2040.forge       # RP2040 HAL
│   │   ├── timers.forge       # Timer peripherals
│   │   ├── spi.forge          # SPI interface
│   │   ├── uart.forge         # UART interface
│   │   ├── dma.forge          # DMA controller
│   │   └── gpio.forge         # GPIO control
│   ├── protocols/             # Communication protocols
│   │   ├── mod.forge
│   │   ├── dshot.forge        # DShot ESC protocol
│   │   ├── sbus.forge         # SBUS receiver protocol
│   │   ├── crsf.forge         # CRSF receiver protocol
│   │   └── msp.forge          # MSP telemetry protocol
│   ├── sensors/               # Sensor drivers
│   │   ├── mod.forge
│   │   ├── imu.forge          # IMU sensor interface
│   │   ├── mpu6000.forge      # MPU6000 driver
│   │   ├── icm20602.forge     # ICM20602 driver
│   │   ├── bmi270.forge       # BMI270 driver
│   │   ├── barometer.forge    # Barometer interface
│   │   └── gps.forge          # GPS receiver
│   ├── control/               # Flight control modules
│   │   ├── mod.forge
│   │   ├── pid.forge          # PID controller
│   │   ├── sensor_fusion.forge # Attitude estimation
│   │   ├── flight_modes.forge # Flight mode logic
│   │   └── motor_mixer.forge  # Motor mixing
│   ├── safety/                # Safety and failsafe
│   │   ├── mod.forge
│   │   ├── arming.forge       # Arming logic
│   │   ├── failsafe.forge     # Failsafe handling
│   │   └── emergency.forge    # Emergency procedures
│   ├── telemetry/             # Communication systems
│   │   ├── mod.forge
│   │   ├── protocol.forge     # Telemetry protocol
│   │   ├── usb.forge          # USB communication
│   │   └── radio.forge        # Radio telemetry
│   ├── storage/               # Data storage
│   │   ├── mod.forge
│   │   ├── config.forge       # Configuration management
│   │   ├── blackbox.forge     # Flight logging
│   │   ├── flash.forge        # SPI flash driver
│   │   └── sd_card.forge      # SD card interface
│   ├── osd/                   # On-screen display
│   │   ├── mod.forge
│   │   ├── max7456.forge      # Analog OSD chip
│   │   ├── msp_osd.forge      # MSP OSD for digital
│   │   └── elements.forge     # OSD elements
│   └── math/                  # Mathematical utilities
│       ├── mod.forge
│       ├── filters.forge      # Digital filters
│       ├── pid.forge          # PID implementations
│       ├── quaternion.forge   # Quaternion math
│       └── vector3.forge      # 3D vector operations
├── tests/                     # Test modules
│   ├── unit/                  # Unit tests
│   │   ├── test_pid.forge
│   │   ├── test_sensor_fusion.forge
│   │   ├── test_dshot.forge
│   │   └── test_safety.forge
│   ├── integration/           # Integration tests
│   │   ├── test_motor_control.forge
│   │   ├── test_flight_modes.forge
│   │   └── test_telemetry.forge
│   └── hardware/              # Hardware-in-the-loop tests
│       ├── test_imu_reading.forge
│       ├── test_receiver.forge
│       └── test_full_system.forge
├── tools/                     # Development tools
│   ├── flash-tool/            # Firmware flashing utility
│   │   ├── Forge.toml
│   │   └── src/
│   │       └── main.forge
│   ├── configurator/          # Ground station configurator
│   │   ├── Forge.toml
│   │   └── src/
│   │       ├── main.forge
│   │       ├── gui/
│   │       └── protocol/
│   └── blackbox-viewer/       # Flight log analyzer
│       ├── Forge.toml
│       └── src/
│           └── main.forge
├── docs/                      # Documentation
│   ├── api/                   # API documentation
│   ├── hardware/              # Hardware setup guides
│   │   ├── stm32f405.md
│   │   ├── stm32h743.md
│   │   └── rp2040.md
│   ├── protocols/             # Protocol specifications
│   │   ├── dshot.md
│   │   ├── sbus.md
│   │   ├── crsf.md
│   │   └── telemetry.md
│   ├── tuning/                # Tuning guides
│   │   ├── pid_tuning.md
│   │   ├── filter_setup.md
│   │   └── safety_config.md
│   └── examples/              # Example configurations
│       ├── racing_quad.toml
│       ├── freestyle_quad.toml
│       └── cinematic_quad.toml
├── scripts/                   # Build and deployment scripts
│   ├── build.sh               # Build script
│   ├── flash.sh               # Flash firmware script
│   ├── test.sh                # Run tests script
│   └── release.sh             # Create release builds
└── .github/                   # GitHub workflows
    └── workflows/
        ├── ci.yml             # Continuous integration
        ├── release.yml        # Release automation
        └── test.yml           # Automated testing
```

## Build Commands

### Basic Commands
```bash
# Build for default target (STM32F405)
forge build

# Build for specific target
forge build --target stm32h743

# Build with specific features
forge build --features "dshot1200,crsf,bmi270"

# Clean build artifacts
forge clean
```

### Development Commands
```bash
# Format all source code
forge fmt

# Check code without building
forge check

# Run unit tests
forge test

# Run tests on hardware
forge test --target stm32f405 --hardware

# Generate documentation
forge doc
```

### Deployment Commands
```bash
# Flash firmware to device
forge flash --target stm32f405

# Flash with specific programmer
forge flash --target stm32f405 --programmer stlink

# Enter DFU mode and flash
forge flash --target stm32f405 --dfu

# Debug with GDB
forge debug --target stm32f405
```

### Package Management
```bash
# Install dependencies
forge install embedded-hal

# Update dependencies
forge update

# Search for packages
forge search flight-control

# List installed packages
forge list
```

## File Descriptions

### Core Source Files
- **`main.forge`** - Main entry point, task scheduling, system initialization
- **`hardware/`** - Low-level hardware abstraction for different MCUs
- **`protocols/`** - Communication protocol implementations (DShot, SBUS, CRSF)
- **`sensors/`** - Sensor drivers and interfaces (IMU, barometer, GPS)
- **`control/`** - Flight control algorithms (PID, sensor fusion, flight modes)
- **`safety/`** - Safety systems (arming, failsafe, emergency procedures)
- **`telemetry/`** - Ground station communication and parameter tuning
- **`storage/`** - Configuration and blackbox logging
- **`osd/`** - On-screen display for FPV video feed
- **`math/`** - Mathematical utilities and algorithms

### Configuration Files
- **`Forge.toml`** - Main project configuration with targets and features
- **`memory.ld`** - Linker scripts for different MCU memory layouts
- **`build.rs`** - Custom build script for hardware-specific compilation

### Testing
- **`tests/unit/`** - Unit tests for individual modules
- **`tests/integration/`** - Integration tests for module interactions
- **`tests/hardware/`** - Hardware-in-the-loop tests for real hardware

### Tools
- **`tools/flash-tool/`** - Firmware flashing utility
- **`tools/configurator/`** - Ground station software for tuning
- **`tools/blackbox-viewer/`** - Flight log analysis tool

### Documentation
- **`docs/hardware/`** - Hardware setup and pinout guides
- **`docs/protocols/`** - Communication protocol specifications
- **`docs/tuning/`** - PID tuning and configuration guides
- **`docs/examples/`** - Example configurations for different aircraft types

## Key Features of Forge Implementation

### Memory Safety
- **Zero-cost abstractions** - High-level code compiles to optimal assembly
- **Ownership system** - Prevents memory corruption and race conditions
- **No garbage collector** - Deterministic real-time behavior

### Real-Time Guarantees
- **Compile-time timing verification** - Ensures control loop meets deadlines
- **Priority-based scheduling** - Critical tasks get guaranteed execution time
- **Deterministic memory allocation** - No dynamic allocation in flight code

### Safety-Critical Design
- **Built-in safety constraints** - Compile-time verification of safety rules
- **Type-safe hardware interfaces** - Prevents invalid register access
- **Formal verification support** - Mathematical proofs of correctness

### Hardware Integration
- **Direct register access** - Zero-overhead hardware control
- **Type-safe peripherals** - Compile-time peripheral configuration validation
- **Cross-platform support** - Same code runs on STM32F4, STM32H7, and RP2040

### Development Experience
- **Comprehensive tooling** - Integrated compiler, debugger, and package manager
- **Hardware-in-the-loop testing** - Test on real hardware automatically
- **Live parameter tuning** - Adjust PID gains and filters in real-time
- **Advanced debugging** - GDB integration with real-time trace capabilities

This project structure demonstrates how Forge enables building complex, safety-critical embedded systems with the expressiveness of high-level languages while maintaining the performance and reliability required for FPV drone flight controllers.
