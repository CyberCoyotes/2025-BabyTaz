# LED Subsystem Documentation

## Overview
The LED subsystem provides control over a CTRE CANdle LED controller, allowing for various LED states and animations on a robot. The system is designed to handle different states (like enabled, autonomous, targeting, etc.) with corresponding colors and animation patterns.

## Core Components

### 1. LEDConfig
- Handles configuration settings for the LED system
- Key constants:
  - `CANDLE_ID`: 1 (CAN bus identifier)
  - `LED_COUNT`: 300 (total number of LEDs)
  - `DEFAULT_BRIGHTNESS`: 0.9
- Supports zone definitions for LED segments
- Configurable properties include:
  - Strip type (GRB)
  - Brightness
  - VBat output mode
  - Status LED behavior
  - Loss of signal behavior

### 2. LEDState
An enumeration defining different robot states with corresponding RGB values:
- `OFF`: (0, 0, 0)
- `ENABLED`: (0, 255, 0)
- `DISABLED`: (255, 20, 0)
- `AUTONOMOUS`: (0, 0, 255)
- `TARGET_VISIBLE`: (255, 255, 0)
- `TARGET_LOCKED`: (0, 255, 0)
- `INTAKING`: (0, 255, 255)
- `SCORING`: (255, 105, 180)
- `ERROR`: (255, 0, 0)

### 3. LEDHardware
- Direct interface with the CANdle hardware
- Features:
  - Hardware configuration management
  - Error handling with retry logic (up to 3 attempts)
  - Status monitoring (voltage, current, temperature)
  - Animation and RGB control
- Provides diagnostic information through the `Status` class

### 4. LEDSubsystem
- Main control class extending WPILib's `SubsystemBase`
- Capabilities:
  - State management
  - Animation control
  - Brightness adjustment
  - Speed control for animations
- Animation types based on states:
  - Rainbow animation for `AUTONOMOUS`
  - Strobe animation for `TARGET_VISIBLE` and `ERROR`
  - Larson animation for `INTAKING`
  - Color flow animation for `SCORING`
  - Solid colors for other states

## Key Features

### Animation Support
- Different animation patterns for specific states:
  - Rainbow effects
  - Strobing effects
  - Scanning effects ("Larson scanner")
  - Color flow animations
- Configurable animation speed and brightness
- Option to disable animations entirely

### Error Handling
- Robust configuration retry system
- Automatic error state indication
- Extensive logging through `DataLogManager`
- Hardware status monitoring

### Telemetry
- Dashboard reporting via SmartDashboard:
  - Current state
  - Animation status
  - Hardware metrics (voltage, current, temperature)
  - Connection status
  - Configuration status

## Usage Example

```java
// Create and initialize the subsystem
LEDSubsystem ledSubsystem = new LEDSubsystem();

// Change LED state
ledSubsystem.setState(LEDState.AUTONOMOUS);

// Adjust animation properties
ledSubsystem.setAnimationSpeed(1.0);
ledSubsystem.setBrightness(0.8);

// Toggle animations
ledSubsystem.setAnimationEnabled(true);

// Handle vision targeting
ledSubsystem.setVisionLEDState(true);  // Sets to TARGET_VISIBLE
```

## Safety Features
- Brightness limits (0.0 to 1.0)
- Configuration validation
- Hardware connection monitoring
- Automatic error state indication on hardware failures
- Graceful handling of configuration failures
