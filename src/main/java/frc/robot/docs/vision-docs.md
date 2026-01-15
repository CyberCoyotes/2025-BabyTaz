# Vision System Documentation: Limelight Integration for FRC
Last updated 12/28/2024

## Overview
This codebase implements a robust vision system for FRC robots using Limelight cameras. It follows a modular architecture with abstraction layers to separate hardware interfaces from high-level logic.

## Key Components

### 1. VisionIO Interface
`VisionIO.java` defines the hardware abstraction layer:
- Provides a common interface for vision sensors
- Uses an inner class `VisionIOInputs` to store sensor data
- Supports basic operations like LED control
- Allows for easy testing and simulation through interface implementation

### 2. VisionIOLimelight Implementation
`VisionIOLimelight.java` implements the VisionIO interface specifically for Limelight cameras:
- Manages NetworkTable communications with Limelight
- Caches sensor values to minimize network queries
- Handles unit conversions (degrees to radians)
- Provides thread-safe updates through synchronization

### 3. LimelightHelpers Utility
`LimelightHelpers.java` provides extensive utility functions for Limelight operations:
- Pose estimation and coordinate transformations
- AprilTag and fiducial marker detection
- JSON parsing for complex data structures
- Camera configuration and LED control
- Network table management

### 4. VisionSubsystem Integration
`VisionSubsystem.java` ties everything together:
- Manages the vision processing pipeline
- Implements robust validation and filtering
- Integrates with drivetrain for pose estimation
- Handles LED feedback and driver station reporting

## Key Features

### Pose Estimation
- Supports both 2D and 3D pose estimation
- Handles alliance-specific coordinate systems
- Implements median filtering for noise reduction
- Validates poses with multiple confidence checks

### Target Validation
The system uses multiple criteria to validate targets:
```java
private static final double MAX_POSE_CHANGE_METERS = 1.0;
private static final double MAX_VALID_DISTANCE = 5.0;
private static final double MIN_VALID_DISTANCE = 0.5;
private static final double MAX_TAG_AGE_SECONDS = 0.25;
```

### Error Handling
Robust error handling throughout the system:
- Graceful degradation on sensor failure
- Comprehensive error reporting to Driver Station
- Timestamp validation for data consistency
- Multiple redundancy checks

### Configuration
Limelight mounting parameters are cleanly organized:
```java
private static class LimelightConstants {
    static final double MOUNT_HEIGHT_METERS = 0.5;
    static final double TARGET_HEIGHT_METERS = 1.45;
    static final double MOUNT_ANGLE_RADIANS = Math.toRadians(30.0);
    // ... other mount parameters
}
```

## Usage Example

1. Initialize the vision subsystem:
```java
VisionIO limelightIO = new VisionIOLimelight("limelight");
VisionSubsystem vision = new VisionSubsystem(
    "limelight",
    limelightIO,
    drivetrain,
    ledSubsystem,
    fieldLayout
);
```

2. Use vision data in your robot code:
```java
if (vision.hasValidTarget()) {
    double xOffset = vision.getTargetXOffset();
    double area = vision.getTargetArea();
    // Use the data for alignment, targeting, etc.
}
```

## Best Practices

1. **Validation**: Always check target validity before using vision data
2. **Filtering**: Use the built-in median filters for smooth data
3. **Error Handling**: Monitor Driver Station for vision-related errors
4. **Configuration**: Update mounting parameters to match your robot
5. **Testing**: Utilize the interface abstraction for simulation testing

The system is designed to be robust and maintainable while providing accurate vision processing for FRC robotics applications. It handles common edge cases and provides extensive telemetry for debugging and monitoring.