# Shuffleboad

Shuffleboard setup is commonly done in a RobotContainer or individual subsystems

## Within RobotContainer
- Used for autonomous command selection and high-level robot controls/settings
- Best for things that need to be accessible across multiple subsystems

## Within individual Subsystems
- Used for subsystem-specific data and tuning
- Each subsystem handles its own telemetry

## Examples

### Robot Container

```java
// At the top of RobotContainer.java
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

public class RobotContainer {
    // Create tabs for different groupings
    private final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
    private final ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    private final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");

    public RobotContainer() {
        // Configure Shuffleboard tabs
        configureShuffleboard();
        
        // Rest of your constructor...
    }

    private void configureShuffleboard() {
        // Add autonomous chooser to main tab
        mainTab.add("Auto Routine", autoChooser)
            .withSize(2, 1)
            .withPosition(0, 0);

        // Add drive controls/settings
        driveTab.addNumber("Max Speed", () -> MaxSpeed)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withSize(2, 1);

        // Add vision controls
        visionTab.addBoolean("Has Target", () -> vision.hasTarget())
            .withSize(1, 1)
            .withPosition(0, 0);
    }
}

```
### Subsystem

``` java
public class VisionSubsystem extends SubsystemBase {
    private final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
    
    public VisionSubsystem(String tableName, CommandSwerveDrivetrain drivetrain, LEDSubsystem leds) {
        // Constructor...
        
        configureShuffleboard();
    }

    private void configureShuffleboard() {
        // Add vision processing data
        visionTab.addNumber("Horizontal Offset", () -> getHorizontalOffset())
            .withWidget(BuiltInWidgets.kGraph)
            .withSize(3, 2)
            .withPosition(1, 0);
            
        visionTab.addNumber("Distance", () -> getDistance())
            .withWidget(BuiltInWidgets.kDial)
            .withSize(2, 2)
            .withPosition(4, 0);

        // Add vision settings
        visionTab.addBoolean("LED State", () -> ledsEnabled)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withSize(1, 1)
            .withPosition(0, 2);
    }

    @Override
    public void periodic() {
        // Existing periodic code...
        
        // Log to SmartDashboard - useful for basic debugging
        SmartDashboard.putNumber("Vision/TagID", getTagId());
        SmartDashboard.putString("Vision/State", currentState.toString());
    }
}

```

Reference conversation (https://claude.ai/chat/838dfcc4-69df-412d-8719-93d35be8f34c)
