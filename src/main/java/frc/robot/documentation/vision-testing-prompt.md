# AI Vision Model Testing

## Goal

Help me write **multiple** vision alignment test *models* using a Limelight 4 and AprilTags on my FRC robot, BabyTaz, which has a CTRE swerve drivetrain. Ultimately, I want code that can perform tasks:
    - (a) a rotational alignment with an AprilTag centered on the robot y-axis
    - (b) a rotational alignment with an AprilTag centered on the robot y-axis and the *target* x-axis distance
    - (c) a rotational alignment with an AprilTag centered and perpendicula to front of the robot y-axis and the *target* x-axis distance
    - (d) a hunt and seek pipeline (1) with a rotational alignment based on color blob detection (color should be a teal color) and the *target* x-axis distance

## Directions and assumptions

- Use separate subsystems and methods and commands for each model with good in-code documentation to have clear delineation from one model test to another
- Leave buttons available to test different vision alignment models with a single deploy; use virtual shuffleboard buttons if those are more feasible and reliable.
- The Limelight helper file LimelightHelpers v1.12 (REQUIRES LLOS 2025.0 OR LATER)) should already be up to date and not touched without convincing evidence otherwise.
- The Limelight physical offsets should already be present in code \vision\VisionConstants.java
- The previous test model "AlignToAprilTagCommand.java" with its subsystem was decent and needs further testing, so leave it intact aside from possible button re-mapping. otherwise do not assuming existing vision related subsystems and commands are correct as it was all beta testing
- With any of the model tests, output relevant data to Shuffleboard, e.g. desired and actual values, in quick and reliable way (network tables is probably most stable?)
- assume any distance from *target* requests are 1.2 meters on the x-axis unless otherwise specified
- The following official limelight examples be able to meet the code task requests with slight adaptions
  - Adapt the "Aiming and ranging with Swerve" example from Limelight to my robot
  - Adapt the "Aiming with Servoing" example from Limelight to my robot
  - Adapt the "Localization with MegaTag" but use MegaTag2 if possible
  - Adapt the "Getting in Range" in this scenario, the correct distance *target*
  - Adapt the "Aiming and Ranging Simultaneously" in this scenario with the correct distance *target*
