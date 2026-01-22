Files Created:
File	Purpose
subsystems/vision/VisionTestConstants.java	Centralized constants for all test models
commands/visiontest/RotationalAlignCommand.java	Model A - Rotation only
commands/visiontest/RotationalRangeAlignCommand.java	Model B - Rotation + Range
commands/visiontest/PerpendicularAlignCommand.java	Model C - Perpendicular + Range
commands/visiontest/ColorBlobHuntCommand.java	Model D - Color blob hunt & seek
subsystems/vision/VisionTestDashboard.java	Shuffleboard UI controls
Model Overview:
Model	Description	Limelight Example
A	Rotation only - centers AprilTag on Y-axis	"Aiming with Servoing"
B	Rotation + 1.2m X-axis distance	"Aiming and Ranging Simultaneously"
C	Perpendicular alignment + 1.2m + lateral centering	"Aiming and Ranging with Swerve" + MegaTag2
D	Hunt (search) + Seek (align) with teal color blob	"Getting in Range" with color pipeline
Button Bindings:
Controller:

A: Original AlignToTag (kept intact)
B: Model A (rotation only)
X: Model B (rotation + range)
Y: Model C (perpendicular + range)
Left Bumper: Model D (color hunt)
Right Bumper: Stop all tests
Start: Reset field-centric heading
Shuffleboard: Tab "Vision Tests" with virtual buttons for all models

Telemetry Output:
All models output to both NetworkTables (prefix VisionTest/ModelX/) and AdvantageKit for replay analysis.

Setup Required for Model D (Color Hunt):
Configure Pipeline 1 in Limelight web interface for teal color detection
Set HSV thresholds: approximately H=85-100, S=100-255, V=100-255 (adjust for your lighting)
Calibrate TARGET_AREA_FOR_DISTANCE in VisionTestConstants.ModelD by measuring target area at 1.2m