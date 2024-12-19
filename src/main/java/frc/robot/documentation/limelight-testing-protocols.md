@@ -1,19 +1,19 @@
# Limelight Feedback testing
Create an enum or state machine that returns LED feedback
Create an enum or state machine that returns LED feedback based on Limelight readings of AprilTags 1 through 4

## Level 1 Testing with LED
Status check Level 1; check on AprilTag IDs (1 to 4 or -1),
    Return a specific color for each
## Level 1 Status Check
Status check of AprilTag IDs (1 to 4 or -1),

Status check Level 2; check on AprilTags IDS (1 to 4 or -1) AND check to see if the distance is less than 20 (cm),
    Return a new set of specific colors
## Level 2 Status Check
Status check; check on AprilTags IDS (1 to 4 or -1) AND check to see if the distance is less than 30 (cm),

Status check Level 1;
    If tagID = Target1, make the robot drive left for 10 seconds at 10 percent max speed
    If tagID = Target2, make the robot drive right for 10 seconds at 10 percent max speed
    If tagID = Target3, make the robot drive forward for 10 seconds at 10 percent max speed
    If tagID = Target4, make the robot drive backwards for 10 seconds at 10 percent max speed
## LED Feedback
Return a specific CTRE CANdle color for each AprilTag in status checks. e.g. RED for none or id -1, YELLOW for id 1, GREEN for id 2, BLUE for id 3, PURPLE for id 4

## Level 2 Testing with 
Status check Level 2;
    Same as level 1 driver

## Drivetrain Feedback
Status check; check on AprilTag IDs (1 to 4 or -1),
    If tagID = Target1, make the robot drive left for 10 seconds at 10 percent max speed and return the corresponding apriltag color
    If tagID = Target2, make the robot drive right for 10 seconds at 10 percent max speed and return the corresponding apriltag color
    If tagID = Target3, make the robot drive forward for 10 seconds at 10 percent max speed and return the corresponding apriltag color
    If tagID = Target4, make the robot drive backwards for 10 seconds at 10 percent max speed and return the corresponding apriltag color