# Limelight Feedback testing
Create an enum or state machine that returns LED feedback

## Level 1 Testing with LED
Status check Level 1; check on AprilTag IDs (1 to 4 or -1),
    Return a specific color for each

Status check Level 2; check on AprilTags IDS (1 to 4 or -1) AND check to see if the distance is less than 20 (cm),
    Return a new set of specific colors

Status check Level 1;
    If tagID = Target1, make the robot drive left for 10 seconds at 10 percent max speed
    If tagID = Target2, make the robot drive right for 10 seconds at 10 percent max speed
    If tagID = Target3, make the robot drive forward for 10 seconds at 10 percent max speed
    If tagID = Target4, make the robot drive backwards for 10 seconds at 10 percent max speed

## Level 2 Testing with 
Status check Level 2;
    Same as level 1 driver