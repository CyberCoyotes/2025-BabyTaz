Create a new pipeline (1) for note detection
Set these values:

HSV Threshold: Tune for orange color (typical starting values)

Hue: 10-30
Saturation: 150-255
Value: 150-255


Contour filtering:

Area: 0.1% to 100%
Fullness: 50% to 100%


Enable Smart Speckle Rejection
Set Contour Sort Mode to "Largest"



Key points to tune:

AREA_TO_DISTANCE constants (A and B) - calibrate these by measuring actual distances
NOTE_COLLECTION_DISTANCE - adjust based on your intake position
PID constants (ROTATION_KP and FORWARD_KP)
MAX_ROTATION_SPEED and MAX_FORWARD_SPEED for safe operation
HSV values in Limelight config for reliable note detection

