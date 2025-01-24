package frc.robot.subsystems.wrist;

public class WristSubsystem {
          /* Arm has been set to invert so positive values for arm poses */
          public static final double DEG_TO_ARM_NATIVE = 0.93;// -83.7 / 90
          public static final double ARM_NATIVE_TO_DEG = 1 / DEG_TO_ARM_NATIVE;
          public static final double ARM_MAX_DUTY_CYCLE_OUT = 0.6;
          public static final double ARM_SUPPLY_CURRENT_LIMIT = 5;
          public static final double ARM_STATOR_CURRENT_LIMIT = 20;
      
          /* Arm poses */
          public static final int ARM_REV_LIMIT = 0;
          public static final double ARM_HOME_POSE = 0;
          public static final double ARM_LOW_POSE = 10;
          public static final double ARM_MID_POSE = 25.5; // 30 // 20
          public static final double ARM_HIGH_POSE = 55;
          public static final double ARM_AMP_POSE = 90; // 90 -> 85 -> 80 -> 70
          // public static final double ARM_SAFETY_POSE = 27.5;
          public static final int ARM_FWD_LIMIT = 100; // Previous event 91
      
          /* Distances (inches) from front Bumper Speaker Base to front Bumper and arm in canon units
          Pose 0 is at the base, Pose 1 is at the leg of the Stage, i.e. "Safety Pose"
      
          | Pose  | Dist  | Arm |
          |------ |-----  | --- |
          | 0     | 0     | 0   |
          | 1     | 126   | 20  | Stage / Safety Pose
          | 2     | 150   | 00  |
          | 3     | 180   | 00  |
          | 4     | 200   | 00  |
          
          */
      
      
          public static final int ARM_MAX_ACCEL = 100;
          public static final int ARM_MAX_VEL = 200;
          public static final int ARM_JERK = 0;
      
          public static final int ARM_STATOR_LIMIT = 15;
          public static final int ARM_SUPPLY_LIMIT = 15;
          public static final double ARM_ROTATION_LIMIT_NATIVE = 0;
      
          public static final double ARM_MANUAL_POWER = 6; // for Voltage control
      
}
