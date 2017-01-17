package org.usfirst.frc.team3238.robot;

/**
 * Created by Jefferson on 1/16/2017.
 */
public class Constants
{
    public class Chassis
    {
        public static final int DRIVE_TALON_ID_RIGHT_A = 1;
        public static final int DRIVE_TALON_ID_RIGHT_B = 2;
        public static final int DRIVE_TALON_ID_LEFT_A = 3;
        public static final int DRIVE_TALON_ID_LEFT_B = 4;
    }
    
    public class Robot
    {
        public static final int MAIN_JOYSTICK_PORT = 0;
    }
    

    public class Climber
    {
        public static final int CLIMB_TALON_ONE_ID = 1;
        public static final int CLIMB_TALON_TWO_ID = 2;
        
        public static final int CLIMBER_ACTIVATION_BUTTON = 1;
        public static final int CLIMBER_GO_DOWN_BUTTON = 2;
        
        public static final int CLIMBER_GO_UP_VALUE = 1;
        public static final int CLIMBER_GO_DOWN_VALUE = -1;
        public static final int CLIMBER_INACTIVE_VALUE = 0;
    }
  
    public class Collector
    {
        public static final int LEFT_TALON_ID = 5;
        public static final int RIGHT_TALON_ID = 6;
        public static final int LIFT_TALON_ID = 7;
        public static final int LIMIT_CHANNEL = 0;
        
        public static final double ENCODER_COUNT_PER_DEGREE = 1;
        public static final int ALLOWABLE_ERROR = 5;
        public static final double LIFT_P_CONST = 0.1;
        public static final double LIFT_I_CONST = 0.0;
        public static final double LIFT_D_CONST = 0.0;
        
        public static final double INTAKE_POWER = 0.6;
        public static final double FEED_INTAKE_POWER = 0.3;
        public static final double RAISE_INTAKE_POWER = -0.4;
        public static final double RAISE_SECONDS = 1.0;
    }
}
