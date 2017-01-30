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
        public static final double DEADZONE = 0.25;
        public static final double TWIST_DEADZONE = 0.4;
        
        public static final double TWIST_SCALE = 1.0;
        public static final double MOVE_SCALE = 1.0;
    }
    
    public class Robot
    {
        public static final int MAIN_JOYSTICK_PORT = 0;
    }
    

    public class Climber
    {
        public static final int CLIMB_TALON_ONE_ID = 8;
        public static final int CLIMB_TALON_TWO_ID = 9;
        
        public static final int CLIMBER_ACTIVATION_BUTTON = 6;
        public static final int CLIMBER_GO_DOWN_BUTTON = 4;
        public static final int CLIMBER_KILL_BUTTON = 1;
        
        public static final double CLIMBER_GO_UP_VALUE = -0.75;
        public static final double CLIMBER_GO_DOWN_VALUE = 0.5;
        public static final int CLIMBER_INACTIVE_VALUE = 0;
    }
  
    public class Collector
    {
        public static final int LEFT_TALON_ID = 6;
        public static final int RIGHT_TALON_ID = 7;
        public static final int LIFT_TALON_ID = 5;
        
        public static final int COLLECT_RAISE_BUTTON = 0;
        public static final int COLLECT_LOWER_BUTTON = 180;
        public static final int COLLECT_IN_BUTTON = 3;
        public static final int COLLECT_OUT_BUTTON = 5;
        public static final int COLLECT_GROUND_BUTTON = 2;
        public static final int COLLECT_FEED_BUTTON = 3;
        public static final int DISABLE_BUTTON = 11;
        
        public static final double INTAKE_POWER = 0.6;
        public static final double FEED_INTAKE_POWER = 0.3;
        public static final double RAISE_INTAKE_POWER = -0.4;
        public static final double RAISE_POWER = 0.5;
        public static final double LOWER_POWER = -0.5;
        
        public static final double RAISE_SECONDS = 1.0;
    }
}
