package org.usfirst.frc.team3238.robot;


public class Constants
{
    public class Chassis
    {
        public static final int DRIVE_TALON_ID_RIGHT_A = 1;
        public static final int DRIVE_TALON_ID_RIGHT_B = 2;
        public static final int DRIVE_TALON_ID_LEFT_A = 3;
        public static final int DRIVE_TALON_ID_LEFT_B = 4;
        public static final double DEADZONE = 0.25;
        public static final double TWIST_DEADZONE = 0.3;
        public static final double TWIST_MAX_THRESHOLD = 0.75;
        
        public static final double TWIST_SCALE = -0.6;
        public static final double SHOOTER_TWIST_SCALE = -0.9;
        public static final double MOVE_SCALE = 0.8;

        public static final double PLACING_GEAR_SPEED = 0.5;
        public static final double PLACING_GEAR_TIME = 0.1;
    }
    
    public class Robot
    {
        public static final int MAIN_JOYSTICK_PORT = 0;
        public static final int OVERRIDE_JOYSTICK_PORT = 1;
    }
    
    public class Climber
    {
        public static final int CLIMB_TALON_ONE_ID = 8;
        public static final int CLIMB_TALON_TWO_ID = 9;
        
        public static final int CLIMBER_UP_BUTTON = 6;
        public static final int CLIMBER_SLOW_BUTTON = 4;
        
        public static final double CLIMBER_GO_UP_VALUE = -1.0;
        public static final double CLIMBER_SLOW_VALUE = -0.9;
    }
  
    public class Collector
    {
        public static final int LEFT_TALON_ID = 6;
        public static final int RIGHT_TALON_ID = 7;
        public static final int LIFT_TALON_ID = 5;

        public static final int COLLECT_RAISE_POV = 180;
        public static final int COLLECT_LOWER_POV = 0;
        public static final int COLLECT_IN_POV = 270;
        public static final int COLLECT_OUT_POV = 90;
        public static final int COLLECT_RAISE_IN_POV = 225;
        public static final int COLLECT_RAISE_OUT_POV = 135;
        public static final int COLLECT_LOWER_IN_POV = 315;
        public static final int COLLECT_LOWER_OUT_POV = 45;
        public static final int COLLECT_GROUND_BUTTON = 2;
        public static final int PLACE_GEAR_BUTTON = 1;
        public static final int DISABLE_BUTTON = 3;

        public static final double INTAKE_POWER = -0.65;
        public static final double RAISE_POWER = 0.55;
        public static final double LOWER_POWER = -0.55;
        public static final double PLACE_GEAR_POWER = -0.7;
        public static final int ENCODER_GEAR_BOTTOM_LIMIT = -600;
        public static final int CURRENT_THRESHOLD = 25;
        public static final int CURRENT_CYCLES_INCREASE = 10;
        public static final int CURRENT_CYCLES_THRESH = 30;
    }
    
    public class Shooter
    {
        public static final int DISTANCE_PORT = 0;
        public static final int SHOOT_RPM = 10070;
        public static final int SHOOT_PREP_BUTTON = 7;
        public static final int SHOOT_BUTTON = 8;
        public static final int DISABLE_BUTTON = 3;

        public static final int SHOOTER_TALON_ID = 10;
        public static final int AGITATOR_TALON_ID = 11;

        public static final int CODES_PER_REV = 360;
        public static final double AGITATOR_SPEED = 0.55;
        public static final int ALLOWED_RPM_ERROR = 10;
    }
    public class Autonomous {
        public static final String DISABLED = "Disabled";
        public static final String BOILER_GEAR = "Boiler Gear";
        public static final String BOILER_SHOOT = "Boiler Gear + Shoot";
        public static final String CENTER_GEAR = "Center Gear";
        public static final String CENTER_SHOOT = "Center Gear + Shoot";
        public static final String RETRIEVAL_GEAR = "Retrieval Gear";
        public static final String RETRIEVAL_RUN = "Retrieval Gear + Run";
        public static final String MLG_HOPPER = "MLG HOPPER!!!!";
        public static final String SHOOT_THEN_GEAR = "Shoot then place";
        public static final String BOILER_RUN = "Boiler Gear + Run";
    }
}
