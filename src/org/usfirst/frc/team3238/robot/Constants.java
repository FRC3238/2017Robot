package org.usfirst.frc.team3238.robot;

import edu.wpi.first.wpilibj.Joystick;

import java.security.PublicKey;

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
}
