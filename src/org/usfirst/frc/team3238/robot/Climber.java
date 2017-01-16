package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Joystick;

/**
 * Created by BUTLEJEF000 on 1/16/2017.
 */
public class Climber
{
    CANTalon climbTalonOne, climbTalonTwo;
    Joystick joy;
    
    Climber(CANTalon climbTalonOne, CANTalon climbTalonTwo)
    {
        
        climbTalonOne.setInverted(false);
        climbTalonTwo.setInverted(true);
        climbTalonOne.set(0.0);
        climbTalonTwo.set(0.0);
    }
    
    public void run()
    {
        if(joy.getRawButton(Constants.Climber.CLIMBER_ACTIVATION_BUTTON))
        {
            climbTalonOne.set(1.0);
            climbTalonTwo.set(1.0);
        }
    }
}

