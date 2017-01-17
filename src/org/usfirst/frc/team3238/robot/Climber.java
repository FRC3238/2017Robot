package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Joystick;

/**
 * Created by Jefferson on 1/16/2017.
 */
public class Climber
{
    CANTalon climbTalonOne, climbTalonTwo;
    Joystick joy;
    
    Climber(CANTalon climbTalonOne, CANTalon climbTalonTwo, Joystick joy)
    {
        this.joy = joy;
        this.climbTalonOne = climbTalonOne;
        this.climbTalonTwo = climbTalonTwo;
        climbTalonOne.setInverted(false);
        climbTalonTwo.setInverted(true);
        climbTalonOne.set(0.0);
        climbTalonTwo.set(0.0);
    }
    
    public void run()
    {
        if(joy.getRawButton(Constants.Climber.CLIMBER_ACTIVATION_BUTTON))
        {
            climbTalonOne.set(Constants.Climber.CLIMBER_GO_UP_VALUE);
            climbTalonTwo.set(Constants.Climber.CLIMBER_GO_UP_VALUE);
        } else if(joy.getRawButton(Constants.Climber.CLIMBER_GO_DOWN_BUTTON))
        {
            climbTalonTwo.set(Constants.Climber.CLIMBER_GO_DOWN_VALUE);
            climbTalonOne.set(Constants.Climber.CLIMBER_GO_DOWN_VALUE);
        } else
        {
            climbTalonOne.set(Constants.Climber.CLIMBER_INACTIVE_VALUE);
            climbTalonTwo.set(Constants.Climber.CLIMBER_INACTIVE_VALUE);
        }
    }
}

