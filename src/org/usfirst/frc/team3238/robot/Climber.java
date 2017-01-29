package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Joystick;

/**
 * Created by Jefferson on 1/16/2017.
 */
public class Climber
{
    String state;
    CANTalon climbTalonOne, climbTalonTwo;
    Joystick joy;
    
    Climber(CANTalon climbTalonOne, CANTalon climbTalonTwo, Joystick joy)
    {
        this.joy = joy;
        this.climbTalonOne = climbTalonOne;
        this.climbTalonTwo = climbTalonTwo;
        climbTalonOne.set(0.0);
        climbTalonTwo.set(0.0);
        state = "disabled";
    }
    
    public void run()
    {
        if(joy.getRawButton(Constants.Climber.CLIMBER_ACTIVATION_BUTTON))
        {
            state = "pulling";
        } else if(joy.getRawButton(Constants.Climber.CLIMBER_KILL_BUTTON))
        {
            state = "disabled";
        }
        switch(state)
        {
            case "pulling":
                climbTalonOne.set(Constants.Climber.CLIMBER_GO_UP_VALUE);
                climbTalonTwo.set(Constants.Climber.CLIMBER_GO_UP_VALUE);
                break;
            case "disabled":
                climbTalonOne.set(Constants.Climber.CLIMBER_INACTIVE_VALUE);
                climbTalonTwo.set(Constants.Climber.CLIMBER_INACTIVE_VALUE);
                break;
        }
    }
}

