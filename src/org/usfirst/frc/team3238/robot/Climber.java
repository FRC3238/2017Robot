package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Joystick;

/**
 * Created by Jefferson on 1/16/2017.
 */
class Climber
{
    private CANTalon climbTalonOne, climbTalonTwo;
    private Joystick joy;
    
    Climber(CANTalon climbTalonOne, CANTalon climbTalonTwo, Joystick joy)
    {
        this.joy = joy;
        this.climbTalonOne = climbTalonOne;
        this.climbTalonTwo = climbTalonTwo;
        climbTalonOne.set(0.0);
        climbTalonTwo.set(0.0);
    }
    
    void run()
    {
        climbTalonOne.enableBrakeMode(true);
        climbTalonTwo.enableBrakeMode(true);
        if(joy.getRawButton(Constants.Climber.CLIMBER_UP_BUTTON))
        {
            set(Constants.Climber.CLIMBER_GO_UP_VALUE);
        } else if(joy.getRawButton(Constants.Climber.CLIMBER_DOWN_BUTTON))
        {
            set(Constants.Climber.CLIMBER_GO_DOWN_VALUE);
        } else
        {
            set(0.0);
        }
    }
    
    private void set(double power)
    {
        climbTalonOne.set(power);
        climbTalonTwo.set(power);
    }
    
    void disabled()
    {
        climbTalonOne.enableBrakeMode(false);
        climbTalonTwo.enableBrakeMode(false);
    }
}

