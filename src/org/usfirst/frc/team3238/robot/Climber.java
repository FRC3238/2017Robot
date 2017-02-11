package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Joystick;

/**
 * Controlling class for climber subsystem
 *
 * @author jefferson
 */
class Climber
{
    private CANTalon climbTalonOne, climbTalonTwo;
    private Joystick joy;
    
    /**
     * Sets up talon objects, initializes climber
     *
     * @param climbTalonOne climber talon
     * @param climbTalonTwo climber talon
     * @param joy           main driver joystick
     */
    Climber(CANTalon climbTalonOne, CANTalon climbTalonTwo, Joystick joy)
    {
        this.joy = joy;
        this.climbTalonOne = climbTalonOne;
        this.climbTalonTwo = climbTalonTwo;
    }
    
    /**
     * Runs climber motors based on joystick input
     */
    void run()
    {
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
    
    /**
     * Runs both climber motors at specified speed
     *
     * @param power speed
     */
    private void set(double power)
    {
        climbTalonOne.set(power);
        climbTalonTwo.set(power);
    }
}

