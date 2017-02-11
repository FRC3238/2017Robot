package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;

/**
 * Controlling class for chassis subsystem
 *
 * @author jefferson, aaron
 */
public class Chassis
{
    private RobotDrive driveTrain;
    private Joystick joy;
    
    /**
     * Sets up talons, initializes subsystem
     *
     * @param leftTalonA  left talon
     * @param leftTalonB  left talon
     * @param rightTalonA right talon
     * @param rightTalonB right talon
     * @param joy         main driver joystick
     */
    Chassis(CANTalon leftTalonA, CANTalon leftTalonB, CANTalon rightTalonA,
            CANTalon rightTalonB, Joystick joy)
    {
        this.joy = joy;
        
        driveTrain = new RobotDrive(leftTalonA, leftTalonB, rightTalonA,
                rightTalonB);
    }
    
    /**
     * Runs drive train with mapping of joystick inputs
     */
    public void run()
    {
        double y;
        if(Math.abs(joy.getY()) >= Constants.Chassis.DEADZONE)
        {
            y = Constants.Chassis.MOVE_SCALE * Math.pow(joy.getY(), 3);
        } else
        {
            y = 0.0;
        }
        
        double twist;
        if(joy.getTwist() > Constants.Chassis.TWIST_DEADZONE)
        {
            twist = -Constants.Chassis.TWIST_SCALE * Math
                    .pow(joy.getTwist(), 2);
        } else if(joy.getTwist() < Constants.Chassis.TWIST_DEADZONE)
        {
            twist = Constants.Chassis.TWIST_SCALE * Math.pow(joy.getTwist(), 2);
        } else
        {
            twist = 0.0;
        }
        autoRun(y, twist);
    }
    
    /**
     * Sets chassis speed
     *
     * @param y     forward speed
     * @param twist twist speed
     */
    public void autoRun(double y, double twist)
    {
        driveTrain.arcadeDrive(y, twist);
    }
}

