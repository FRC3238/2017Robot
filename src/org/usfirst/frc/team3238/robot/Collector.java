package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Created by aaron on 1/16/2017.
 */
public class Collector
{
    CANTalon leftIntake;
    CANTalon rightIntake;
    CANTalon lift;
    Joystick joy;
    
    Timer timer;
    String state;
    
    public Collector(CANTalon leftIntake, CANTalon rightIntake, CANTalon lift, Joystick joy)
    {
        leftIntake.enableLimitSwitch(false, false);
        
        rightIntake.changeControlMode(CANTalon.TalonControlMode.Follower);
        rightIntake.set(leftIntake.getDeviceID());
        rightIntake.reverseSensor(true);
        
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        this.lift = lift;
        this.joy = joy;
        
        timer = new Timer();
        state = "disabled";
    }
    
    public void init()
    {
        state = "inactive";
    }
    
    public void run()
    {
        if(joy.getRawButton(Constants.Collector.COLLECT_IN_BUTTON))
        {
            setIntake(Constants.Collector.FEED_INTAKE_POWER);
        } else if(joy.getRawButton(Constants.Collector.COLLECT_OUT_BUTTON))
        {
            setIntake(-Constants.Collector.FEED_INTAKE_POWER);
        } else
        {
            setIntake(0.0);
        }
    
        if(joy.getPOV() == Constants.Collector.COLLECT_RAISE_BUTTON)
        {
            lift.set(0.5);
        } else if(joy.getPOV() == Constants.Collector.COLLECT_LOWER_BUTTON)
        {
            lift.set(-0.5);
        } else
        {
            lift.set(0.0);
        }
        
        DriverStation.reportError("Collect Limit: " + leftIntake.isRevLimitSwitchClosed(), false);
        
        
//        switch(state)
//        {
//            case "disabled":
//                break;
//            case "inactive":
//                lift.set(0.0);
//                setIntake(0.0);
//                watchJoy();
//                break;
//            case "collecting ground":
//                setIntake(Constants.Collector.INTAKE_POWER);
//                lift.set(0.0);
//                watchLimit();
//                break;
//            case "collecting feed":
//                setIntake(Constants.Collector.FEED_INTAKE_POWER);
//                lift.set(0.0);
//                watchLimit();
//                break;
//            case "raising":
//                lift.set(0.0);
//                setIntake(Constants.Collector.RAISE_INTAKE_POWER);
//                time(Constants.Collector.RAISE_SECONDS);
//                break;
//        }
    }
    
    private void setIntake(double power)
    {
        leftIntake.set(power);
    }
    
    private void resetTime()
    {
        timer.reset();
        timer.start();
    }
    
    private void time(double seconds)
    {
        if(timer.get() >= seconds)
        {
            state = "inactive";
        }
    }
    
    private void watchJoy()
    {
        if(joy.getRawButton(2))
        {
            state = "collecting ground";
        }
        if(joy.getRawButton(3))
        {
            state = "collecting feed";
        }
    }
}
