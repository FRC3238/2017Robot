package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

/**
 * Created by aaron on 1/16/2017.
 */
public class Collector
{
    CANTalon leftIntake;
    CANTalon rightIntake;
    CANTalon lift;
    DigitalInput limit;
    Joystick joy;
    
    Timer timer;
    String state;
    
    public Collector(CANTalon leftIntake, CANTalon rightIntake, CANTalon lift,
            DigitalInput limit, Joystick joy)
    {
        lift.changeControlMode(CANTalon.TalonControlMode.Position);
        lift.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        lift.setAllowableClosedLoopErr(Constants.Collector.ALLOWABLE_ERROR);
        lift.setP(Constants.Collector.LIFT_P_CONST);
        lift.setI(Constants.Collector.LIFT_I_CONST);
        lift.setD(Constants.Collector.LIFT_D_CONST);
        
        rightIntake.changeControlMode(CANTalon.TalonControlMode.Follower);
        rightIntake.set(leftIntake.getDeviceID());
        rightIntake.reverseSensor(true);
        
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        this.lift = lift;
        this.limit = limit;
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
        switch(state)
        {
            case "disabled":
                break;
            case "inactive":
                setLift(90);
                setIntake(0.0);
                watchJoy();
                break;
            case "collecting ground":
                setIntake(Constants.Collector.INTAKE_POWER);
                setLift(0);
                watchLimit();
                break;
            case "collecting feed":
                setIntake(Constants.Collector.FEED_INTAKE_POWER);
                setLift(90);
                watchLimit();
                break;
            case "raising":
                setLift(90);
                setIntake(Constants.Collector.RAISE_INTAKE_POWER);
                time(Constants.Collector.RAISE_SECONDS);
                break;
        }
    }
    
    private void setLift(int degrees)
    {
        lift.set(degrees * Constants.Collector.ENCODER_COUNT_PER_DEGREE);
    }
    
    private void setIntake(double power)
    {
        leftIntake.set(power);
    }
    
    private void watchLimit()
    {
        if(limit.get())
        {
            state = "raising";
            resetTime();
        }
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
