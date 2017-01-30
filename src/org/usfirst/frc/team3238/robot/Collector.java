package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import com.sun.xml.internal.bind.v2.runtime.reflect.opt.Const;
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
    
    public Collector(CANTalon leftIntake, CANTalon rightIntake, CANTalon lift,
            Joystick joy)
    {
        leftIntake.enableLimitSwitch(false, false);
        
        rightIntake.changeControlMode(CANTalon.TalonControlMode.Follower);
        rightIntake.setInverted(true);
        rightIntake.set(leftIntake.getDeviceID());
        
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        this.lift = lift;
        this.joy = joy;
        
        timer = new Timer();
        state = "inactive";
    }
    
    public void init()
    {
        state = "inactive";
    }
    
    public void run()
    {
        switch(state)
        {
            case "inactive": // Not moving
                if(joy.getRawButton(Constants.Collector.COLLECT_IN_BUTTON))
                    setIntake(Constants.Collector.FEED_INTAKE_POWER);
                else if(joy
                        .getRawButton(Constants.Collector.COLLECT_OUT_BUTTON))
                    setIntake(-Constants.Collector.FEED_INTAKE_POWER);
                else
                    setIntake(0.0);
                if(joy.getPOV() == Constants.Collector.COLLECT_RAISE_BUTTON)
                    lift.set(0.5);
                else if(joy.getPOV()
                        == Constants.Collector.COLLECT_LOWER_BUTTON)
                    lift.set(-0.5);
                else
                    lift.set(0.0);
                watchJoy(Constants.Collector.COLLECT_GROUND_BUTTON,
                        "collecting ground");
                watchJoy(Constants.Collector.COLLECT_FEED_BUTTON,
                        "collecting feed");
                break;
            case "collecting ground": // Collectors spinning inward, lift lowered
                setIntake(Constants.Collector.INTAKE_POWER);
                lift.set(Constants.Collector.LOWER_POWER);
                watchLimit("raising");
                watchJoy(Constants.Collector.COLLECT_FEED_BUTTON,
                        "collecting feed");
                watchJoy(Constants.Collector.DISABLE_BUTTON, "inactive");
                break;
            case "collecting feed": // Collectors spinning slowly inward, lift raised
                setIntake(Constants.Collector.FEED_INTAKE_POWER);
                lift.set(Constants.Collector.RAISE_POWER);
                watchLimit("raising done");
                watchJoy(Constants.Collector.COLLECT_GROUND_BUTTON,
                        "collecting ground");
                watchJoy(Constants.Collector.DISABLE_BUTTON, "inactive");
                break;
            case "raising": // Raising lift
                lift.set(Constants.Collector.RAISE_POWER);
                setIntake(0.0);
                watchLimit("raising done");
                watchJoy(Constants.Collector.DISABLE_BUTTON, "inactive");
                watchJoy(Constants.Collector.COLLECT_GROUND_BUTTON,
                        "collecting ground");
                watchJoy(Constants.Collector.COLLECT_FEED_BUTTON,
                        "collecting feed");
                break;
            case "raising done": // Raising gear
                lift.set(0.0);
                setIntake(Constants.Collector.RAISE_INTAKE_POWER);
                time(Constants.Collector.RAISE_SECONDS, "inactive");
                watchJoy(Constants.Collector.DISABLE_BUTTON, "inactive");
                watchJoy(Constants.Collector.COLLECT_GROUND_BUTTON,
                        "collecting ground");
                watchJoy(Constants.Collector.COLLECT_FEED_BUTTON,
                        "collecting feed");
                break;
        }
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
    
    private void time(double seconds, String state)
    {
        if(timer.get() >= seconds)
        {
            resetTime();
            this.state = state;
        }
    }
    
    private void watchJoy(int button, String state)
    {
        if(joy.getRawButton(button))
        {
            this.state = state;
        }
    }
    
    private void watchLimit(String state)
    {
        if(leftIntake.isRevLimitSwitchClosed())
        {
            this.state = state;
        }
    }
    
    private void watchPOV(int pov, String state)
    {
        if(joy.getPOV() == pov)
        {
            this.state = state;
        }
    }
}
