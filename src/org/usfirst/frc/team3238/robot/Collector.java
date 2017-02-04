package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

/**
 * Class to control Gear Collector
 *
 * @author aaron
 * @version 1.0
 */
class Collector
{
    private CANTalon leftIntake, rightIntake;
    private CANTalon lift;
    private Joystick joy;
    
    private Timer timer;
    private String state;
    
    Collector(CANTalon leftIntake, CANTalon rightIntake, CANTalon lift,
            Joystick joy)
    {
        leftIntake.enableLimitSwitch(false, false);
        
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        this.lift = lift;
        this.joy = joy;
        
        timer = new Timer();
        state = "inactive";
    }
    
    void init()
    {
        state = "inactive";
    }
    
    void run()
    {
        switch(state)
        {
            case "inactive": // Not moving
                if(joy.getRawButton(11))
                    setIntake(0.5);
                else if(joy.getRawButton(Constants.Collector.COLLECT_IN_POV))
                    setIntake(-Constants.Collector.FEED_INTAKE_POWER);
                else if(joy
                        .getRawButton(Constants.Collector.COLLECT_OUT_POV))
                    setIntake(Constants.Collector.FEED_INTAKE_POWER);
                else
                    setIntake(0.0);
                if(joy.getPOV() == Constants.Collector.COLLECT_RAISE_POV)
                    lift.set(Constants.Collector.RAISE_POWER);
                else if(joy.getPOV()
                        == Constants.Collector.COLLECT_LOWER_POV)
                    lift.set(Constants.Collector.LOWER_POWER);
                else
                    lift.set(0.0);
                watchJoy(Constants.Collector.COLLECT_GROUND_BUTTON,
                        "collecting ground");
                watchJoy(Constants.Collector.COLLECT_FEED_BUTTON,
                        "collecting feed");

                DriverStation.reportWarning("in disabled", false);

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
            case "raise":
//                state = "inactive";
                setIntake(0.0);
                lift.set(Constants.Collector.RAISE_POWER);
                watchUpperLimit("inactive");
                watchJoy(Constants.Collector.COLLECT_GROUND_BUTTON,
                        "collecting ground");
                watchJoy(Constants.Collector.COLLECT_FEED_BUTTON,
                        "collecting feed");
                watchJoy(Constants.Collector.DISABLE_BUTTON, "inactive");
                break;
            case "releasing":
                watchJoy(Constants.Collector.DISABLE_BUTTON, "inactive");
                DriverStation.reportWarning("in Releasing",false);
                setIntake(0.5);
                break;
            case "raising": // Raising lift
//                state = "inactive";
                lift.set(Constants.Collector.RAISE_POWER);
                setIntake(0.0);
                watchUpperLimit("raising done");
                watchJoy(Constants.Collector.DISABLE_BUTTON, "inactive");
                watchJoy(Constants.Collector.COLLECT_GROUND_BUTTON,
                        "collecting ground");
                watchJoy(Constants.Collector.COLLECT_FEED_BUTTON,
                        "collecting feed");
                break;
            case "raising done": // Raising gear
//                state = "disabled";
                lift.set(0.0);
//                setIntake(Constants.Collector.RAISE_INTAKE_POWER);
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
        rightIntake.set(-power);
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
            resetTime();
            this.state = state;
        }
    }
    
    private void watchLimit(String state)
    {
        if(leftIntake.isRevLimitSwitchClosed())
        {
            resetTime();
            this.state = state;
        }
    }
    
    private void watchUpperLimit(String state)
    {
        if(lift.isRevLimitSwitchClosed())
        {
            resetTime();
            this.state = state;
        }
    }
    
    private void watchPOV(int pov, String state)
    {
        if(joy.getPOV() == pov)
        {
            resetTime();
            this.state = state;
        }
    }
}
