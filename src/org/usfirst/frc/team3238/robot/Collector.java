package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class to control Gear Collector
 *
 * @author aaron
 */
class Collector
{
    private CANTalon leftIntake, rightIntake;
    private CANTalon lift;
    private Joystick joy;
    
    private Timer timer;
    private String state;
    
    /**
     * Sets up necessary objects, initializes class.
     *
     * @param leftIntake  left collector talon
     * @param rightIntake right collector talon
     * @param lift        lift talon
     * @param joy         main driver joystick
     */
    Collector(CANTalon leftIntake, CANTalon rightIntake, CANTalon lift,
            Joystick joy)
    {
        leftIntake.enableLimitSwitch(false, false);
        
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        this.lift = lift;
        this.joy = joy;
        
        timer = new Timer();
        init();
    }
    
    /**
     * Initializes collector, sets state to inactive
     */
    void init()
    {
        state = "inactive";
    }
    
    /**
     * Runs collector for teleop control
     */
    void run()
    {
        switch(state)
        {
            case "inactive": // Not moving
                manualControls(Constants.Collector.RAISE_POWER, 0.0);
                
                watchJoy(Constants.Collector.COLLECT_GROUND_BUTTON,
                        "collecting ground");
                break;
            case "collecting ground": // Collectors spinning inward, lift lowered
                manualControls(Constants.Collector.LOWER_POWER,
                        Constants.Collector.INTAKE_POWER);
                watchLimit("raising");
                watchJoy(Constants.Collector.DISABLE_BUTTON, "inactive");
                watchJoy(Constants.Collector.RAISE_LIFT_BUTTON, "raising");
                break;
            case "raising": // Raising lift
                manualControls(Constants.Collector.RAISE_POWER, 0.0);
                
                watchUpperLimit("inactive");
                //                watchUpperLimit("raising done");
                watchJoy(Constants.Collector.DISABLE_BUTTON, "inactive");
                watchJoy(Constants.Collector.COLLECT_GROUND_BUTTON,
                        "collecting ground");
            case "manual":
                manualControls(0.0, 0.0);
                watchJoy(Constants.Collector.COLLECT_GROUND_BUTTON,
                        "collecting ground");
                watchJoy(Constants.Collector.RAISE_LIFT_BUTTON, "raising");
                break;
        }
        
        SmartDashboard.putString("Collector state", state);
    }
    
    /**
     * Drives lift and intake with allowances for manual controlling. If manual
     * controls are used, switches to manual state.
     *
     * @param liftPower   speed of lift motor
     * @param intakePower speed of intake motors
     */
    private void manualControls(double liftPower, double intakePower)
    {
        if(joy.getPOV() == Constants.Collector.COLLECT_IN_POV)
        {
            state = "manual";
            setIntake(-Constants.Collector.FEED_INTAKE_POWER);
        } else if(joy.getPOV() == Constants.Collector.COLLECT_OUT_POV)
        {
            state = "manual";
            setIntake(Constants.Collector.FEED_INTAKE_POWER);
        } else
        {
            setIntake(intakePower);
        }
        
        if(joy.getPOV() == Constants.Collector.COLLECT_RAISE_POV)
        {
            state = "manual";
            lift.set(Constants.Collector.RAISE_POWER);
        } else if(joy.getRawButton(Constants.Collector.COLLECT_LOWER_BUTTON)
                || joy.getPOV() == Constants.Collector.COLLECT_LOWER_POV)
        {
            state = "manual";
            lift.set(Constants.Collector.LOWER_POWER);
        } else
        {
            lift.set(liftPower);
        }
    }
    
    /**
     * Sets intake speed, accounting for reversed motors
     *
     * @param power speed
     */
    private void setIntake(double power)
    {
        leftIntake.set(power);
        rightIntake.set(-power);
    }
    
    /**
     * Resets and starts the timer
     */
    private void resetTime()
    {
        timer.reset();
        timer.start();
    }
    
    /**
     * Watches timer, if time is over seconds, switches to state
     *
     * @param seconds time
     * @param state   next state
     */
    private void time(double seconds, String state)
    {
        if(timer.get() >= seconds)
        {
            resetTime();
            this.state = state;
        }
    }
    
    /**
     * Watches joystick button, if button is pressed, switches to state
     *
     * @param button joystick button number
     * @param state  next state
     */
    private void watchJoy(int button, String state)
    {
        if(joy.getRawButton(button))
        {
            resetTime();
            this.state = state;
        }
    }
    
    /**
     * Watches intake limit switch, if switch pressed, switches to state
     *
     * @param state next state
     */
    private void watchLimit(String state)
    {
        if(leftIntake.isRevLimitSwitchClosed())
        {
            resetTime();
            this.state = state;
        }
    }
    
    /**
     * Watches upper lift limit switch, if switch pressed, switches to state
     *
     * @param state next state
     */
    private void watchUpperLimit(String state)
    {
        if(lift.isRevLimitSwitchClosed())
        {
            resetTime();
            this.state = state;
        }
    }
    
    /**
     * Watches joystick pov, if direction equals pov, switches to state
     *
     * @param pov   direction, in degrees, of pov to watch
     * @param state next state
     */
    private void watchPOV(int pov, String state)
    {
        if(joy.getPOV() == pov)
        {
            resetTime();
            this.state = state;
        }
    }
}
