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
class Collector {
    private CANTalon leftIntake, rightIntake;
    private CANTalon lift;
    private Joystick joy;

    private Timer timer;
    private String state;

    Collector(CANTalon leftIntake, CANTalon rightIntake, CANTalon lift,
              Joystick joy) {
        leftIntake.enableLimitSwitch(false, false);

        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        this.lift = lift;
        this.joy = joy;

        timer = new Timer();
        state = "inactive";
    }

    void init() {
        state = "inactive";
    }

    void run() {
        switch (state) {
            case "inactive": // Not moving
                manualControls(Constants.Collector.RAISE_POWER, 0.0);

                watchJoy(Constants.Collector.COLLECT_GROUND_BUTTON,
                        "collecting ground");
                break;
            case "collecting ground": // Collectors spinning inward, lift lowered
                manualControls(Constants.Collector.LOWER_POWER, Constants.Collector.INTAKE_POWER);
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
                break;
//            case "raising done": // Raising gear
//                manualControls(Constants.Collector.RAISE_POWER, Constants.Collector.RAISE_INTAKE_POWER);
//                time(Constants.Collector.RAISE_SECONDS, "inactive");
//                watchJoy(Constants.Collector.DISABLE_BUTTON, "inactive");
//                watchJoy(Constants.Collector.COLLECT_GROUND_BUTTON,
//                        "collecting ground");
//                watchJoy(Constants.Collector.COLLECT_FEED_BUTTON,
//                        "collecting feed");
//                break;
            case "manual":
                manualControls(0.0, 0.0);
                watchJoy(Constants.Collector.COLLECT_GROUND_BUTTON,
                        "collecting ground");
                watchJoy(Constants.Collector.RAISE_LIFT_BUTTON, "raising");
                break;

        }

        DriverStation.reportWarning("Collector State: " + state, false);
    }

    private void manualControls(double liftPower, double intakePower) {
        if (joy.getPOV() == Constants.Collector.COLLECT_IN_POV) {
            state = "manual";
            setIntake(-Constants.Collector.FEED_INTAKE_POWER);
        } else if (joy
                .getPOV() == Constants.Collector.COLLECT_OUT_POV) {
            state = "manual";
            setIntake(Constants.Collector.FEED_INTAKE_POWER);
        } else {
            setIntake(intakePower);
        }

        if (joy.getPOV() == Constants.Collector.COLLECT_RAISE_POV) {
            state = "manual";
            lift.set(Constants.Collector.RAISE_POWER);
        } else if (joy.getRawButton(Constants.Collector.COLLECT_LOWER_BUTTON) || joy.getPOV() == Constants.Collector.COLLECT_LOWER_POV) {
            state = "manual";
            lift.set(Constants.Collector.LOWER_POWER);
        } else {
            lift.set(liftPower);
        }
    }

    private void setIntake(double power) {
        leftIntake.set(power);
        rightIntake.set(-power);
    }

    private void resetTime() {
        timer.reset();
        timer.start();
    }

    private void time(double seconds, String state) {
        if (timer.get() >= seconds) {
            resetTime();
            this.state = state;
        }
    }

    private void watchJoy(int button, String state) {
        if (joy.getRawButton(button)) {
            resetTime();
            this.state = state;
        }
    }

    private void watchLimit(String state) {
        if (leftIntake.isRevLimitSwitchClosed()) {
            resetTime();
            this.state = state;
        }
    }

    private void watchUpperLimit(String state) {
        if (lift.isRevLimitSwitchClosed()) {
            resetTime();
            this.state = state;
        }
    }

    private void watchPOV(int pov, String state) {
        if (joy.getPOV() == pov) {
            resetTime();
            this.state = state;
        }
    }
}
