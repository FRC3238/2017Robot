package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class to control Gear Collector
 *
 * @author aaron
 */
class Collector {
    private CANTalon leftIntake, rightIntake;
    private CANTalon lift;
    private Joystick joy;

    private Timer timer;
    private String state;

    private int currentCounter = 0;
    private double currentMult = 1.0;

    Collector(CANTalon leftIntake, CANTalon rightIntake, CANTalon lift,
              Joystick joy) {
        leftIntake.enableLimitSwitch(false, true);
        lift.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        lift.configEncoderCodesPerRev(1044);
        lift.enableZeroSensorPositionOnReverseLimit(true);

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

    public void placeGear() {
        state = "placing";
    }

    void run() {

        DriverStation.reportWarning(state, false);
        switch (state) {
            case "inactive": // Not moving
                manualControls(Constants.Collector.RAISE_POWER, 0.0);

                watchJoy(Constants.Collector.COLLECT_GROUND_BUTTON,
                        "collecting ground");
                watchJoy(Constants.Collector.PLACE_GEAR_BUTTON, "placing");
                DriverStation.reportWarning(state, false);
                break;
            case "collecting ground": // Collectors spinning inward, lift lowered
                manualControls(Constants.Collector.LOWER_POWER,
                        Constants.Collector.INTAKE_POWER);
                watchLimit();
                watchJoy(Constants.Collector.DISABLE_BUTTON, "inactive");
                watchCurrent();
                break;
            case "placing":
                manualControls(Constants.Collector.PLACE_GEAR_POWER, 0.0);
                watchJoy(Constants.Collector.DISABLE_BUTTON, "inactive");
                watchJoy(Constants.Collector.COLLECT_GROUND_BUTTON,
                        "collecting ground");
                watchEncoder();
                break;
            case "manual":
                manualControls(0.0, 0.0);
                watchJoy(Constants.Collector.COLLECT_GROUND_BUTTON,
                        "collecting ground");
                watchJoy(Constants.Collector.DISABLE_BUTTON, "inactive");
                watchJoy(Constants.Collector.PLACE_GEAR_BUTTON, "placing");
                break;
        }

        SmartDashboard.putString("Collector state", state);
        SmartDashboard.putNumber("Collector encoder", lift.getEncPosition());
        if (currentCounter > SmartDashboard.getNumber("Current count", 0)) {
            SmartDashboard.putNumber("Current count", currentCounter);
        }
    }

    private void manualControls(double liftPower, double intakePower) {
        if (joy.getPOV() == Constants.Collector.COLLECT_IN_POV) {
            state = "manual";
            setIntake(-Constants.Collector.INTAKE_POWER);
            lift.set(0.0);
        } else if (joy
                .getPOV() == Constants.Collector.COLLECT_OUT_POV) {
            state = "manual";
            setIntake(Constants.Collector.INTAKE_POWER);
            lift.set(0.0);
        } else if (joy.getPOV() == Constants.Collector.COLLECT_RAISE_POV) {
            state = "manual";
            lift.set(Constants.Collector.RAISE_POWER);
            setIntake(0.0);
        } else if (joy.getPOV()
                == Constants.Collector.COLLECT_LOWER_POV) {
            state = "manual";
            lift.set(Constants.Collector.LOWER_POWER);
            setIntake(0.0);
        } else if (joy.getPOV()
                == Constants.Collector.COLLECT_RAISE_IN_POV) {
            state = "manual";
            lift.set(Constants.Collector.RAISE_POWER);
            setIntake(-Constants.Collector.INTAKE_POWER);
        } else if (joy.getPOV()
                == Constants.Collector.COLLECT_RAISE_OUT_POV) {
            state = "manual";
            lift.set(Constants.Collector.RAISE_POWER);
            setIntake(Constants.Collector.INTAKE_POWER);
        } else if (joy.getPOV()
                == Constants.Collector.COLLECT_LOWER_IN_POV) {
            state = "manual";
            lift.set(Constants.Collector.LOWER_POWER);
            setIntake(-Constants.Collector.INTAKE_POWER);
        } else if (joy.getPOV()
                == Constants.Collector.COLLECT_LOWER_OUT_POV) {
            state = "manual";
            lift.set(Constants.Collector.LOWER_POWER);
            setIntake(Constants.Collector.INTAKE_POWER);
        } else {
            lift.set(liftPower);
            setIntake(intakePower);
        }
    }

    private void watchEncoder() {
        if (lift.getEncPosition() < Constants.Collector.ENCODER_GEAR_BOTTOM_LIMIT) {
            this.state = "inactive";
        }
    }

    private void setIntake(double power) {
        leftIntake.set(power);
        rightIntake.set(-power);
    }

    private void watchJoy(int button, String state) {
        if (joy.getRawButton(button)) {
            this.state = state;
        }
    }

    private void watchLimit() {
        if (leftIntake.isRevLimitSwitchClosed()) {
            this.state = "inactive";
        }
    }

    private void watchCurrent() {
        if (leftIntake.getOutputCurrent() > Constants.Collector.CURRENT_THRESHOLD || rightIntake.getOutputCurrent() > Constants.Collector.CURRENT_THRESHOLD) {
            currentCounter++;
        } else {
            currentCounter = 0;
        }
        if (currentCounter > Constants.Collector.CURRENT_CYCLES_THRESH) {
            state = "inactive";
            DriverStation.reportError("COLLECTOR INTAKE IS STUCK!!!!", false);
        } else if (currentCounter > Constants.Collector.CURRENT_CYCLES_INCREASE) {
            currentMult = 1.2;
            DriverStation.reportWarning("COLLECTOR INTAKE MIGHT BE STUCK!!!", false);
        }
    }
}
