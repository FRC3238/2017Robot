package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Created by BUTLEJEF000 on 1/16/2017.
 */
public class Chassis implements PIDOutput, PIDController.Tolerance {
    private AHRS navX;
    private PIDController pid;
    private RobotDrive driveTrain;
    private CANTalon leftLeader, rightLeader;
    private Joystick joy;
    private Timer timer;
    private boolean ShooterDisabled = false;
    private boolean placingInit = false;
    private String state = "default";

    private double rotateRate;
    private int onTargetCounter = 0;

    private static final double P_CONST = 0.045;
    private static final double I_CONST = 0.001;
    private static final double D_CONST = 0.05;

    private static final double TOLERANCE_DEGREES = 2.0f;

    Chassis(CANTalon leftTalonA, CANTalon rightTalonA, Joystick joy, AHRS navX) {
        this.navX = navX;
        this.joy = joy;

        timer = new Timer();
//        driveTrain = new RobotDrive(leftTalonA, leftTalonB, rightTalonA,
//                rightTalonB);
        this.leftLeader = leftTalonA;
        this.rightLeader = rightTalonA;

        pid = new PIDController(P_CONST, I_CONST, D_CONST, navX, this);
        pid.setInputRange(-180.0f, 180.0f);
        pid.setOutputRange(-0.7, 0.7);
//        pid.setAbsoluteTolerance(TOLERANCE_DEGREES);
        pid.setTolerance(this);
        pid.setContinuous(true);

        LiveWindow.addActuator("DriveTrain", "TurnController", pid);
    }

    public void printAngle() {
        SmartDashboard.putNumber("Angle", navX.getYaw());
        SmartDashboard.putString("Chassis state", state);
    }

    public void rotateToAngle(double degrees) {
        pid.setSetpoint(degrees);
        pid.enable();
        state = "turning";
    }

    public void init() {
        pid.disable();
        state = "default";
    }

    public void placeGear() {
        timer.reset();
        timer.start();
        state = "placing";
    }

    public void disable() {
        leftLeader.set(0);
        rightLeader.set(0);
    }

    public void run() {
        DriverStation.reportError("LeftENC: " + leftLeader.getEncPosition() + "\nRightENC: " + rightLeader.getEncPosition(), false);
        double y;
        double twist;

        if (joy.getRawButton(Constants.Collector.PLACE_GEAR_BUTTON) && !placingInit) {
            placeGear();
            placingInit = true;
        } else {
            placingInit = false;
        }

        if (Math.abs(joy.getY()) >= Constants.Chassis.DEADZONE) {
            y = joy.getY() * Constants.Chassis.MOVE_SCALE;
        } else {
            y = 0.0;
        }
        if (Math.abs(joy.getTwist()) >= Constants.Chassis.TWIST_MAX_THRESHOLD) {
            twist = -joy.getTwist();
        } else if (Math.abs(joy.getTwist()) >= Constants.Chassis.TWIST_DEADZONE) {
            if (!ShooterDisabled) {
                twist = joy.getTwist() * Constants.Chassis.TWIST_SCALE;
            } else {
                twist = joy.getTwist() * Constants.Chassis.SHOOTER_TWIST_SCALE;
            }
        } else {
            twist = 0.0;
        }
        autoRun(y, twist);
    }
    public void reportEncoders() {
        DriverStation.reportError("LeftEncoder: " + leftLeader.getEncPosition() +"\n" +
                "RightEncoder: " + rightLeader.getEncPosition(), false);
    }
    public void proRun() {
        double y;
        if (joy.getRawButton(Constants.Collector.PLACE_GEAR_BUTTON) && !placingInit) {
            placeGear();
            placingInit = true;
        } else {
            placingInit = false;
        }
        if (Math.abs(joy.getY()) >= Constants.Chassis.DEADZONE) {
            y = joy.getY() * Constants.Chassis.MOVE_SCALE;
        } else {
            y = 0.0;
        }
        double twist;

        if (Math.abs(joy.getTwist()) >= Constants.Chassis.TWIST_DEADZONE) {
            twist = joy.getTwist();
            twist = -(Math.pow(Math.abs(joy.getTwist()), 3.8)) * (Math.abs(twist) / twist);
        } else {
            twist = 0.0;
        }
        autoRun(y, twist);
    }

    public void setShooterDisabled(boolean shooterDisabled) {
        this.ShooterDisabled = shooterDisabled;
    }

    public void autoRun(double y, double twist) {
//        driveTrain.arcadeDrive(y, twist);
        switch (state) {
            case "default":
                leftLeader.set(y - twist);
                rightLeader.set(-y - twist);
                break;
            case "placing":
                DriverStation.reportError("Time: " + timer.get(), false);
                leftLeader.set(Constants.Chassis.PLACING_GEAR_SPEED);
                rightLeader.set(-Constants.Chassis.PLACING_GEAR_SPEED);

                if (timer.get() > Constants.Chassis.PLACING_GEAR_TIME) {
                    state = "default";
                }
                break;
            case "turning":
                DriverStation.reportError("Turning to angle " + pid.getSetpoint() + " - " + navX.getAngle(), false);
                SmartDashboard.putNumber("PID Error", pid.getError());
                leftLeader.set(y + rotateRate);
                rightLeader.set(-y + rotateRate);

                if (pid.onTarget()) {
                    onTargetCounter++;
                } else {
                    onTargetCounter = 0;
                }

                if (onTargetCounter > 7) {
                    pid.disable();
                    state = "default";
                }
                break;
        }
    }

    @Override
    public void pidWrite(double output) {
        rotateRate = output;
    }

    public void resetNavX() {
        navX.reset();
        state = "default";
        pid.disable();
    }

    @Override
    public boolean onTarget() {
        return Math.abs(pid.getError()) < 2.0;
    }
}

