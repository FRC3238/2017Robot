package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.*;

/**
 * Created by BUTLEJEF000 on 1/16/2017.
 */
public class Chassis {
    private RobotDrive driveTrain;
    private CANTalon leftLeader, rightLeader;
    private Joystick joy;
    private Timer timer;
    private boolean ShooterDisabled = false;
    private boolean placingInit = false;
    private String state = "default";

    Chassis(CANTalon leftTalonA, CANTalon rightTalonA, Joystick joy) {
        this.joy = joy;

        timer = new Timer();
//        driveTrain = new RobotDrive(leftTalonA, leftTalonB, rightTalonA,
//                rightTalonB);
        this.leftLeader = leftTalonA;
        this.rightLeader = rightTalonA;
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
            if (joy.getRawButton(Constants.Collector.PLACE_GEAR_BUTTON) && !placingInit) {
                placeGear();
                placingInit = true;
            } else {
                placingInit = false;
            }

            double y;
            if (Math.abs(joy.getY()) >= Constants.Chassis.DEADZONE) {
                y = joy.getY() * Constants.Chassis.MOVE_SCALE;
            } else {
                y = 0.0;
            }

            double twist;
            if (Math.abs(joy.getTwist()) >= Constants.Chassis.TWIST_MAX_THRESHOLD) {
                twist = -joy.getTwist();
            } else if (Math.abs(joy.getTwist()) >= Constants.Chassis.TWIST_DEADZONE) {
                if(!ShooterDisabled) {
                    twist = joy.getTwist() * Constants.Chassis.TWIST_SCALE;
                } else {
                    twist = joy.getTwist() * Constants.Chassis.SHOOTER_TWIST_SCALE;
                }
            } else {
                twist = 0.0;
            }
            autoRun(y, twist);
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
            twist = -(Math.pow(Math.abs(joy.getTwist()), 3.8))*(Math.abs(twist)/twist);
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
        }
    }
}

