package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;

/**
 * Created by BUTLEJEF000 on 1/16/2017.
 */
public class Chassis {

    CANTalon leftTalonA, leftTalonB, rightTalonA, rightTalonB;

    Joystick joy;

    Chassis(CANTalon leftTalonA, CANTalon leftTalonB, CANTalon rightTalonA,
            CANTalon rightTalonB, Joystick joy) {
        this.leftTalonA = leftTalonA;
        this.leftTalonB = leftTalonB;
        this.rightTalonB = rightTalonB;
        this.rightTalonA = rightTalonA;
        this.joy = joy;

    }

    public void run() {
        arcadeDrive(joy.getY(), joy.getTwist());
    }

    public void autoRun(double y, double twist) {
        arcadeDrive(y, twist);

    }

    public void arcadeDrive(double forward, double turn) {
        leftTalonA.set(forward + turn);
        leftTalonB.set(forward + turn);
        rightTalonA.set(forward - turn);
        rightTalonB.set(forward - turn);
    }
}

