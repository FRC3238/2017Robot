package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Created by Team 3238 on 2/18/2017.
 */
public class Shooter {
    private CANTalon agitator, shooter;
    private Joystick joy;

    private boolean ready = false;

    Shooter(CANTalon agitator, CANTalon shooter, Joystick joy) {
        this.joy = joy;
        this.agitator = agitator;
        this.shooter = shooter;
        shooter.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        shooter.configEncoderCodesPerRev(Constants.Shooter.CODES_PER_REV);
        shooter.configNominalOutputVoltage(+0.0f, -0.0f);
        shooter.configPeakOutputVoltage(+12.0f, -12.0f);
        this.agitator.setInverted(true);

    }
    public void autoPrep(int rpm) {
        shooter.changeControlMode(CANTalon.TalonControlMode.Speed);
        shooter.set(rpm);
    }
    public void autoShoot() {
        agitator.set(Constants.Shooter.AGITATOR_SPEED);
    }
    public void stopShootAuto() {
        shooter.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        shooter.set(0);
    }
    void run() {
        SmartDashboard.putNumber("Shooter Fire Rate in RPM", shooter.getSpeed());
//        SmartDashboard.put
        SmartDashboard.putNumber("Shooter Encoder", shooter.getEncVelocity()/360);
        SmartDashboard.putBoolean("Shooter ready", ready);
        SmartDashboard.putNumber("Shooter P val", shooter.getP());
        SmartDashboard.putNumber("Prefs speed", Preferences.getInstance().getInt("Shooter Speed", 1));
        SmartDashboard.putNumber("Throttle speed", (joy.getThrottle() + 1) / 2);
        if (joy.getRawButton(Constants.Shooter.SHOOT_PREP_BUTTON)) {
            shooter.changeControlMode(CANTalon.TalonControlMode.Speed);
            shooter.set(Preferences.getInstance().getInt("Shooter Speed", 1));
            ready = true;
//            shooter.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
//            shooter.set((joy.getThrottle() + 1) / 2);
        } else if (joy.getRawButton(Constants.Shooter.DISABLE_BUTTON)) {
            shooter.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            agitator.set(0.0);
            shooter.set(0.0);
            ready = false;

        }
        if (joy.getRawButton(Constants.Shooter.SHOOT_BUTTON)&&ready) {
            agitator.set(Preferences.getInstance().getDouble("Agitator", 0.65));
        } else {
            agitator.set(0.0);
        }
    }
}
