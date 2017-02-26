package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Created by Team 3238 on 2/18/2017.
 */
public class Shooter {
    private CANTalon agitator, shooter;
    private Joystick joy;
    private AnalogInput distSensor;
    private boolean ready = false;

    Shooter(CANTalon agitator, CANTalon shooter, Joystick joy) {
        shooter.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        distSensor = new AnalogInput(Constants.Shooter.DISTANCE_PORT);
        shooter.configNominalOutputVoltage(+0.0f, -0.0f);
        shooter.configPeakOutputVoltage(+12.0f, 0.0f);
        shooter.reverseSensor(true);
        shooter.reverseOutput(false);

        agitator.setInverted(true);

        this.joy = joy;
        this.agitator = agitator;
        this.shooter = shooter;

    }

    public void init() {
        shooter.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        shooter.set(0.0);
        shooter.setEncPosition(0);
        SmartDashboard.putNumber("Shooter Encoder", shooter.getEncPosition());
    }

    public void autoPrep(int rpm) {
        shooter.changeControlMode(CANTalon.TalonControlMode.Speed);
        shooter.set(rpm);
        agitator.set(0);
    }
    public void autoShoot() {
        if(isWithinRPM(Constants.Shooter.SHOOT_RPM)) {
            agitator.set(Constants.Shooter.AGITATOR_SPEED);
        } else {
            agitator.set(0);
        }
    }

    public void quickShoot() {

        agitator.set(Constants.Shooter.AGITATOR_SPEED);
    }
    public void stopShootAuto() {
        shooter.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        shooter.set(0);
    }

    public void stop() {
        shooter.set(0);
        agitator.set(0);
    }
    public boolean isWithinRPM(double shootSpeed) {
        double rp = Math.abs(shootSpeed - shooter.getSpeed());
        DriverStation.reportWarning(""+rp,false);
        return  rp < Constants.Shooter.ALLOWED_RPM_ERROR;
    }
    public void runDirect() {
        shooter.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        shooter.set(Preferences.getInstance().getDouble("ShooterS", 0.4));
        if (joy.getRawButton(7))
            agitator.set(Preferences.getInstance().getDouble("Agitator", 0.65));
        else
            agitator.set(0);
    }

    public void printEnc() {
        DriverStation.reportError("Shooter ENC: " + shooter.getEncPosition(), false);
    }

    void run() {



////        SmartDashboard.put
//        SmartDashboard.putNumber("Shooter Encoder", shooter.getEncVelocity()/360);
//        SmartDashboard.putBoolean("Shooter ready", ready);
//        SmartDashboard.putNumber("Shooter P val", shooter.getP());
//        SmartDashboard.putNumber("Prefs speed", Preferences.getInstance().getInt("Shooter Speed", 1));
//        SmartDashboard.putNumber("Throttle speed", (joy.getThrottle() + 1) / 2);
        int shootSpeedRPM = Preferences.getInstance().getInt("Shooter Speed", 1);
        SmartDashboard.putNumber("Recieved Shoot SpeedRPM", shootSpeedRPM);
        SmartDashboard.putNumber("Shooter Encoder", shooter.getEncPosition());
        if (joy.getRawButton(Constants.Shooter.SHOOT_PREP_BUTTON))
            ready = true;
        else if(joy.getRawButton(Constants.Shooter.DISABLE_BUTTON))
            ready = false;
        if (ready) {
            shooter.changeControlMode(CANTalon.TalonControlMode.Speed);
            shooter.set(shootSpeedRPM);
            SmartDashboard.putNumber("sdiogjsoijgj", shooter.getSpeed());

            SmartDashboard.putNumber("Distance Sensor", distSensor.getAverageValue());
//            shooter.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
//            shooter.set((joy.getThrottle() + 1) / 2);
        } else {
            shooter.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            agitator.set(0.0);
            shooter.set(0.0);

        }
        double agitatorSpeed = Preferences.getInstance().getDouble("Agitator", 0.65);
        if (joy.getRawButton(Constants.Shooter.SHOOT_BUTTON) && ready) {
            DriverStation.reportError("" + shootSpeedRPM + ", " + shooter.getSpeed() + " " + (shootSpeedRPM - shooter.getSpeed()) + "\n" + Math.abs(shootSpeedRPM - shooter.getSpeed()), false);
            if (isWithinRPM(shootSpeedRPM)) {
                agitator.set(agitatorSpeed);
            } else {
                agitator.set(0);
                DriverStation.reportError("Not at RPM", false);
            }
        } else if (joy.getRawButton(10) && ready) {
            agitator.set(agitatorSpeed);
        } else {
            agitator.set(0.0);
        }
    }
}
