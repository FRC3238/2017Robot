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
    private int rpmCounter = 0;
    private boolean ready = false;

    Shooter(CANTalon agitator, CANTalon shooter, Joystick joy) {
        shooter.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        distSensor = new AnalogInput(Constants.Shooter.DISTANCE_PORT);
        shooter.configNominalOutputVoltage(+0.0f, -0.0f);
        shooter.configPeakOutputVoltage(+12.0f, 0.0f);
        shooter.reverseSensor(true);
        shooter.reverseOutput(false);
        shooter.setProfile(0);

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
        SmartDashboard.putNumber("Distance Sensor", distanceMap());
        ready = false;
    }

    public void autoPrep(int rpm) {
        if (rpm != -1) {
            shooter.changeControlMode(CANTalon.TalonControlMode.Speed);
            shooter.set(rpm);
            agitator.set(0);
        } else {
            double distance = distanceMap();
            SmartDashboard.putNumber("Distance Mapped", distance);
            int shootSpeedRPM = getRPM(distance);
            SmartDashboard.putNumber("Auto Shooter RPM", shooter.getSpeed());
            shooter.changeControlMode(CANTalon.TalonControlMode.Speed);
            shooter.set(shootSpeedRPM);
            agitator.set(0);
        }
    }
    public void autoShoot() {
        if(isWithinRPM(Constants.Shooter.SHOOT_RPM)) {
            agitator.set(Constants.Shooter.AGITATOR_SPEED);
        } else {
            agitator.set(0);
        }
    }

    public void printTargetSpeed() {
        SmartDashboard.putNumber("Shooter target", getRPM(distanceMap()));
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

    public void calculateFeedForward()
    {
        shooter.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        shooter.set(0.5);
        SmartDashboard.putNumber("Feed Forward", (0.5 * 1023) / ((shooter.getSpeed() / 600) * 4096));
    }

    void run() {



////        SmartDashboard.put
//        SmartDashboard.putNumber("Shooter Encoder", shooter.getEncVelocity()/360);
//        SmartDashboard.putBoolean("Shooter ready", ready);
//        SmartDashboard.putNumber("Shooter P val", shooter.getP());
//        SmartDashboard.putNumber("Prefs speed", Preferences.getInstance().getInt("Shooter Speed", 1));
//        SmartDashboard.putNumber("Throttle speed", (joy.getThrottle() + 1) / 2);
        double distance = distanceMap();
        SmartDashboard.putNumber("Distance Sensor", distance);
        int shootSpeedRPM = getRPM(distance);
        SmartDashboard.putNumber("Recieved Shoot SpeedRPM", shootSpeedRPM);
        SmartDashboard.putNumber("Shooter Encoder", shooter.getEncPosition());
        int pidSlot = Preferences.getInstance().getInt("PID Slot", 0);
        if (joy.getRawButton(Constants.Shooter.SHOOT_PREP_BUTTON))
            ready = true;
        else if(joy.getRawButton(Constants.Shooter.DISABLE_BUTTON))
            ready = false;
        if (ready) {
            shooter.changeControlMode(CANTalon.TalonControlMode.Speed);
            shooter.set(shootSpeedRPM);
            SmartDashboard.putNumber("sdiogjsoijgj", shooter.getSpeed());
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
                rpmCounter++;
            } else {
                rpmCounter = 0;
                DriverStation.reportError("Not at RPM", false);
            }
            SmartDashboard.putNumber("Hopper speed", shooter.getSpeed());

            if (rpmCounter > 4)
            {
                agitator.set(agitatorSpeed);
                SmartDashboard.putNumber("Hopper activated speed", shooter.getSpeed());
            } else {
                agitator.set(0.0);
            }
        } else if (joy.getRawButton(10) && ready) {
            SmartDashboard.putNumber("Hopper activated speed", shooter.getSpeed());
            agitator.set(agitatorSpeed);
        } else {
            agitator.set(0.0);
        }
    }
    public double getIZone() {

        return shooter.GetFirmwareVersion();
    }

    private double distanceMap()
    {
        SmartDashboard.putNumber("Throttle", joy.getThrottle());
        return ((distSensor.getAverageValue() - 14.6773) / 119.68) + 0.3333-(joy.getThrottle());
    }

    private int getRPM(double distance)
    {
        return (int) (7950 + 125 * distance + 10 * Math.pow(distance, 2));
    }
}
