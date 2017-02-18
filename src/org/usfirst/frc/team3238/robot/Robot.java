package org.usfirst.frc.team3238.robot;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team3238.robot.Autonomous.MotionProfileExample;
import org.usfirst.frc.team3238.robot.Autonomous.Profiles;

public class Robot extends IterativeRobot implements PIDOutput
{
    private Chassis chassis;
    private Collector collector;
    private Climber climber;
    private AHRS navX;
    private CANTalon leftLeader, rightLeader;
    double[][] boilerStraightFirst, boilerStraightSecond, centerLift, retrievalStraightFirst,
    retrievalStraightSecond;
    private CANTalon.MotionProfileStatus _status = new CANTalon.MotionProfileStatus();
    PIDController turnController;
    double iFix = kP;
    boolean adjusted = false;
    boolean firstStage = false, secondStage = false;;
    boolean rotateToAngle = false;
    int valid_counter = 0;
    Timer t = new Timer();
    boolean e = false;
    MotionProfileExample leftController, rightController;
    public static final double kP = 0.05,
                                kI = 0.0,
    kD = 0.02,
            kF = 0.0;
    int leftCount = 0, rightCount = 0, counterA = 0, controlStatus = 0,
            controlCalls = 0;
    public static final double fTalon = 0.415, pTalon = 1.5, kToleranceDegrees = 2.0;
    double rotateToAngleRate = 0.0;
    private int codesPerRev = 360;
    @Override public void robotInit()
    {
        leftLeader = new CANTalon(Constants.Chassis.DRIVE_TALON_ID_LEFT_A);
        CANTalon leftFollower = new CANTalon(Constants.Chassis.DRIVE_TALON_ID_LEFT_B);
        rightLeader = new CANTalon(
                Constants.Chassis.DRIVE_TALON_ID_RIGHT_A);
        CANTalon rightFollower = new CANTalon(
                Constants.Chassis.DRIVE_TALON_ID_RIGHT_B);
        CANTalon leftCollect = new CANTalon(Constants.Collector.LEFT_TALON_ID);
        CANTalon rightCollect = new CANTalon(Constants.Collector.RIGHT_TALON_ID);
        CANTalon liftCollect = new CANTalon(Constants.Collector.LIFT_TALON_ID);
        CANTalon climbTalonOne = new CANTalon(Constants.Climber.CLIMB_TALON_ONE_ID);
        CANTalon climbTalonTwo = new CANTalon(Constants.Climber.CLIMB_TALON_TWO_ID);
        Joystick joystick = new Joystick(Constants.Robot.MAIN_JOYSTICK_PORT);
        
        climber = new Climber(climbTalonOne, climbTalonTwo, joystick);
        chassis = new Chassis(leftLeader, rightLeader,  joystick);
        collector = new Collector(leftCollect, rightCollect, liftCollect,
                joystick);
        leftFollower.changeControlMode(CANTalon.TalonControlMode.Follower);
        leftFollower.set(leftLeader.getDeviceID());
        rightFollower.changeControlMode(CANTalon.TalonControlMode.Follower);
        rightFollower.set(rightLeader.getDeviceID());
        navX = new AHRS(SPI.Port.kMXP);

        leftLeader.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        leftLeader.configEncoderCodesPerRev(codesPerRev);
        rightLeader.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
        rightLeader.configEncoderCodesPerRev(codesPerRev);
        turnController = new PIDController(kP, kI, kD, kF, navX, this);
        turnController.setInputRange(-180.0f, 180.0f);

        turnController.setOutputRange(-0.51625, 0.51625);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
        boilerStraightFirst = Profiles.boilerForwardFirst.Points;
        boilerStraightSecond = Profiles.boilerForwardSecond.Points;
        setEncoderInversions();
        leftController = new MotionProfileExample(leftLeader);
        rightController = new MotionProfileExample(rightLeader);
        setEncoderInversions();
        resetEncoderPosition();

    }
    
    @Override public void disabledPeriodic()
    {
        resetEncoderPosition();
        DriverStation.reportError("Nav : " + navX.getAngle(), false);
    }
    
    @Override public void autonomousInit()
    {
        resetDiagnostics();
        setSideSpeedProfile();

        prepEncodersForProfile();

    }
    
    @Override public void autonomousPeriodic()
    {
        startMotionProfilesLoop();
        DriverStation.reportError("NavX: " + navX.getAngle(), false);
    }
    
    @Override public void teleopInit()
    {
        collector.init();
        setMotorsDriveMode();
    }
    
    @Override public void teleopPeriodic()
    {
        chassis.run();
        collector.run();
        climber.run();
        DriverStation.reportError("Left Enc Position: " + leftLeader.getEncPosition()
        + "\nRight Enc Position: " + rightLeader.getEncPosition(), false);
    }

    
    @Override public void testInit()
    {
        resetDiagnostics();

        setLayeredProfile(1);
        prepEncodersForProfile();
        valid_counter = 0;
        firstStage = false;
        secondStage = false;
        t.reset();
        iFix = 0.01;
        turnController.setPID(kP, kI, kD, kF);
        adjusted = false;
        navX.zeroYaw();
        gear = false;
    }
    boolean gear = false;
    @Override public void testPeriodic()
    {
//     chassis.proRun();
//     collector.run();
//     climber.run();
        collector.run();
        boilerGear();
        DriverStation.reportError("", false);
    }
    public void boilerGear() {
        DriverStation.reportWarning("Angle: " + navX.getAngle(), false);
        if(!firstStage && startMotionProfilesLoop()) {

//            DriverStation.reportError("First Forward", false);
            firstStage = true;
            setMotorsDriveMode();
            t.reset();
            t.start();
        }
        if(firstStage && !secondStage) {
//

//            DriverStation.reportError("Angle Turn", false);
            if(turnToDesiredAngle() > 10) {
                setLayeredProfile(2);
                secondStage = true;
                resetDiagnostics();
                prepEncodersForProfile();
                setLayeredProfile(2);
            }
        }
        if(secondStage && !gear) {

//            DriverStation.reportError("Second Forward", false);
            if(startMotionProfilesLoop()) {
                gear = true;

                collector.placeGear();
                chassis.placeGear();
            }

        }
        if(gear) {
            setMotorsDriveMode();
            collector.run();
            chassis.autoRun(0,0);
        }
    }

    @Override
    public void pidWrite(double output) {
        rotateToAngleRate = output;
    }
    Notifier _notifer = new Notifier(new PeriodicRunnable());
    class PeriodicRunnable implements java.lang.Runnable
    {
        public void run()
        {
            leftLeader.processMotionProfileBuffer();
            rightLeader.processMotionProfileBuffer();
        }
    }
    public void pollProfileStatus()
    {
        leftLeader.getMotionProfileStatus(_status);
        rightLeader.getMotionProfileStatus(_status);
    }
    public boolean startMotionProfilesLoop()
    {
        pollProfileStatus();
        DriverStation.reportWarning(
                "auto periodic: MP status = " + _status.toString(), false);

        switch(controlStatus)
        {
            case 0:
                DriverStation.reportWarning("auto case 0", false);
                _notifer.startPeriodic(0.005);
                leftController.fillFed();
                rightController.fillFed();
                leftLeader.set(CANTalon.SetValueMotionProfile.Enable.value);
                rightLeader.set(CANTalon.SetValueMotionProfile.Enable.value);
                controlStatus++;
                return e;
            default:
                DriverStation.reportWarning("auto case 1", false);
                // _talon.set(CANTalon.SetValueMotionProfile.Enable.value);
                if(!e)
                {

                    double motorOutput = leftLeader.getOutputVoltage()
                            / leftLeader.getBusVoltage();
                    double altMotorOutput = rightLeader.getOutputVoltage()
                            / rightLeader.getBusVoltage();
                    System.out.println("motor output: " + motorOutput);
                    System.out.println("motor speed: " + leftLeader.getSpeed());
                    System.out.println("error in native units: "
                            + leftLeader.getClosedLoopError());
                }


                if(_status.activePointValid && _status.activePoint.isLastPoint)
                {
                    SmartDashboard.putNumber("Loops", controlCalls);
                    DriverStation.reportWarning(
                            "auto case default: profile done", false);
                    leftLeader
                            .set(CANTalon.SetValueMotionProfile.Disable.value);
                    rightLeader
                            .set(CANTalon.SetValueMotionProfile.Disable.value);
                    DriverStation.reportWarning("# auto loops: " + controlCalls,
                            false);
                    System.out.println(leftLeader.getPosition());
                    System.out.println(rightLeader.getPosition());
                    e = true;
                }

                controlCalls++;

                return e;
        }
    }
    public void resetDiagnostics()
    {
        controlStatus = 0;
        controlCalls = 0;
        e = false;
    }
    public void prepEncodersForProfile()
    {
        resetEncoderPosition();
        enableMotionProfileMode();
        setEncoderInversions();
        setEncoderPID();
        resetDiagnostics();
    }
    public void resetEncoderPosition()
    {
        leftLeader.setPosition(0);
        rightLeader.setPosition(0);
    }
    public void enableMotionProfileMode()
    {
        leftLeader.clearMotionProfileTrajectories();
        leftLeader.changeControlMode(CANTalon.TalonControlMode.MotionProfile);
        leftLeader.set(CANTalon.SetValueMotionProfile.Disable.value);
        leftLeader.changeMotionControlFramePeriod(5);
        rightLeader.clearMotionProfileTrajectories();
        rightLeader.changeControlMode(CANTalon.TalonControlMode.MotionProfile);
        rightLeader.set(CANTalon.SetValueMotionProfile.Disable.value);
        rightLeader.changeMotionControlFramePeriod(5);

    }
    public void setEncoderInversions()
    {
        rightLeader.reverseOutput(false);
        leftLeader.reverseSensor(false);
        rightLeader.reverseSensor(true);
        leftLeader.reverseOutput(true);
    }
    public void setEncoderPID()
    {
        leftLeader.setF(fTalon);
        leftLeader.setP(pTalon);
        leftLeader.setI(0.0);
        rightLeader.setP(pTalon);
        leftLeader.setD(0.0);
        rightLeader.setF(fTalon);
        rightLeader.setI(0.0);
        rightLeader.setD(0.0);
    }
    public void setLayeredProfile(int n) {
        if(n == 1) {
            leftController.setFedProfile(boilerStraightFirst);

            rightController.setFedProfile(boilerStraightFirst);
        } else if(n == 2) {
            leftController.setFedProfile(boilerStraightSecond);


            rightController.setFedProfile(boilerStraightSecond);
        }
    }
    public void setMotorsDriveMode()
    {
        leftLeader.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        rightLeader.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    }
    public int turnToDesiredAngle() {
//        boolean done = false;
        double currentRotationRate = 0;
        if(!turnController.isEnabled()) {
            turnController.setSetpoint(-60.0f);
            turnController.enable();
        }
        if(turnController.onTarget()) valid_counter++;
        currentRotationRate = rotateToAngleRate;
        chassis.autoRun(0, -currentRotationRate);
        DriverStation.reportWarning(""+navX.getAngle(),false);
        return valid_counter;
    }
    public void setCenterLiftProfile() {
        leftController.setFedProfile(centerLift);
        rightController.setFedProfile(centerLift);

    }
    public void setRetrievalProfile() {
        leftController.setFedProfile(retrievalStraightFirst);
        rightController.setFedProfile(retrievalStraightSecond);
        navX.zeroYaw();
    }
    public void setSideSpeedProfile() {
        leftController.setFedProfile(Profiles.rightBoiler.Points);
        rightController.setFedProfile(Profiles.leftBoiler.Points);
    }
}

